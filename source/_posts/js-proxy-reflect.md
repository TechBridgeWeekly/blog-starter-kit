---
title: 一起來了解 Javascript 中的 Proxy 與 Reflect
date: 2018-05-27 23:20:17
tags:
  - javascript
  - es6
  - proxy
  - reflect
  - frameworkless
author: arvinh
---

# 前言

在眾多 ES6 提供的新功能上，Proxy 與 Reflect 算是最少被提及的，主要原因我想還是因為瀏覽器的支援度較低，不過在我前陣子看到 [Frameworkless JavaScript Part 3: One-Way Data Binding](https://jack.ofspades.com/frameworless-javascript-part-3-one-way-data-binding/) 這篇文章時（好文推薦！很有趣），特意去查了一下才發現目前支援度已經越來越好：

![Can I Use Proxy](/img/arvinh/caniuseproxy.png)

![Can I Use Reflect](/img/arvinh/caniusereflect.png)

常用的瀏覽器幾乎都支援，我想也是可以來好好了解一下這兩個神奇的物件了！

最後有個參考該篇文章實作的 Todo app 範例，如果懶得看介紹的可以先 <a href="#todo-sample">跳下去</a> 玩玩，但若是對 Proxy 與 Reflect 不了解的人還是建議先看一下。

# Proxy

> Proxy 物件被使用於定義基本操作的自定行為（例如：尋找屬性、賦值、列舉、函式調用等等）。 - [MDN](https://developer.mozilla.org/zh-TW/docs/Web/JavaScript/Reference/Global_Objects/Proxy)

不知道為什麼唸起來有點饒口，但基本上跟其字面意思相同，就是代理（代為管理）物件行為。

Proxy 是一個函式物件（可被建構），他提供一個機會讓你能介入一般物件的基本操作行為，像是在你 assign 一個值給某個物件時，可以透過 Proxy 先進行一些 validation 等等，藉此讓使用被代理過後的物件之開發者可以專注在其他核心功能上。

咦？聽起來很像許多 framework 或 helper library 會做的事情？有趣！讓我們繼續看下去。

使用方法如下：

```js
const proxyObj = new Proxy(target, handler);
```

`target` 就是你想要代理的對象；而 `handler` 則是一個物件，其中定義了所有你想替 target 代為管理的操作定義，包含了：

* construct(target, args) - 代理 Object 的 `new` operator

* get(target, prop, receiver) - 代理 Object getting properties 時的行為

* set(target, prop, value, receiver) - 代理 Object setting properties 時的行為

* apply(target, object, args) - 代理 function call，像是 f.apply()

* has(target, prop) - 代理 `in` operator

* defineProperty(target, propKey, propDesc) - 代理 `Object.defineProperty`.

* deleteProperty(target, prop) - 代理 `delete` operator

* getOwnPropertyDescriptor(target, prop) - 代理 `Object.getOwnPropertyDescriptor`.

* getPrototypeOf(target) - 代理 `Object.getPrototypeOf`.

* setPrototypeOf(target, proto) - 代理 `Object.setPrototypeOf`.

* ownKeys(target) - 代理 `Object.getOwnPropertyNames` 與 `Object.getOwnPropertySymbols`.

* isExtensible(target) - 代理 `Object.isExtensible`.

* preventExtensions(target) - 代理 `Object.preventExtensions`.

`handler` object 所包含的 method 定義可以從 [MDN](https://developer.mozilla.org/zh-TW/docs/Web/JavaScript/Reference/Global_Objects/Proxy) 看到更多範例與描述。

## 說了這麼多，我們到底能拿 Proxy 來做什麼呢？直接來點範例吧！

### 直觀的私有變數

以往在 Javascript 中，我們可能需要透過 `closure` 來實現物件的私有變數，像是：

```js
const FooBar = function() {
  this.closeTime = 'never';
  this.setSecretDrink = (secret) => { secretDrink = secret; }
  this.getSecretDrink = () => { return secretDrink; }
}
const fooBar = new FooBar();
fooBar.setSecretDrink('Jäger Bom');
console.log(fooBar.getSecretDrink()); // 'Jäger Bom'
console.log(fooBar.closeTime); // never
console.log(fooBar.secretDrink); // undefined
```

但透過 Proxy，我們可以很直觀地在一個 Object 內達成類似效果：

```js
let FooBar = {
  _secretDrink: 'Jäger Bom',
  closeTime: 'never'
};
FooBarProxy = new Proxy(FooBar, {
  get: function(target, prop) {
    // 以底線開頭的作為私有變數
    if (prop.startsWith('_')) {
      console.log('不能存取私有變數！');
      return false;
    }
    return target[prop]; // 非私有變數，那就回傳原物件的原屬性值
  },
  set: function(target, prop, value) {
    if (prop.startsWith('_')) {
      console.log('不能修改私有變數！');
      return false;
    }
    target[prop] = value;
  },
  has: function(target, prop) {
    return prop.startsWith('_') ? false : (prop in target);
  }
});

FooBarProxy._secretDrink; // 不能存取私有變數！
console.log(FooBarProxy.closeTime); // never
FooBarProxy._secretDrink = 'Cola'; // 不能修改私有變數！
console.log('_secretDrink' in FooBarProxy); // false
console.log('closeTime' in FooBarProxy); // true
```

眼尖一點的讀者可能會發現，這邊 handler 裡面的 `get`、`set` 好像跟上面定義中的參數不同，少了 `receiver` 這個參數？

沒錯，這個神奇的第三個參數其實是指向你產生的 Proxy 實例，以上面例子來看就是 `FooBarProxy` 本身，由於範例中用不到，所以不宣告也沒關係，不過晚點在 `Reflect` 的介紹會再度提起。

另外，若是沒有被你代理到的操作，則會直接 fallback 回原始 target 物件的操作上。

### 在設置物件屬性前進行 Validation

延續剛剛的例子，我們的 FooBar 除了秘密飲料外，也需要紀錄一下基本資訊，像是電話、地址等等，這時候 Proxy 就能為我們帶來另一個好處：驗證屬性值：

```js
let FooBar = {
  _secretDrink: 'Jäger Bom',
  closeTime: 'never',
  phoneNumber: '02-2849-2839'
};
FooBar = new Proxy(FooBar, {
  set: function(target, prop, value) {
    if (prop === 'phoneNumber') {
      // phone number validation
      var re = /^\(?\d{2}\)?[\s\-]?\d{4}\-?\d{4}$/;
      if (!re.test(value)) {
        throw Error(`Cannot set ${prop} to ${value}. Wrong format. Should be xx-xxxx-xxxx`);
      }
    }
    target[prop] = value;
  },
  //..
});
```

![proxy example validation](/img/arvinh/proxy-validationexample.png)

只要你設置的 `phoneNumber` 不符合 regex 的規則，就會拋出一個 Error 告訴開發者，此物件的 `phoneNumber` 屬性值是有固定 format 的。

當然，javascript 充滿彈性，你也可以有彈性一點的寫法，把 validator 抽離出來：

```js
var BarValidator = {
  _secretDrink: function (value) {
    if (value === 'cola') {
      throw Error('lame...');
    }
  },
  phoneNumber: function(value) {
    var re = /^\(?\d{2}\)?[\s\-]?\d{4}\-?\d{4}$/;
    if (!re.test(value)) {
      throw Error(`Cannot set phoneNumber to ${value}. Wrong format. Should be xx-xxxx-xxxx`);
    }
  }
};

FooBar = new Proxy(FooBar, {
  set: function(target, prop, value) {
    BarValidator[prop](value);
    target[prop] = value;
  },
  //..
});
```

### 用來設定屬性預設值

```js
// 實際上沒有這個屬性
console.log(FooBar.revenue); // undefined

// 但經過 Proxy 後
FooBar = new Proxy(FooBar, {
  get: function(target, prop, value) {
    if(prop === 'revenue') {
      return 'None of your business';
    }
    return target[prop];
  },
  //..
});
// 可以讀取到我們設定的預設值
console.log(FooBar.revenue); // 'None of your business'
```

### 複寫原有物件，讓測試更加順利（mock object）

寫測試的時候很常會需要 mock object，像是 function 中若有讀取 `document.location.href` 的部分，在你開發機上基本上都會是 `localhost`，這時候就會需要把這個值 mock 掉。

這時我們就可以將 `document.location` 委託給 proxy 代理：

```js
const mockDocument = {
  location: new Proxy(document.location, {
      get: function(target, prop) {
          if (prop == "href")
              return "your-website-com";
          return target[prop];
      }
  })
};
console.log("location href: ", mockLocation.location.href); // https://blog.arvinh.info
```

### 看到這邊想必很多人都會想到，我們可以實作 Observe function！

```js
function observe(o, callback) {
  return new Proxy(o, {
    set(target, property, value) {
      callback(property, value);
      target[property] = value;
    },
  });
}
const FooBar = { open: false };
const FooBarObserver = observe(FooBar, (property, value) => {
  property === 'open' && value ? alert('FooBar is open!!!') : console.log('keep waiting');
});
FooBarObserver.open = true;
```

### 不是什麼都可以被代理的

不知道大家會不會有個疑問，難道所有物件都能被 proxy 代理嗎？有沒有辦法限制我的某個物件就是不希望被他人代理？

當然有！

如果你的物件擁有 `configurable: false` 與 `writable: false` 的屬性，那該物件就無法被 proxy 代理：

```js
const target = Object.defineProperties({}, {
  FooBar: {
    writable: false,
    configurable: false
  },
});
const handler = {
  get(target, propKey) {
    return '???';
  }
};
const proxy = new Proxy(target, handler);
proxy.FooBar
// Uncaught TypeError: 'get' on proxy: property 'FooBar' is a read-only and non-configurable data property on the proxy target but the proxy did not return its actual value (expected 'undefined' but got '???')
```

### 小結論

這邊我只列了幾個我覺得比較能凸顯 Proxy 用途的範例，而其他 handler 可以介入的操作如果大家也想了解並看看例子的話，阮一峰的 [ECMAScript 6 入門](http://es6.ruanyifeng.com/#docs/proxy) 中有針對每個操作給予例子做解析，可以參考。

# Reflect

接著我們來看看 Reflect。Reflect 不能建構實例，就像 Math 一樣，單純包含了一系列的靜態方法。

## Reflect 與 Proxy 的完美搭配

網路上許多文章都說 Reflect 是因應 Proxy 才增加的規範，最明確的連結是，Reflect 所定義的靜態方法包含了 Proxy Handler 能處理的所有代理操作，但他提供的是呼叫原始物件的操作，舉例來說：

`Reflect.get(target, name);` 效果等同於 `target[name];`

所以我們在 Proxy 中，如果需要 target 物件的預設操作，使用 Reflect 會更合理更清楚：

```js
const loggedObj = new Proxy(obj, {
  get: function(target, name) {
    console.log("get", target, name);
    return Reflect.get(target, name);
  }
});
```

### 主要的理由在於，Reflect 讓我們對物件的操作可以用函數來處理

例如在判斷物件有無特定屬性，或是刪除物件屬性時，以往我們會這樣做：

```js
'_secretDrink' in FooBar;
delete Object._secretDrink;
```

有了 Reflect 我們可以這樣做：

```js
Reflect.has(FooBar, '_secretDrink');
Reflect.deleteProperty(FooBar, '_secretDrink');
```

因此，在 Proxy 中，比起使用 `delete target[name]`, `Reflect.deleteProperty` 更能保持一制性：

```js
const loggedObj = new Proxy(obj, {
  deleteProperty: function(target, name) {
    // instead of `delete target[name]...
    return Reflect.deleteProperty(target, name);
  }
});
```

### 控制被 Proxy 代理的函數之 this 參考對象

這個例子比較難懂，但這是說明為何 Reflect 是因應 Proxy 而生的好例子([source](https://stackoverflow.com/questions/35276559/benefits-of-es6-reflect-api))：

```js
const target = {
    get foo() {
        return this.bar;
    },
    bar: 3
};
const handler = {
    get(target, propertyKey, receiver) {
        if (propertyKey === 'bar') return 2;
        console.log('Reflect.get ', Reflect.get(target, propertyKey, receiver)); // this in foo getter references Proxy instance; logs 2
        console.log('target[propertyKey] ', target[propertyKey]); // this in foo getter references "target" - logs 3
    }
};
const obj = new Proxy(target, handler);
console.log(obj.bar);
// 2
obj.foo;
// Reflect.get  2
// target[propertyKey]  3
```
<a class="jsbin-embed" href="http://jsbin.com/bimadip/2/embed?js,console">JS Bin on jsbin.com</a><script src="http://static.jsbin.com/js/embed.min.js?4.1.4"></script>

假設你的 object target 有一個 getter 函數 foo()，現在你透過 Proxy 代理 get 函數，當今天你呼叫 `obj.bar` 時，會印出 `2`，因為 Proxy handler 攔截並代理了原始 target 物件的 get 函數；接著，若你呼叫 `obj.foo`，會出現兩個結果: `Reflect.get  2` 與 `target[propertyKey]  3`。

為什麼？

這是因為只有透過 `Reflect.get()` 的第三個參數 `receiver`，將指向 Proxy 本身的實例傳進去原始物件的 get 呼叫，才能夠真的呼叫到 Proxy.get。

若是直接透過 `target['foo']`，則原本在 `foo` 中的 this，就會指向原始的 target 本身，而不會觸發 Proxy 的 get。

這邊概念真的比較難懂，若我有任何錯誤地方歡迎指正，我相信大家多看幾次範例後都能悟道的。

除了與 Proxy 匹配的優勢外，Reflect 還帶來了一些好處（source: [Benefits of ES6 Reflect API](https://goo.gl/9v9STM), [Harmony-reflect](https://github.com/tvcutsem/harmony-reflect/wiki))：

### 更優雅、更好用的回傳值

以往使用 `Object.defineProperty(obj, name, desc);` 時，若成功，會回傳 obj，失敗則有可能會拋出 Error。而使用 `Reflect.defineProperty(obj, name, desc)` 的話，則會回傳 boolean 值，讓失敗或成功的結果有統一的格式。

<!-- 介紹其他更多好處 https://github.com/tvcutsem/harmony-reflect/wiki-->
### 更可靠的 `apply`

在 es5 時，大家都很習慣透過 `f.apply(obj, args)` 的方式來 apply 函數到物件上頭，但很有可能在某些情況下，`f.apply` 被串改了，這時候就會有不預期的結果。

Senior 一點的會知道可以利用 `Function.prototype.apply.call(f, obj, args)` 來呼叫，至少 prototype 不會騙你，但這種方式總是不夠優雅。

現在有了 Reflect 後，就不需要擔心這種事情，透過 `Reflact.apply(obj, args)` 就能輕鬆達到一樣效果。

### 接受可變參數的 Constructor

這個優點只有跟 ES5 比較時才有優勢。主要是讓你能透過：`const obj = Reflect.construct(FooBar, args)` 來在建構物件實例時，傳遞可變參數；若是在 ES5 的世界，只有 `FooBar.apply` 或 `FooBar.call` 能夠接受變動參數，但是在 `new` 物件實例時，並沒有 `apply` 或 `call` 可以使用。

而現在透過 ES6 的 spread syntax，我們可以在建構物件實例時，直接傳遞可變參數：`const obj = new FooBar(...args)`。

<!-- 最後說明 https://jack.ofspades.com/frameworkless-javascript-part-3-one-way-data-binding/ 中 data binding 的實作-->
<span id="todo-sample"></span>

## 最終範例：利用 Proxy 與 Reflect 完成 one way data binding

在了解完 Proxy 與 Reflect 的基本使用方式後，想分享一個很有趣的應用，也就是我開頭提到，激發我研究 Proxy 的範例：frameworkless js one way data binding.

結合先前提過的 Observe function，來實作一個簡單 Todo App：

<p data-height="339" data-theme-id="dark" data-slug-hash="LrpOEw" data-default-tab="js,result" data-user="arvin0731" data-embed-version="2" data-pen-title="Oneway-data-binding-js-proxy-reflect" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/LrpOEw/">Oneway-data-binding-js-proxy-reflect</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

基本上是結合了上述介紹的 Proxy 與 Reflect 特性，並融合[這篇文章]((https://jack.ofspades.com/frameworkless-javascript-part-3-one-way-data-binding/)的範例所製作的，如果大家有看完前面的介紹，對於這段 code 應該不難理解。

主要是透過 Proxy 來代理 Object 的 `set` 與 `deleteProperty` 操作，讓 Todo list 的變動能夠被代理。

此外，在先前的介紹中，都是以 Object 為主，但 todo app 範例中被 Proxy 代理的是 Array。

最大的差別在於，當你 push 一個新 item 進入 Array 時，`set` 會被呼叫兩次，一次是新的 item 被塞入陣列時，一次是 Array 的 `length` property 加一時。所以要特別濾掉 `length` 更動的那次代理操作。

最後，只要在代理的操作中，想辦法把 DOM 做對應的修改，如同上面程式中的 `line 22 ~ line 40`，定義一些 render template 的 function 來更新 DOM 即可。

<!-- 最終結論 -->

# 結論

Javascript 的變動總是快於瀏覽器支援度，所以常常造成一些新的 Spec 我們不熟悉、不知如何運用，這次的研究學習了不少，而上面的 Todo App 是個簡陋不嚴謹的範例，不過也足以展現 Proxy 與 Reflect 在實際運用上的情境，並帶給我們另一種思考方向，很多時候不用一開始就套用 Framework，透過越來越進步的瀏覽器與 ES 版本，我們也能達到一樣目的。雖然比不上 framework 包山包海的優化，但或許能讓我們更了解實際要解決的問題是什麼，以及解決方法背後的概念。

<!-- 資料來源 -->

## 資料來源

1. [Frameworkless JavaScript Part 3: One-Way Data Binding](https://jack.ofspades.com/frameworkless-javascript-part-3-one-way-data-binding/)
2. [MDN Proxy](https://developer.mozilla.org/zh-TW/docs/Web/JavaScript/Reference/Global_Objects/Proxy)
3. [ES6 之 Proxy 介绍](https://www.jianshu.com/p/34f0e6abe312)
4. [ECMAScript 6 入门](http://es6.ruanyifeng.com/#docs/proxy)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化