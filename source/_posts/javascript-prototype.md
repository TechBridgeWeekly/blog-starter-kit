---
title: 該來理解 JavaScript 的原型鍊了
date: 2017-04-22 11:53:31
tags:
  - JavaScript
  - prototype
author: huli
---

## 前言

老實說 JavaScript 的原型鍊一直是我很懼怕的一個主題，理由很簡單，因為真的不太好理解。光是一堆名詞跟錯綜複雜的關係就可以把你搞瘋，例如說`prototype`, `__proto__`, `constructor`, `Object.prototype`, `Function.prototype`, `new`等等。

可是呢，這又確實是 JavaScript 很重要的一部分，而且是面試的必考題，就算現在不懂，以後遲早有一天要把它弄懂，不然的話永遠都沒辦法把自己的技術能力往上提高一個檔次。

有關原型鍊的文章你可以在網路上搜到一大堆，每一篇的理解方式都不太一樣，有些直接搬出一大堆專有名詞，嚇都把你嚇死了。而我也是一直到最近，看了幾篇我覺得切入角度比較不錯的文章，才真正對原型鍊有比較深刻的理解。

就趁著現在這個機會，讓我們多瞭解一點 JavaScript 的原型鍊吧！這篇適合對 JavaScript 有一點概念但又不是很清楚的人觀看，如果文章中有講錯的地方，也麻煩不吝在評論中指出，感謝。

## JavaScript 中的 class

要理解原型鍊，可以先從這兩篇我覺得很棒的切入角度開始：

1. [Javascript继承机制的设计思想](http://www.ruanyifeng.com/blog/2011/06/designing_ideas_of_inheritance_mechanism_in_javascript.html)
2. [从设计初衷解释 JavaScript 原型链](http://www.jasonsi.com/2017/03/15/36/)

這兩篇講到為什麼當初 JavaScript 的機制是這樣設計的，我認為從這個角度開始理解，會是一個比較好的開始。（強烈建議先看過這兩篇之後再往下看，會幫助你更瞭解原型鍊到底是什麼東西）

首先呢，JavaScript 不像 Java 或是其他物件導向的程式語言，它是沒有 class 的（ES6 的 class 也只是語法糖而已）。可是儘管沒有 class，卻還是可以設計出一個類似的機制來達成差不多的功能。

在 Java 裡面，如果你要從 class 生出一個 instance 的話，你可以這樣寫：

``` java
Point p = new Point();
```

於是 JavaScript 就把這個語法拿來用，有了`new`這個關鍵字。可是 JavaScript 又沒有 class，`new`後面要接什麼呢？

這時候他就想到，每一個 class 在初始化的時候，不是都會呼叫 constructor 嗎？也就是構造函數，那在 JavaScript 裡面，後面就接構造函數吧！

於是，下面的程式碼就很好理解了：

``` js
// constructor
function Person(name, age) {
  this.name = name;
  this.age = age;
}
  
var nick = new Person('nick', 18);
var peter = new Person('peter', 18);
```

就如同上面講到的一樣，`Person`就是一個構造函數，可以用`new`這個關鍵字 new 出一個 instance 來。

如果你只看下面宣告 nick 那一行（`var nick = new Person('nick', 18);`），語法是不是跟你在寫 Java 的時候有 87 分像？除此之外，你也可以幫`Person`加入一些方法。

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
  this.log = function () {
    console.log(this.name + ', age:' + this.age);
  }
}
  
var nick = new Person('nick', 18);
nick.log(); // nick, age:18
  
var peter = new Person('peter', 20);
peter.log(); // peter, age:20
```

可是這樣其實還有一個小問題， name 跟 age 這兩個屬性，很明顯是每一個 instance 都會不一樣的。可是 log 這個 method，其實是每一個 instance 彼此之間可以共享的，因為都在做同一件事情。

在現在這種情況下，雖然 nick 的 log 這個 function 跟 peter 的 log 這個 function 是在做同一件事，但其實還是佔用了兩份空間，意思就是他們其實是兩個不同的 function。

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
  this.log = function () {
    console.log(this.name + ', age:' + this.age);
  }
}
  
var nick = new Person('nick', 18);
var peter = new Person('peter', 20);
  
console.log(nick.log === peter.log) // false
```

那怎麼辦呢？我們可以把這個 function 抽出來，變成所有 Person 都可以共享的方法。講到這邊，你應該有聽過一個東西叫做`prototype`。只要把 log 這個 function 指定在 `Person.prototype` 上面，所有 `Person` 的 instance 都可以共享這個方法。

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
}
  
Person.prototype.log = function () {
  console.log(this.name + ', age:' + this.age);
}
  
var nick = new Person('nick', 18);
var peter = new Person('peter', 20);
  
console.log(nick.log === peter.log) // true
  
// 功能依舊跟之前一樣
nick.log(); // nick, age:18
peter.log(); // peter, age:20
```

有些人會直接在 `Array.prototype` 上面加一些函式，讓自己可以更方便地做一些操作，原理也是這樣。可是一般來說，不推薦直接去修改不屬於你的 Object。

``` js
Array.prototype.last = function () {
    return this[this.length - 1];
};
  
console.log([1,2,3].last()) // 3
```

最後幫大家總結一下，上面這一段其實主要是幫大家複習一下 JavaScript 的一些基礎。

你有一個叫做`Person`的函數，就可以把`Person`當作 constructor，利用`var obj = new Person()`來 new 出一個`Person`的 instance，並且可以在`Person.prototype`上面加上你想讓所有 instance 共享的屬性或是方法。

## 探究原理

不知道你會不會好奇一件事，以上面`var nick = new Person('nick', 18);`的例子來說，當我在呼叫`nick.log()`的時候，JavaScript 是怎麼找到這個 function 的？

因為 nick 這個 instance 本身並沒有 log 這個 function。但根據 JavaScript 的機制，nick 是 Person 的 instance，所以如果在 nick 本身找不到，它會試著從`Person.prototype`去找。

可是，JavaScript 怎麼知道要到這邊去找？所以一定是 nick 跟`Person.prototype`會透過某種方式連接起來，才知道說要往哪邊去找 log 這個 function。

而這個連接的方式，就是`__proto__`。
（附註：其實比較好的方式是用`Object.getPrototypeOf()`，但這邊為了方便起見，還是使用比較常見的`__proto__`，更詳細的說明可參考：[MDN: Object.prototype.__proto__](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Object/proto)）

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
}
  
Person.prototype.log = function () {
  console.log(this.name + ', age:' + this.age);
}
  
var nick = new Person('nick', 18);
  
console.log(nick.__proto__ === Person.prototype) // true
```

nick 的`__proto__`會指向`Person.prototype`，所以在發現 nick 沒有 log 這個 method 的時候，JavaScript 就會試著透過`__proto__`找到`Person.prototype`，去看`Person.prototype`裡面有沒有 log 這個 method。

那假如`Person.prototype`還是沒有呢？那就繼續依照這個規則，去看`Person.prototype.__proto__`裡面有沒有 log 這個 method，就這樣一直不斷找下去。找到時候時候為止？找到某個東西的`__proto__`是 null 為止。意思就是這邊是最上層了。

而上面這一條透過`__proto__`不斷串起來的鍊，就叫做原型鍊。透過這一條原型鍊，就可以達成類似繼承的功能，可以呼叫自己 parent 的 method。

看下面這段程式碼你大概會有一些感覺：

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
}
  
Person.prototype.log = function () {
  console.log(this.name + ', age:' + this.age);
}
  
var nick = new Person('nick', 18);
  
// 這個剛講過了，nick.__proto__ 會指向 Person.prototype
console.log(nick.__proto__ === Person.prototype) // true
  
// 那 Person.prototype.__proto__ 會指向誰呢？會指向 Object.prototype
console.log(Person.prototype.__proto__ === Object.prototype) // true
  
// 那 Object.prototype.__proto__ 又會指向誰呢？會指向 null，這就是原型鍊的頂端了
console.log(Object.prototype.__proto__) // null
```

如果想知道一個屬性是存在 instance 身上，還是存在於它屬於的原型鍊當中，可以用`hasOwnProperty`這個方法：

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
}
  
Person.prototype.log = function () {
  console.log(this.name + ', age:' + this.age);
}
  
var nick = new Person('nick', 18);
console.log(nick.hasOwnProperty('log')); // false
console.log(nick.__proto__.hasOwnProperty('log')); // true
```

有了`hasOwnProperty`之後，我們就可以自己來模擬這段往上找的過程：

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
}
  
Person.prototype.log = function () {
  console.log(this.name + ', age:' + this.age);
}
  
var nick = new Person('nick', 18);
  
function call(obj, methodName) {
  var realMethodOwner = obj;
  
  // 不斷往上找，直到 null 或者是找到真的擁有這個 method 的人為止
  while(realMethodOwner && !realMethodOwner.hasOwnProperty(methodName)) {
    realMethodOwner = realMethodOwner.__proto__;
  }
  
  // 找不到就丟一個 error，否則執行這個 method
  if (!realMethodOwner) {
    throw 'method not found.';
  } else {
    realMethodOwner[methodName].apply(obj);
  }
}
  
call(nick, 'log'); // nick, age:18
call(nick, 'not_exist'); // Uncaught method not found.
```

做到這邊，其實你已經對原型鍊有了比較深刻的了解了。

來考你一題，`Person.__proto__`會是什麼？

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
}
  
Person.prototype.log = function () {
  console.log(this.name + ', age:' + this.age);
}
  
var nick = new Person('nick', 18);
  
console.log(Person.__proto__ === Function.prototype); // true
console.log(Function.prototype.__proto__ === Object.prototype) // true
console.log(Object.prototype.__proto__); //null
```

因為`Person`其實就是個 Function 的 instance，所以`Person.__proto__`當然就是`Function.prototype`囉！

## instanceof

顧名思義，`A instanceof B` 就是拿來判斷 A 是不是 B 的 instance，舉例來說：

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
}
  
Person.prototype.log = function () {
  console.log(this.name + ', age:' + this.age);
}
  
var nick = new Person('nick', 18);
  
console.log(nick instanceof Person); // true
console.log(nick instanceof Object); // true
console.log(nick instanceof Array); // false
```

從範例中可以看出，只要能在 A 的原型鍊裡面找到 B 的 prototype，就會回傳 true。知道原理之後，我們也可以來簡單模擬一下 instnaceof 在做的事：

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
}
  
Person.prototype.log = function () {
  console.log(this.name + ', age:' + this.age);
}
  
var nick = new Person('nick', 18);
  
function instanceOf(A, B) {
  
  // 已經找完了
  if (!A) return false;
  
  // 沒找到的話，繼續往上找
  return A.__proto__ === B.prototype ? true : instanceOf(A.__proto__, B);
}
  
console.log(instanceOf(nick, Person)); // true
console.log(instanceOf(nick, Object)); // true
console.log(instanceOf(nick, Array)); // false
```

而 instanceof 有一個很有趣的現象，那就是：

``` js
// 這兩個互為彼此的 instance
console.log(Function instanceof Object); // true
console.log(Object instanceof Function); // true
  
// Function 的 __proto__ 會指向 Function.prototype
// 而 Function.prototype 的 __proto__ 會指向 Object.prototype
console.log(Function.__proto__ === Function.prototype); // true
console.log(Function.__proto__.__proto__ === Object.prototype); //true
  
// Object 的 __proto__ 會指向 Function.prototype
console.log(Object.__proto__ === Function.prototype); // true
```

這個東西又會把問題搞得更複雜，在這邊就先不提了。如果想知道的話，可以參考下面這兩篇文章：

1. [从__proto__和prototype来深入理解JS对象和原型链](https://github.com/creeperyang/blog/issues/9)
2. [理解JavaScript的原型链和继承](https://blog.oyanglul.us/javascript/understand-prototype.html)

## constructor

順帶一提，每一個 prototype 都會有一個叫做`constructor`的屬性，例如說`Person.prototype.constructor`，而這個屬性就會指向構造函數。`Person.prototype`的構造函數是什麼？當然就是`Person`囉。

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
}
  
Person.prototype.log = function () {
  console.log(this.name + ', age:' + this.age);
}
  
var nick = new Person('nick', 18);
  
// 這段是要讓大家知道，這邊其實是往原型鍊的上面去找
console.log(nick.constructor === Person); // true
console.log(nick.hasOwnProperty('constructor')); // false
  
// Person 的 constructor 就是 Person
console.log(Person.prototype.constructor === Person); // true
console.log(Person.prototype.hasOwnProperty('constructor')); // true
```

所以其實`constructor`也沒什麼好講的，`A.prototype.constructor === A`，你把 A 用 `Function`, `Person`, `Object` 之類的值帶進去都成立。

有一個比較有趣的地方是，你可以透過這樣的方式來執行一段程式碼：`[].slice.constructor('alert(1)')()`。原理其實就是把`Function('alert(1)')()`的`Function`用`[].slice.constructor`來取代掉。

## new

有了原型鍊的概念之後，就不難理解`new`這個關鍵字背後會做的事情是什麼。

假設現在有一行程式碼是：`var nick = new Person('nick');`，那它有以下幾件事情要做：

1. 創出一個新的 object，我們叫它 O
2. 把 O 的 `__proto__` 指向 Person 的 prototype，才能繼承原型鍊
3. 拿 O 當作 context，呼叫 Person 這個建構函式
4. 回傳 O

我們可以寫一段程式碼來模擬這個情形：

``` js
function Person(name, age) {
  this.name = name;
  this.age = age;
}
  
Person.prototype.log = function () {
  console.log(this.name + ', age:' + this.age);
}
  
function newObj(Constructor, arguments) {
  var o = new Object();
  
  // 讓 o 繼承原型鍊
  o.__proto__ = Constructor.prototype;
  
  // 執行建構函式
  Constructor.apply(o, arguments);
  
  // 回傳建立好的物件
  return o;
}
  
var nick = newObj(Person, ['nick', 18]);
nick.log(); // nick, age:18
```

延伸閱讀：[JS 对象机制深剖——new 运算符](http://www.cnblogs.com/aaronjs/archive/2012/07/04/2575570.html)

## 總結

今天不但更理解了原型鍊到底是什麼東西，也寫了一些簡單的小程式來模擬 JavaScript 在查找原型鍊的過程。藉由自己實作這些機制之後，應該會對原型鍊有更多的一些理解。

在 JavaScript 這個程式語言當中，就是透過原型鍊這樣子的機制，把上下關係給串起來，當你在 A 找不到某個東西的時候，就可以到 A 的 parent（也就是 `A.__proto__`）去找，還是找不到的話就再往上。而原型鍊的盡頭就是`Object.prototype`，再往上找就是`null`了。

在寫這篇文章的時候參考了許多資料，我都附在下面了。有些文章會附上精美圖片，但我覺得從圖片開始反而會有點霧煞煞，因為不知道彼此之間的關聯是怎麼來的。

建議大家看完這篇之後可以看一下底下那些參考資料，也順便複習一下自己的觀念是否正確。

## 參考資料

1. [JavaScript深入之从原型到原型链](https://github.com/mqyqingfeng/Blog/blob/master/JavaScript%E6%B7%B1%E5%85%A5%E4%B9%8B%E4%BB%8E%E5%8E%9F%E5%9E%8B%E5%88%B0%E5%8E%9F%E5%9E%8B%E9%93%BE.md)
2. [JS原型链图解教程](https://www.talkingcoder.com/article/6360227501704156372)
3. [理解JavaScript的原型链和继承](https://blog.oyanglul.us/javascript/understand-prototype.html)
4. [从__proto__和prototype来深入理解JS对象和原型链](https://github.com/creeperyang/blog/issues/9)
5. [Javascript 原型链](http://zencode.in/2.Javascript%E5%8E%9F%E5%9E%8B%E9%93%BE.html)
6. [彻底理解JavaScript原型](http://www.imooc.com/article/2088)

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好