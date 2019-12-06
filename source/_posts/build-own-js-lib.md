---
title: '[筆記] JavaScript: Understanding the Weird Parts - Build your own lib/framework'
date: 2018-05-05 18:44:38
tags:
    - javascript
---

之前趁著 Udemy 特價，買了上面很有名的課程 - JavaScript: Understanding the Weird Parts，當初會想買是因為他最後面有個章節是介紹如何建構自己的 JS Framework，
而我一直都很想有系統性地去瞭解建構一個可供大家使用的 JS library 或是 framework 需要注意哪些事項，該怎麼寫才是安全有彈性的結構。

後來大約花了三天的時間斷斷續續把課程上完，這邊紀錄一下該章節的一些筆記，並實做一個小小的 js library 當範例。

### Goal

目標是建構一個 js library，可以將數字做一些簡化約分，像是我在 [Web Component 實戰](http://blog.techbridge.cc/2017/03/04/webcomopnent-practice/) 中所實作的 function。並讓 user 透過 `<script src="formatNum.js" />` 的方式就能載入使用。

完整的 code 在下面這個 jsbin 中：

<a class="jsbin-embed" href="http://jsbin.com/xuvipaj/1/embed?js,console">JS Bin on jsbin.com</a><script src="http://static.jsbin.com/js/embed.min.js?4.1.4"></script>

可以試著在 console 中輸入以下指令來看看結果：
`var f = F$('1E9')`;
`f.format().log()`;
`f.format('scientific').log()`

### Structuring Safe Code

要建構一個安全的 JS lib，勢必需要保護好自己的 scope 不受外部影響，也不會去影響外部。要做到這件事最簡單的方式就是使用一個 IIFE (Immediately Invoked Function Expression)，

```js
;(function(global) {
    
    
}(window));
```

而我們需要將 lib 能夠 export 到外部供人使用，所以在這個立即執行函式中需要傳入 window 物件，在函式內我們則取名為 global，這樣未來如果想要執行在不同環境，像是 nodejs 裡面時，可以不用更改內部的變數名稱，只要修改傳入的 window 變數即可。

另外最前方可以加上個分號，以免有其他人的 code 沒有用分號做結尾而造成問題，不過這個並不是必須的。

在這個立即執行函式中的變數除非我們刻意 export 出去，否則都只存在於自己的作用域內，是個安全的結構。

下方的 `supportedUnit` 與 `unit` 在外部都無法存取，無法透過 `FormaNum.unit` 取得。

```js
;(function(global) {
     // hidden within the scope of the IIFE and never directly accessible
    const supportedUnit = ['normal', 'scientific'];
    
    const unit = {
        normalUnit: [
          { value: 1000000000,  symbol: "B" },
          { value: 1000000,  symbol: "M" },
          { value: 1000,  symbol: "k" }
        ],
        siUnit: [
          { value: 1E18, symbol: "E" },
          { value: 1E15, symbol: "P" },
          { value: 1E12, symbol: "T" },
          { value: 1E9,  symbol: "G" },
          { value: 1E6,  symbol: "M" },
          { value: 1E3,  symbol: "k" }
        ],
    }

    // ...
    
}(window));
```

### Object, Prototype and Properties

接著就是開始實作我們的 lib 內容了，這門課程中，有帶著我們了解 JQuery 的 source code，看看這個偉大的 lib 是如何架構其內部程式，其中很特別的地方在於它 new 一個物件的方式，
通常我們載入一個別人寫好的物件，或是我們自己寫好了一個物件，要使用的時候會需要透過 `const objectInstanc = new Object()` 的方式來產生物件實例，但為何我們使用 JQuery 的時候都不需要特別使用 new 關鍵字呢？

因為在 JQuery 中，他透過下面的方式來幫你在每次使用它時自動 new 了一個物件：

```js
    // 'new' an object
    const FormatNum = function(num, digits, unit) {
        return new FormatNum.init(num, digits, unit);   
    }

    //...

    // the actual object is created here, allowing us to 'new' an object without calling 'new'
    FormatNum.init = function(num, digits, unit) {
        
        const self = this;
        self.num = num || '';
        self.digits = digits || '';
        self.unit = unit || 'normal';
        
        self.validate();
        
    };
```

這時候你可能會想說，這樣的寫法，不就代表我要加 method 到 prototype 的話，都是要加在 `FormatNum.init.prototype` 了嗎？ 這樣有點奇怪耶，畢竟我的 lib 是叫做 FormatNum呀！

沒錯，所以我們可以將 FormatNum.init.prototype 在指定到 FormatNum.prototype 上：

```js
    FormatNum.init.prototype = FormatNum.prototype;
```

透過短短這兩個步驟，我們就能夠不需要自己 new object，同時又能直接在 FormatNum 上面設置 prototype method！

另外，透過在每個 method 的最後 return this，就能讓我們的 function chainable。

```js
// prototype holds methods (to save memory space)
    FormatNum.prototype = {
        
        validate: function() {
            ///
        },
      
        calculate: function(unitType) {
          ///
        },
      
        formatScientific: function() {
          return this.calculate('siUnit');
        },
        
        formatNormal: function() {
          return this.calculate('normalUnit');
        },

        // chainable methods return their own containing object
        format: function(unit) {
            let formattedNum;
            
            // if undefined or null it will be coerced to 'false'
            if (unit === 'scientific') {
                formattedNum = this.formatScientific();  
            }
            else {
                formattedNum = this.formatNormal();  
            }
          
            this.formattedNum = formattedNum;

            // 'this' refers to the calling object at execution time
            // makes the method chainable
            return this;
        },
        
        log: function() {
            if (console) {
                console.log('formattedNum is: ' + this.formattedNum); 
            }
        },
        
    };
```

### export to outside world

最後我們只要加上 `global.FormatNum = global.F$ = FormatNum;`

就可以在外部使用 `FormatNum` 或是 `F$` 來呼叫我們的 lib 了！

### 結論與小問題

建造 js lib 的概念不難，只是如果很少開發的話，確實容易忘記一些眉眉角角，透過這次的文章也算是稍稍再複習了一下先前課程的內容。
另外，在實作範例時，本來想直接全用 ES6 寫（課程主要都是 ES5），但是在這邊的 function 都不能用 ES6 的 arrow function 取代，`this` 的作用域不同，會造成問題。[這篇有提到](https://derickbailey.com/2015/09/28/do-es6-arrow-functions-really-solve-this-in-javascript/)，arrow function 會 binding 到整個 module 的 scope，而非 object。所以如果是想透過 ES6 來撰寫的話，應該是需要換另一種寫法，之後找到好作法再來補上。


## 資料來源
1. [JavaScript: Understanding the Weird Parts](https://www.udemy.com/understand-javascript/learn/v4/)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化