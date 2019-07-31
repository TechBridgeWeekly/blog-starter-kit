---
title: 淺談 JavaScript 頭號難題 this：絕對不完整，但保證好懂
date: 2019-02-23 10:39:15
tags:
  - javascript
  - this
author: huli
---

# 前言

在 JavaScript 裡面，有一個令新手十分頭痛，老手也不一定能完全理解的主題：「this 是什麼？」。身為一個以 JavaScript 當作吃飯工具的前端工程師，我也被這個問題困擾了許久。

我原本以為我這輩子都不會寫有關於 this 的文章。

原因有兩個，第一個是講解 this 的文章已經超級無敵多了，而且每一篇都寫得很不錯，之前看完 [What's THIS in JavaScript ?](https://kuro.tw/posts/2017/10/12/What-is-THIS-in-JavaScript-%E4%B8%8A/) 系列之後覺得講解的很完整，若是沒有把握自己能夠講得更清楚或是以不同的角度切入，似乎就沒必要再寫一篇文章；第二個原因是若是想要「完全」搞懂 this，要付出的成本可能比你想像中要大得多。

這裡所說的「完全」指的是無論在任何情況下，你都有辦法講出為什麼 this 的值是這樣，直接給大家一個範例：

``` js
var value = 1;
  
var foo = {
  value: 2,
  bar: function () {
    return this.value;
  }
}
  
//範例1
console.log(foo.bar());
//範例2
console.log((foo.bar)());
//範例3
console.log((foo.bar = foo.bar)());
//範例4
console.log((false || foo.bar)());
//範例5
console.log((foo.bar, foo.bar)());
```

你能答的出來嗎？如果不行的話，代表你沒有「完全」懂 this。要完全懂 this 之所以要付出的成本很大，就是因為「完全懂 this」指的就是「熟記 ECAMScript 規範」。this 的值是什麼不是我們憑空想像的，其實背後都有完整的定義，而那個定義就是所謂的 ECMAScript 規範，你必須先搞懂這個規範，才有可能完全理解 this 在每個情況下所指涉的對象。

若是你真的很想完全搞懂，推薦你這一篇：[JavaScript深入之从ECMAScript规范解读this](https://github.com/mqyqingfeng/Blog/issues/7)，我上面的範例就是從這一篇拿來的，想看解答、想理解為什麼的話可以去看這一篇。

既然我前面提了這麼多讓我不寫 this 的理由，怎麼最後我還是跳下來寫了？

因為，在我看了這麼多的文章，吸取了一大堆日月精華並且思考過後，發現如果有個不錯的切入點，或許就可以讓 this 變得不是這麼難懂。以我這篇教的方法，不會讓你把 this 完全搞懂，上面那五個範例你可能會答錯，但基本的題目你依舊可以解的出來。

這也是標題的由來：「絕對不完整，但保證好懂」，此文的目的是希望提供一個不同的角度來看 this，從為什麼會有 this 下手，再用一套規則來解釋 this 的值，至少讓你不再對 this 有誤解，也會知道一些常見的情境底下 this 到底是什麼。

# 要談 this，要從物件導向開始談

（若是你對 JavaScript 的物件導向完全沒概念，你可以先補完相關基礎，並且看完這篇：[該來理解 JavaScript 的原型鍊了](https://github.com/aszx87410/blog/issues/18)）

如果你有寫過其他程式語言，你就知道 this 從來都不是一件什麼困難的事。它代表的就是在物件導向裡面，那個 instance 本身。

我舉個例子：

``` js
class Car {
  setName(name) {
    this.name = name
  }
  
  getName() {
    return this.name
  }
}
  
const myCar = new Car()
myCar.setName('hello')
console.log(myCar.getName()) // hello
```

在上面我們宣告了一個 class `Car`，寫了 setName 跟 getName 兩個方法，在裡面用`this.name`來取放這個 instance 的屬性。

為什麼要這樣寫？因為這是唯一的方法，不然你要把 name 這個屬性存在哪裡？沒有其他地方讓你存了。所以 this 的作用在這裡是顯而易見的，所指到的對象就是那個 instance 本身。

以上面的範例來說，`myCar.setName('hello')`，所以 this 就會是`myCar`。在物件導向的世界裡面，this 的作用就是這麼單純。

或者換句話說，我認為：

> 一但脫離了物件導向，其實 this 就沒有什麼太大的意義

假設今天 this 只能在 class 裡面使用，那應該就不會有任何問題對吧？你有看過其他程式語言像是寫 Java 或是 C++ 的人在抱怨說 this 很難懂嗎？沒有，因為 this 的作用很單純。

那問題是什麼？問題就是在 JavaScript 裡面，你在任何地方都可以存取到 this。所以在 JavaScript 裡的 this 跟其他程式語言慣用的那個 this 有了差異，這就是為什麼 this 難懂的原因。

儘管 this 的定義不太一樣，但我認為本質上還是很類似的。要理解 this 的第一步就是告訴自己：「一但脫離了物件，就不太需要關注 this 的值，因為沒什麼意義」

# 沒什麼太大意義的 this

``` js
function hello(){
  console.log(this)
}
  
hello()
```

this 的值會是什麼？

延續我們前面所講的，在這種情況下我會跟你說 this 沒有任何意義，而且你千萬不要想成 this 會指到`hello`這個 function，沒有這種事。

只要記得我前面跟你說的：「脫離了物件，this 的值就沒什麼意義」。

在這種很沒意義的情況下，this 的值在瀏覽器底下就會是`window`，在 node.js 底下會是`global`，如果是在嚴格模式，this 的值就會是`undefined`。

這個規則應該滿好記的，幫大家重新整理一下：

1. 嚴格模式底下就都是`undefined`
2. 非嚴格模式，瀏覽器底下是`window`
3. 非嚴格模式，node.js 底下是`global`

這個就是你在其他文章看到的「預設綁定」，但我在這篇不打算用任何專有名詞去談 this。我認為不用這些名詞也不會妨礙你的理解，甚至還有可能讓你更好理解。我也不是說專有名詞不重要，是說可以先把概念學起來，再回過頭來補專有名詞。

一但脫離了物件，this 的值就沒什麼意義，在沒意義的情況底下就會有個預設值，而預設值也很好記，嚴格模式就是`undefined`，非嚴格模式底下就是全域物件。

# 更改 this 的值

僅管 this 可能有預設的值，但我們可以透過一些方法來改它。這改的方法也很簡單，一共有三種。

前兩種超級類似，叫做`call`跟`apply`，這兩種都是能夠呼叫 fucntion 的函式，我舉一個例子給你看比較好懂：

``` js
'use strict';
function hello(a, b){
  console.log(this, a, b)
}
  
hello(1, 2) // undefined 1 2
hello.call(undefined, 1, 2) // undefined 1 2
hello.apply(undefined, [1, 2]) // undefined 1 2
```

我們有一個叫做 hello 的函式，會 log 出 this 的值以及兩個參數。在我們呼叫`hello(1, 2)`的時候，因為是嚴格模式所以 this 是 undefined，而 a 跟 b 就是 1 跟 2。

當我們呼叫`hello.call(undefined, 1, 2)`的時候，我們先忽略第一個參數不談，你可以發現他其實跟`hello(1, 2)`是一樣的。

而 apply 的差別只在於他要傳進去的參數是一個 array，所以上面這三種呼叫 function 的方式是等價的，一模一樣。除了直接呼叫 function 以外，你也可以用 call 或是 apply 去呼叫，差別在於傳參數的方式不同。

call 跟 apply 的差別就是這麼簡單，一個跟平常呼叫 function 一樣，一個用 array 包起來。

那我們剛剛忽略的第一個參數到底是什麼呢？

你可能已經猜到了，就是`this`的值！

``` js
'use strict';
function hello(a, b){
  console.log(this, a, b)
}
  
hello.call('yo', 1, 2) // yo 1 2
hello.apply('hihihi', [1, 2]) // hihihi 1 2
```

就是如此簡單，你第一個參數傳什麼，裡面 this 的值就會是什麼。儘管原本已經有 this，也依然會被這種方法給覆蓋掉：

``` js
class Car {
  hello() {
    console.log(this)
  }
}
  
const myCar = new Car()
myCar.hello() // myCar instance
myCar.hello.call('yaaaa') // yaaaa
```

原本 this 的值應該要是 myCar 這個 instance，可是卻被我們在使用 call 時傳進去的參數給覆蓋掉了。

除了以上兩種以外，還有最後一種可以改變 this 的方法：bind。

``` js
'use strict';
function hello() {
  console.log(this)
}
  
const myHello = hello.bind('my')
myHello() // my
```

bind 會回傳一個新的 function，在這邊我們把 hello 這個 function 用 my 來綁定，所以最後呼叫 myHello() 時會輸出 my。

以上就是三種可以改變 this 的值的方法。你可能會好奇如果我們把 call 跟 bind 同時用會怎樣：

``` js
'use strict';
function hello() {
  console.log(this)
}
  
const myHello = hello.bind('my')
myHello.call('call') // my
```

答案是不會改變，一但 bind 了以後值就不會改變了。

這邊還要特別提醒的一點是在非嚴格模式底下，無論是用 call、apply 還是 bind，你傳進去的如果是 primitive 都會被轉成 object，舉例來說：

``` js
function hello() {
  console.log(this)
}
  
hello.call(123) // [Number: 123]
const myHello = hello.bind('my')
myHello() // [String: 'my']
```

幫大家做個中場總結：

1. 在物件以外的 this 基本上沒有任何意義，硬要輸出的話會給個預設值
2. 可以用 call、apply 與 bind 改變 this 的值

# 物件中的 this

最前面我們示範了在物件導向 class 裡面的 this，但在 JavaScript 裡面還有另外一種方式也是物件：

``` js
const obj = {
  value: 1,
  hello: function() {
    console.log(this.value)
  }
}
  
obj.hello() // 1
```

這種跟一開始的物件導向範例不太一樣，這個範例是直接創造了一個物件而沒有透過 class，所以你也不會看到 new 這個關鍵字的存在。

再繼續往下講之前，要大家先記住一件事情：

> this 的值跟作用域跟程式碼的位置在哪裡完全無關，只跟「你如何呼叫」有關

這個機制恰巧跟作用域相反，不確定我在說什麼的可以先看這篇：[所有的函式都是閉包：談 JS 中的作用域與 Closure](https://github.com/aszx87410/blog/issues/35)。

舉個簡單的例子來幫大家複習一下作用域：

``` js
var a = 10
function test(){
  console.log(a)
}
  
const obj = {
  a: 'ojb',
  hello: function() {
    test() // 10
  },
  hello2: function() {
    var a = 200
    test() // 10
  }
}
  
test() // 10
obj.hello()
obj.hello2()
```

無論我在哪裡，無論我怎麼呼叫`test`這個 function，他印出來的 a 永遠都會是全域變數的那個 a，因為作用域就是這樣運作，test 在自己的作用域裡面找不到 a 於是往上一層找，而上一層就是 global scope，這跟你在哪裡呼叫 test 一點關係都沒有。test 這個 function 在「定義」的時候就把 scope 給決定好了。

但 this 卻是完全相反，this 的值會根據你怎麼呼叫它而變得不一樣，還記得我們剛講過的 call、apply 跟 bind 嗎？這就是其中一個範例，你可以用不同的方式去呼叫 function，讓 this 的值變得不同。

所以你要很清楚知道這是兩種完全不同的運行模式，一個是靜態（作用域）、一個是動態（this）。要看作用域，就看這個函式在程式碼的「哪裡」；要看 this，就看這個函式「怎麽」被呼叫。

舉一個最常見的範例：

``` js
const obj = {
  value: 1,
  hello: function() {
    console.log(this.value)
  }
}
  
obj.hello() // 1
const hey = obj.hello
hey() // undefined
```

明明就是同一個函式，怎麼第一次呼叫時 this.value 是 1，第二次呼叫時就變成 undefined 了？

記住我剛說的話：「要看 this，就看這個函式『怎麽』被呼叫」。

再繼續往下講之前，先教大家一個最重要的小撇步，是我從[this 的值到底是什么？一次说清楚](https://zhuanlan.zhihu.com/p/23804247)學來的，是一個很方便的方法。

其實我們可以把所有的 function call，都轉成利用`call`的形式來看，以上面那個例子來說，會是這樣：

``` js
const obj = {
  value: 1,
  hello: function() {
    console.log(this.value)
  }
}
  
obj.hello() // 1
obj.hello.call(obj) // 轉成 call
const hey = obj.hello
hey() // undefined
hey.call() // 轉成 call
```

而規則就是你在呼叫 function 以前是什麼東西，你就把它放到後面去。所以`obj.hello()`就變成了`obj.hello.call(obj)`，`hey()`前面沒有東西，所以就變成了`hey.call()`。

轉成這樣子的形式之後，還記得 call 的第一個參數就是 this 嗎？所以你就能立刻知道 this 的值是什麼了！

舉一個更複雜的例子：

``` js
const obj = {
  value: 1,
  hello: function() {
    console.log(this.value)
  },
  inner: {
    value: 2,
    hello: function() {
      console.log(this.value)
    }
  }
}
  
const obj2 = obj.inner
const hello = obj.inner.hello
obj.inner.hello()
obj2.hello()
hello()
```

你可以不要往下拉，先想一下那三個 function 會各自印出什麼值。

接著我要公布解答了，只要轉成我們上面講的那種形式就好：

``` js
obj.inner.hello() // obj.inner.hello.call(obj.inner) => 2
obj2.hello() // obj2.hello.call(obj2) => 2
hello() // hello.call() => undefined
```

特別講一下最後一個 hello 因為沒有傳東西進去，所以是預設綁定，在非嚴格模式底下是 window，所以會 log 出`window.value`也就是 undefined。

只要你把 function 的呼叫轉成用 call 的這種形式，就很容易看出來 this 的值是什麼。

這也是我前面一直在提的：「要看 this，就看這個函式『怎麽』被呼叫」，而你要看怎麼被呼叫的話，就轉成 call 的形式就行了。

學到這邊，其實你看見九成與 this 相關的題目你都會解了，不信的話我們來試試看（為了可讀性沒有防雷空行，所以請自行拉到程式碼就好，再往下拉就會是解答了）：

``` js
function hello() {
  console.log(this)
}
  
var a = { value: 1, hello }
var b = { value: 2, hello }
hello()
a.hello()
b.hello.apply(a)
```

只要按照我們之前說的，用 call 來轉換一下形式就好：

``` js
hello() // hello.call() => window（瀏覽器非嚴格模式）
a.hello() // a.hello.call(a) => a
b.hello.apply(a) => 直接用 apply，所以就是 a
```

再來一題比較不一樣的，要看仔細囉（假設在瀏覽器底下跑，非嚴格模式）：

``` js
var x = 10
var obj = {
  x: 20,
  fn: function() {
    var test = function() {
      console.log(this.x)
    }
    test()
  }
}
  
obj.fn()
```

這題的話如果你搞錯，一定是你忘記了我們最重要的一句話：

> 要看 this，就看這個函式「怎麽」被呼叫

我們怎麼呼叫 test 的？`test()`，所以就是`test.call()`就是預設綁定，`this`的值就會是 window，所以`this.x`會是 10，因為在第一行宣告了一個全域變數 x = 10。

寫到這裡，再來幫大家做個回顧，避免大家忘記前面在講什麼：

1. 脫離物件的 this 基本上沒有任何意義
2. 沒有意義的 this 會根據嚴格模式以及環境給一個預設值
3. 嚴格模式底下預設就是 undefined，非嚴格模式在瀏覽器底下預設值是 window
4. 可以用 call、apply 與 bind 改變 this 的值
5. 要看 this，就看這個函式「怎麽」被呼叫
6. 可以把 `a.b.c.hello()` 看成 `a.b.c.hello.call(a.b.c)`，以此類推，就能輕鬆找出 this 的值

# 不合群的箭頭函式

原本有關 this 的部分應該講到上面就要結束了，但 ES6 新增的箭頭函式卻有不太一樣的運作方式。它本身並沒有 this，所以「在宣告它的地方的 this 是什麼，它的 this 就是什麼」，好，我知道這聽起來超難懂，我們來看個範例：

``` js
const obj = {
  x: 1,
  hello: function(){
    // 這邊印出來的 this 是什麼，test 的 this 就是什麼
    // 就是我說的：
    // 在宣告它的地方的 this 是什麼，test 的 this 就是什麼
    console.log(this)     
    const test = () => {
      console.log(this.x)
    }
    test()
  }
}
  
obj.hello() // 1
const hello = obj.hello
hello() // undefined
```

在第五行我們在 hello 這個 function 裡面宣告了 test 這個箭頭函式，所以 hello 的 this 是什麼，test 的 this 就是什麼。

所以當我們呼叫`obj.hello()`時，test 的 this 就會是 obj；`hello()`的時候 test 的 this 就會是全域物件。這規則其實都跟之前一樣，差別只有在於說箭頭函式的 this 不是自己決定的，而是取決於在宣告時那個地方的 this。

如果你想看更複雜的範例，可以參考這篇：[鐵人賽：箭頭函式 (Arrow functions)
](https://wcc723.github.io/javascript/2017/12/21/javascript-es6-arrow-function/)。

# 實際應用：React

你有寫過 React 的話，就會知道裡面其實有些概念今天的教學可以派上用場，舉例來說，我們必須在 constructor 裡面先把一些 method 給 bind 好，你有想過是為什麼嗎？

先來看看如果沒有 bind 的話會發生什麼事：

``` js
class App extends React.Component {
  onClick() {
    console.log(this, 'click')
  }
  render() {
    return <button onClick={this.onClick}>click</button>
  }
}
```

最後 log 出來的值會是 undefined，為什麼？這細節就要看 React 的原始碼了，只有 React 知道實際上在 call 我們傳下去的 onClick 函式時是怎麼呼叫的。

所以為什麼要 bind？為了確保我們在`onClick`裡面拿到的 this 永遠都是這個 instance 本身。

``` js
class App extends React.Component {
  constructor() {
    super()
   
    // 所以當你把 this.onClick 傳下去時，就已經綁定好了 this
    // 而這邊的 this 就是這個 component
    this.onClick = this.onClick.bind(this)
  }
  onClick() {
    console.log(this, 'click')
  }
  render() {
    return <button onClick={this.onClick}>click</button>
  }
}
```

還有另外一種方式是用箭頭函式：

``` js
class App extends React.Component {
  render() {
    return <button onClick={() => {
    	console.log(this)
    }}>click</button>
  }
}
```

為什麼箭頭函式也可以？因為我們前面提過，「在宣告它的地方的 this 是什麼，它的 this 就是什麼」，所以這邊 log 出來的 this 就會是 render 這個 function 的 this，而 render 的 this 就是這個 component。

如果你有點忘記了，可以把文章拉到最上面去，因為最上面我們就已經提過這些了。

# 總結

關於講解 this 的文章，我至少看過十幾二十篇，比較常見的就是講幾種不同的綁定方法，以及在哪些時候會用哪一種綁定。但這些我在這篇文章裡都沒有提，因為我認為不影響理解（但最好之後能夠自己去補足相關名詞）。

我也曾經迷惘過，曾經被 this 搞得很混亂，前陣子因為教學的緣故不得不把 this 搞懂，而我也確實比以前理解很多了。從我的經驗看來，我認為 this 之所以複雜，原因之一就是：「在物件以外的地方也可以用 this」，所以我才一再強調我認為物件外的 this 是沒意義的。

我對 this 真正開竅的時候是看到了[this 的值到底是什么？一次说清楚](https://zhuanlan.zhihu.com/p/23804247)這篇文章，簡直就是醍醐灌頂，把一般的 function 呼叫換成用 call 的這種形式是個很容易理解也很容易記的方法，而且可以應用在九成的場景底下。

最後，再次強調這篇文章是有疏漏的，開頭的那幾個範例以這篇文章所學到的知識依然無法解釋，那真的是要看 ECMAScript 才會知道；而瀏覽器的 event 的 this 我也沒提，但這部分比較簡單就是了。我只求這篇文章能讓你知道八成的狀況底下 this 的值會是什麼，其他兩成請另尋高明。

這篇融合了我看過的幾篇文章帶給我的想法，也融合了自己融會貫通後的體悟，希望我也能帶給那些卡在 this 許久的初學者們一些新的想法，在看完這篇文章之後能夠以不同的角度去思考 this 的值以及它存在的意義。

寫完這篇以後，關於 JavaScript 那些非常常見但我以前完全沒弄懂的問題就都解釋的差不多了，有興趣的朋友們可以參考其他的主題：

1. [該來理解 JavaScript 的原型鍊了](https://github.com/aszx87410/blog/issues/18)
2. [深入探討 JavaScript 中的參數傳遞：call by value 還是 reference？](https://github.com/aszx87410/blog/issues/30)
3. [我知道你懂 hoisting，可是你了解到多深？](https://github.com/aszx87410/blog/issues/34)
4. [所有的函式都是閉包：談 JS 中的作用域與 Closure](https://github.com/aszx87410/blog/issues/35)

參考資料：

1. [JavaScript深入之从ECMAScript规范解读this](https://github.com/mqyqingfeng/Blog/issues/7)
2. [this 的值到底是什么？一次说清楚](https://zhuanlan.zhihu.com/p/23804247)
3. [What's THIS in JavaScript ?](https://kuro.tw/posts/2017/10/12/What-is-THIS-in-JavaScript-%E4%B8%8A/)
4. [JS this](https://github.com/nightn/front-end-plan/blob/master/js/js-this.md)

關於作者： 
[@huli](https://github.com/aszx87410/blog) 野生工程師，相信分享與交流能讓世界變得更美好