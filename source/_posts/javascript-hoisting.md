---
title: 我知道你懂 hoisting，可是你了解到多深？
date: 2018-11-10 18:05:13
tags:
  - javascript
  - hoisting
author: huli
---

# 前言

這陣子我在忙一些教學相關的東西，稍微準備一些資料之後教了學生們 JavaScript 裡面的 hoisting，也就是「提升」這個觀念，例如說以下程式碼：

``` js
console.log(a)
var a = 10
```

會輸出`undefined`而不是`ReferenceError: a is not defined`，這種現象就叫做 [Hoisting](https://developer.mozilla.org/zh-TW/docs/Glossary/Hoisting)，變數的宣告被「提升」到最上面去了。

如果你只想了解最基本的 hoisting，其實差不多就是這樣，但後來我還教了`let`跟`const`相關的一些知識，不過前一天剛教學完，隔天就立刻看到相關的技術文章還發現自己教錯，因此特別花了一點時間打算好好理解 hoisting 這個東西。

很多東西沒有深入研究的時候你都會覺得沒什麼，真的跳下去深入去看才會發現自己其實還有一大堆概念沒有搞懂。

有很多人都知道 hoisting，但是理解程度卻不盡相同，我列出了 10 個項目，如果有任何一點你剛好不知道的話，那恭喜，這篇文章應該可以為你帶來一些收穫。

1. 你知道什麼是 hoisting
2. 你知道 hoisting 只會提升宣告而非賦值
3. 你知道 function 宣告、function 的參數以及一般變數宣告同時出現時的提升優先順序
4. 你知道 let 跟 const 沒有 hoisting
5. 你知道第四點是錯的，其實有但只是表現形式不一樣
6. 你知道有關第五點，有個概念叫做 TDZ（Temporal Dead Zone）
7. 你看過 ES3 的規格書，知道裡面是怎麼描述的
8. 你看過 ES6 的規格書，知道裡面是怎麼描述的
9. 你知道 hoisting 背後的原理是什麼
10. 你看過 V8 編譯出來的程式碼

你可能會問說：「我為什麼要知道的這麼深？有什麼用？」，其實我也覺得對 hoisting，只要知道基本的就行了。只要你有好好地宣告變數，就算不知道那些，對日常生活或是工作也不會有太大的影響。

可是假如你像我一樣，想要有朝一日在自己的履歷上面放上「精通 JavaScript」的話，那對這些東西就不能逃避。同時你如果對底層的這些細節愈熟悉，會碰到的問題就愈少，也愈能理解為什麼會有 hoisting 的出現，當你想要在技術這條路上走得更遠爬得更高時，我覺得這些細節是很重要的。

接下來，我們就一步步來看 hoisting 吧！

# 什麼是 hoisting？

在 JavaScript 裡面，如果你試圖去對一個還沒宣告的變數取值，會發生以下錯誤：

``` js
console.log(a)
// ReferenceError: a is not defined
```

會回傳一個`a is not defined`的錯誤，因為你還沒宣告這個變數，所以 JavaScript 也找不到這變數在哪，自然就會拋出錯誤。

可是如果你這樣子寫，神奇的事情發生了：

``` js
console.log(a) // undefined
var a
```

從以前學程式的時候我們就學到了一個觀念，「程式是一行一行跑的」，那既然是一行一行跑的，執行到第一行的時候不是還沒有宣告變數 a 嗎？那為什麼不是拋出`a is not defined`的錯誤，而是輸出了`undefined`？

這種現象就叫做 hoisting，提升，在第二行的`var a`因為某種原因被「提升」到了最上面，所以上面的程式碼你可以「想像」成這樣：

``` js
var a
console.log(a) // undefined
```

我會特別強調「想像」，是因為程式碼的位置其實不會被移動，所以不要把提升想成是 JavaScript 引擎幫你把變數宣告都「移動」到最上面，這是有問題的。它背後的原理跟移動程式碼一點關係都沒有。

再來還有一點要特別注意，那就是只有變數的宣告會提升，賦值不會，看看以下範例你就懂了：

``` js
console.log(a) // undefined
var a = 5
```

上面的程式碼你可以「想像」成這樣：

``` js
var a
console.log(a) // undefined
a = 5
```

你可以把`var a = 5`這句話分成兩個步驟，第一個階段是宣告變數：`var a`，第二個階段是賦值：`a = 5`，只有前面的變數宣告會被提升，賦值不會。

到這邊你可能覺得還行，只是頭腦有一點點混亂，那恭喜你，等一下還有更多東西會讓你更亂，讓我們再把幾個東西加進來，看看能夠搞得多複雜。

如果我們像下面這樣做，會輸出什麼？

``` js
function test(v){
  console.log(v)
  var v = 3
}
test(10)
```

簡單嘛，根據剛剛學過的，把上面的程式碼變換成以下形式：

``` js
function test(v){
  var v
  console.log(v)
  v = 3
}
test(10)
```

答案是`undefined`！輕輕鬆鬆。

可是瑞凡，答案是`10`而不是`undefined`。

其實變換的過程對了，只是遺漏了一個因素：傳進來的參數。把這個因素加進去以後，可以看成是這樣：

``` js
function test(v){
  var v = 10 // 因為下面呼叫 test(10)
  var v
  console.log(v)
  v = 3
}
test(10)
```

這時候你可能還是會問：「可是我在 log 以前不是重新宣告了一次變數並且沒有給值嗎？那它不是會被覆蓋成`undefiend`嗎？」

我們再來看一個簡單的小例子：

``` js
var v = 5
var v
console.log(v)
```

答案會是`5`而不是`undefined`，想理解這個行為，你可以再回想一下前面把一個句子分成兩塊，宣告跟賦值，如果我們這樣切分再加上 hoisting 的話，其實上面程式碼可以想像成這樣：

``` js
var v
var v
v = 5
console.log(v)
```

這樣你就知道為什麼答案是 5 了。

此時的你應該覺得頭腦有點快爆炸了，為什麼這麼多規則要記？別擔心，我們還有最後一個例子，保證讓你叫苦連天。

``` js
console.log(a) //[Function: a]
var a
function a(){}
```

除了變數宣告以外，function 的宣告也會提升而且優先權比較高，因此上面的程式碼會輸出`function`而不是`undefined`。

好，基本的 hoisting 概念就到這邊結束了，幫你畫一下重點：

1. 變數宣告跟函式宣告都會提升
2. 只有宣告會提升，賦值不會提升
3. 別忘了函式裡面還有傳進來的參數

別急，還有 ES6 新增的 let 跟 const 沒有講。

# let 跟 const 與 hoisting

在 ES6 裡面我們有了新的兩個宣告變數的關鍵字，let 與 const，這兩個對 hoisting 的行為是差不多的，因此我下面只拿 let 來舉例，可以看一下以下程式碼：

``` js
console.log(a) // ReferenceError: a is not defined
let a
```

謝天謝地，終於沒有那麼多規則要記了！

從以上程式碼看起來，let 與 const 應該是沒有變數提升吧，否則就不會拋出這個 Error 了。

我之前也是很天真這樣想的，直到我看到以下範例：

``` js
var a = 10
function test(){
  console.log(a)
  let a
}
test()
```

如果 let 真的沒有 hoisting 的話，答案應該會輸出`10`，因為 log 那一行會存取到外面的`var a = 10`的這個變數，可是！！！

答案卻是：`ReferenceError: a is not defined`。

意思就是，它的確提升了，只是提升後的行為跟 var 比較不一樣，所以乍看之下你會以為它沒有提升。

這個觀念我們會在之後詳細講解，但在這之前我們先做個簡單的總結。

有很多提到 hoisting 的文章，講到這邊就差不多了，就大概講一些 hoisting 的行為以及 let 與 const 的不同之處，但我覺得只講到這裡其實是很可惜的一件事情。

因為如果你只理解到這個程度，就會以為 hoisting 不過就是一大堆複雜的規則要記，根本沒什麼，誰能夠記這麼多規則？不就是背書而已嗎？

這是因為上面那些只讓你理解了「表面」，舉出幾個不同的例子跟你說會有這樣的行為發生，可是卻沒有跟你說「為什麼會這樣」，或者是「實際上是怎麼運作的」，若是你想真正了解 hoisting 是什麼，必須要找出以下兩個問題的答案，一但找出了，保證你任督二脈直接被打通：

1. 為什麼我們需要 hoisting？
2. Hoisting 到底是怎麼運作的？

# 為什麼我們需要 hoisting？

在問這樣的一個問題時，其實你可以反過來想：「如果我們沒有 hoisting 會怎樣？」

第一，我們一定要先宣告變數才可以使用。

這點其實很不錯，畢竟這是一個好習慣。

第二，我們一定要先宣告函式才可以使用。

這一點就不太好了，很不方便，如果是這樣的話那可能在每個檔案你都必須把 function 宣告放到最上面去，才能保證你底下的程式碼都可以 call 到這些 function。

第三，沒有辦法達成 function 互相呼叫。

舉個例子：

``` js
function loop(n){
  if (n>1) {
    logEvenOrOdd(--n)
  }
}
  
function logEvenOrOdd(n) {
  console.log(n, n % 2 ? 'Odd' : 'Even')
  loop(n)
}
  
loop(10)
```

我們在`loop`裡面呼叫`logEvenOrOdd`，在`logEvenOrOdd`裡面也呼叫`loop`，如果我們沒有 hoisting，那以上的程式碼就不可能達成，因為你不可能同時做到 A 在 B 上面而 B 又在 A 上面。

所以為什麼我們需要 hoisting？就是為了要解決上面的問題。

為了增添這一個說法的正確性，我引用一篇文章給大家看，在 [Note 4. Two words about “hoisting”.](http://dmitrysoshnikov.com/notes/note-4-two-words-about-hoisting/) 裡面有提到作者發了個 Twitter 問說 hoisting 的目的到底是什麼，然後 JavaScript 的作者有出來回應，以下是文章裡面附的原文跟回應：

> By the way, recently I raised this topic on Twitter and also mentioned as one of the reasons the mutual recursion. Brendan Eich also gave an acknowledgment that FDs hoisting is “for mutual recursion & generally to avoid painful bottom-up ML-like order”.

如果想看完整對話截圖，可以看這篇文章：[JavaScript系列文章：变量提升和函数提升](https://www.cnblogs.com/liuhe688/p/5891273.html)，最下面有附。

# hoisting 到底是怎麼運作的？

現在我們知道了什麼是 hoisting，知道了為什麼需要 hoisting，接著就是欠缺的最後一塊拼圖了：到底 hoisting 是怎麼運作的？

要回答這個問題，最好的方法就是去找 ECMAScript 的規格書來看，就像你今天想研究型別轉換的問題，解法一樣是去找規格書來看，理由很簡單，因為那些規則都清清楚楚寫在上面了。

ECMAScript 有很多版本，越後面的版本規格越多，所以為了方便起見，我們底下用 ES3 當作範例。

如果你看過 ES3 的規則，會發現用 hoisting 當關鍵字完全找不到東西，而與 hoisting 這現象有關的段落其實在第十章：Execution Contexts。

這邊先非常簡單介紹一下什麼是 Execution Contexts（以下簡稱 EC），每當你進入一個 function 的時候，就會產生一個 EC，裡面儲存跟這個 function 有關的一些資訊，並且把這個 EC 放到 stack 裡面，當 function 執行完以後，就會把 EC 給 pop 出來。

示意圖大概就像這樣，要記得除了 function 有 EC 以外，還有一個 global EC：

![](/img/huli/hoisting/ec.png)
（來源：https://medium.freecodecamp.org/lets-learn-javascript-closures-66feb44f6a44）

簡而言之呢，所有 function 需要的資訊都會存在 EC，也就是執行環境裡面，你要什麼都去那邊拿就對了。

ECMAScript 是這樣描述的：

> When control is transferred to ECMAScript executable code, control is entering an execution context. Active execution contexts logically form a stack. The top execution context on this logical stack is the running execution context.

再來就是重點了，在`10.1.3 Variable Instantiation`的地方是這樣寫的：

> Every execution context has associated with it a variable object. Variables and functions declared in the source text are added as properties of the variable object. For function code, parameters are added as properties of the variable object.

每個 EC 都會有相對應的 variable object（以下簡稱 VO），在裡面宣告的變數跟函式都會被加進 VO 裡面，如果是 function，那參數也會被加到 VO 裡。

首先，你可以把 VO 想像成就是一個 JavaScript 的物件就好。

再來，VO 什麼時候會用到？你在存取值的時候會用到，例如說 `var a = 10` 這一句，之前有講過可以分成左右兩塊：

1. `var a`：去 VO 裡面新增一個屬性叫做 a（如果沒有 a 這個屬性的話）並初始化成 undefined
2. `a = 10`：先在 VO 裡面找到叫做 a 的屬性，找到之後設定為 10

（這邊如果 VO 裡面找不到怎麼辦？它會透過 scope chain 不斷往上尋找，如果每一層都找不到就會拋出錯誤。至於尋找跟建立 scope chain 的過程雖然與本文有關但可以講得太多了，額外再開一篇會比較適合，這邊就先不提了）

接著來看下一段：

> Which object is used as the variable object and what attributes are used for the properties depends on the type of code, but the remainder of the behaviour is generic. On entering an execution context, the properties are bound to the variable object in the following order:

最精華的只有這一句：「On entering an execution context, the properties are bound to the variable object in the following order」，在進入 EC 的時候，會按照以下順序把東西放到 VO 裡面：

下面的段落有點長，我節錄一下：

> For function code: for each formal parameter, as defined in the FormalParameterList, create a property of the variable object whose name is the Identifier and whose attributes are determined by the type of code. The values of the parameters are supplied by the caller as arguments to [[Call]].
> 
> If the caller supplies fewer parameter values than there are formal parameters, the extra formal parameters have value undefined

簡單來說就是對於參數，它會直接被放到 VO 裡面去，如果有些參數沒有值的話，那它的值會被初始化成 undefined。

舉例來說，假設我 function 長這樣：

``` js
function test(a, b, c) {}
test(10)
```

那我的 VO 就長這樣：

``` js
{
  a: 10,
  b: undefined,
  c: undefined
}
```

所以參數是第一個優先順序，再來我們看第二個：

> For each FunctionDeclaration in the code, in source text order, create a property of the variable object whose name is the Identifier in the FunctionDeclaration, whose value is the result returned by creating a Function object as described in 13, and whose attributes are determined by the type of code. 
> 
> If the variable object already has a property with this name, replace its value and attributes. Semantically, this step must follow the creation of FormalParameterList properties.

對於 function 的宣告，一樣在 VO 裡面新增一個屬性，至於值的話就是建立 function 完之後回傳的東西（可以想成就是一個指向 function 的指標就好）。

再來是重點：「如果 VO 裡面已經有同名的屬性，就把它覆蓋掉」，舉個小例子：

``` js
function test(a){
  function a(){}
}
test(1)
```

VO 會長的像這樣，原本的參數`a`被覆蓋掉了：

``` js
{
  a: function a
}
```

最後來看對於變數的宣告該怎麼處理：

> For each VariableDeclaration or VariableDeclarationNoIn in the code, create a property of the variable object whose name is the Identifier in the VariableDeclaration or VariableDeclarationNoIn, whose value is undefined and whose attributes are determined by the type of code. If there is already a property of the variable object with the name of a declared variable, the value of the property and its attributes are not changed. 
> 
> Semantically, this step must follow the creation of the FormalParameterList and FunctionDeclaration properties. In particular, if a declared variable has the same name as a declared function or formal parameter, the variable declaration does not disturb the existing property.

對於變數，在 VO 裡面新增一個屬性並且把值設為 undefined，再來是重點：「如果 VO 已經有這個屬性的話，值不會被改變」

來重新整理一下，當我們在進入一個 EC 的時候（你可以把它想成就是在執行 function 後，但還沒開始跑 function 內部的程式碼以前），會按照順序做以下三件事：

1. 把參數放到 VO 裡面並設定好值，傳什麼進來就是什麼，沒有值的設成 undefined
2. 把 function 宣告放到 VO 裡，如果已經有同名的就覆蓋掉
3. 把變數宣告放到 VO 裡，如果已經有同名的則忽略

在你看完規格並且稍微理解以後，你就可以用這個理論來解釋我們前面看過的程式碼了：

``` js
function test(v){
  console.log(v)
  var v = 3
}
test(10)
```

每個 function 你都可以想成其實執行有兩個階段，第一個階段是進入 EC，第二個階段才是真的一行行執行程式。

在進入 EC 的時候開始建立 VO，因為有傳參數進去，所以先把 v 放到 VO 並且值設定為 10，再來對於裡面的變數宣告，VO 裡面已經有 v 這個屬性了，所以忽略不管，因此 VO 就長這樣子：

``` js
{
  v: 10
}
```

進入 EC 接著建立完 VO 以後，才開始一行行執行，這也是為什麼你在第二行時會印出 10 的緣故，因為在那個時間點 VO 裡面的 v 的確就是 10 沒錯。

如果你把程式碼換成這樣：

``` js
function test(v){
  console.log(v)
  var v = 3
  console.log(v)
}
test(10)
```

那第二個印出的 log 就會是 3，因為執行完第三行以後， VO 裡面的值被換成 3 了。

以上就是 ES3 的規格書裡面提到的執行流程，你只要記得這個執行流程，碰到任何關於 hoisting 的題目都不用怕，你按照規格書的方法去跑絕對沒錯。

當我知道了這一段執行流程以後，第一個感想是豁然開朗，覺得 hoisting 不再是什麼神秘的東西，你只要假裝自己是 JS 引擎，跟著跑就好。第二個感想是，JS 到底怎麼做到的？

# 編譯與直譯：JS 引擎到底怎麼運作的？

還記得我上面的時候有提過，以前學程式的時候一直有個概念，那就是「直譯」代表程式是一行行跑的，而 JS 作為一個直譯的語言，不是也該一行行跑嗎？

可是如果真的一行行跑，那怎麼可能達成 hoisting 這個功能？你在執行第 n 行的時候根本不知道 n + 1 行是什麼，想提升是不可能的。

針對這個疑惑我上網找了很久的資料，最後找到一篇的說法我覺得滿合理的：[虚拟机随谈（一）：解释器，树遍历解释器，基于栈与基于寄存器，大杂烩](http://rednaxelafx.iteye.com/blog/492667)。

裡面提到了幾點我覺得寫得非常不錯，有破除了我滿多以前的迷思：

第一，語言一般只會定義抽象語義，不會強制用某種方式實現，像是 C 我們會說它是編譯型語言，可是 C 也有直譯器。所以當我們在說某種程式語言是直譯或編譯型的時候，其實是在指涉「大多數」而不是全部。

換言之，我們說 JavaScript 是直譯型語言，不代表 JavaScript 不能有編譯器，反之亦然。

第二，直譯器跟編譯器最大的差別在於「執行」。

編譯這個步驟就是把原始碼 A 編譯為目的碼 B，就這樣而已，但你要保證 A 跟 B 執行完的結果要相同。

而直譯就是你輸入原始碼 A，輸出就直接是你程式碼裡面要執行的語義，裡面怎麼做的是一個黑箱子。

原文裡面有一張圖畫得很不錯：

![](/img/huli/hoisting/compile.png)

所以直譯器裡面也能有編譯，這是不衝突的，或是你也可以寫一個超簡單直譯器，就是你輸入原始碼以後幫你編譯完然後執行。

事實上很多種直譯器內部的運作方式都是先把原始碼編譯成某種中間碼再去執行，所以編譯這個步驟還是很常見的，而 JS 也是這樣運作的。

當你拋開以前那種「JS 就是要一行行執行」的舊觀念並擁抱「其實主流 JS 引擎都有編譯這個步驟」的想法後，你就不會覺得 hoisting 是無法達成的事情了。

前面我們已經有看規格，知道在 ES3 裡面的運行模式並且知道 VO 這個東西，但規格裡面描述的也只是抽象的東西，它並沒有寫說「實際上」是在哪個地方做處理的，而這地方其實就是編譯階段。

話說關於這個編譯直譯的問題其實我卡滿久的，因為以前觀念不正確的地方很多，現在慢慢把它修正過來，而對於 hoisting 其實我之前有點分不清楚規格跟實作的差別，後來還跑去問了You-Dont-Know-JS 的作者，也很幸運地得到回覆，有興趣的人可以看看：[https://github.com/getify/You-Dont-Know-JS/issues/1375](https://github.com/getify/You-Dont-Know-JS/issues/1375)。

# JS 引擎的運作

如同我上面所說的，其實現在主流 JS 引擎內部都會有編譯這個階段，而 hoisting 其實就是在編譯這個階段做處理的。引入了編譯階段以後，可以把 JS 分成編譯階段跟執行階段兩個步驟。

在編譯階段的時候，會處理好所有的變數及函式宣告並且加入到 scope 裡面，在執行的時候就一樣可以去使用它。詳細情形這一篇寫得很好：[Hoisting in JavaScript
](https://john-dugan.com/hoisting-in-javascript/)，我下面就直接改一下裡面的程式碼當做例子。

舉例來說，我有這樣一段程式碼：

``` js
var foo = "bar"
var a = 1
function bar() {
    foo = "inside bar"
    var a = 2
    c = 3
    console.log(c)
    console.log(d)
}
bar()
```

在編譯階段的時候會處理宣告的部分，所以會是這樣：

```
Line 1：global scope，我要宣告一個變數叫做 foo
Line 2：global scope，我要宣告一個變數叫做 a
Line 3：global scope，我要宣告一個函式叫做 bar
Line 4：沒有任何變數宣告，不做事
Line 5：bar scope，我要宣告一個變數叫做 a
Line 6：沒有任何變數宣告，不做事
Line 7：沒有任何變數宣告，不做事
Line 8：沒有任何變數宣告，不做事
```

處理完後的東西差不多就長這樣：

``` js
globalScope: {
  foo: undefined,
  a: undefined,
  bar: function
}
  
barScope: {
  a: undefined
}
```

再來進入到執行階段，這邊有兩個專有名詞先記一下，在介紹之前我先給一個範例會比較好理解：

``` js
var a = 10
console.log(a)
```

上面這兩行有個差異，第一行的時候我們只需要知道「a 的記憶體位置在哪裡」就好，我們不關心它的值是什麼。

而第二行則是「我們只關心它的值是什麼，把值給我就好」，所以儘管兩行裡面都有`a`，但你可以看出來他們所要做的事情是不一樣的。

而第一行的 a 我們叫它 LHS（Left hand side）引用，第二行叫它 RHS（Right hand side）引用，這邊的 left 跟 right 指的是相對於等號的左右邊，但用這種方式理解的話其實不夠精確，因此像下面這樣記就好：

LHS：請幫我去查這個變數的位置在哪裡，因為我要對它賦值。  
RHS：請幫我查詢這個變數的值是什麼，因為我要用這個值。

有了這個概念以後，再看一次上面的範例程式碼，就可以一步一步來解釋：

``` js
var foo = "bar"
var a = 1
function bar() {
    foo = "inside bar"
    var a = 2
    c = 3
    console.log(c)
    console.log(d)
}
bar()
```

### Line 1：var foo = "bar"

JS 引擎：global scope，我這裡有個對 foo 的 LHS 引用，你有看過它嗎？  
執行結果：scope 說有，所以成功找到 foo 並且賦值  

這時候的 global scope：

``` js
{
  foo: "bar",
  a: undefined,
  bar: function
}
```

### Line 2：var a = 1
JS 引擎：global scope，我這裡有個對 a 的 LHS 引用，你有看過它嗎？
執行結果：scope 說有，所以成功找到 a 並且賦值

這時候的 global scope：

``` js
{
  foo: "bar",
  a: 1,
  bar: function
}
```

### Line 10：bar()

JS 引擎：global scope，我這裡有個對 bar 的 RHS 引用，你有看過它嗎？  
執行結果：scope 說有，所以成功返回 bar 的值並且呼叫 function

### Line 4：foo = "inside bar"

JS 引擎：bar scope，我這裡有個對 foo 的 LHS 引用，你有看過它嗎？  
執行結果：bar scope 說沒有，所以去問上一層的 global scope  
JS 引擎：global scope，我這裡有個對 foo 的 LHS 引用，你有看過它嗎？  
執行結果：有，所以成功找到 foo 並且賦值

這時候的 global scope：

``` js
{
  foo: "inside bar",
  a: 1,
  bar: function
}
```

### Line 5：var a = 2

JS 引擎：bar scope，我這裡有個對 a 的 LHS 引用，你有看過它嗎？  
執行結果：bar scope 說有，所以成功找到 a 並且賦值

此時的 bar scope：

``` js
{
  a: 2
}
```

### Line 6：c = 3

JS 引擎：bar scope，我這裡有個對 c 的 LHS 引用，你有看過它嗎？  
執行結果：bar scope 說沒有，所以去問上一層的 global scope  
JS 引擎：global scope，我這裡有個對 c 的 LHS 引用，你有看過它嗎？  
執行結果：沒有。

這時候有幾種結果，如果你是處在嚴格模式底下（use strict），會返回 `ReferenceError: c is not defined` 錯誤，如果你不是在嚴格模式，那 global scope 就會把 c 加上去並且設定成 3，這邊先假設我們不是在嚴格模式。

此時的 global scope：

``` js
{
  foo: "inside bar",
  a: 1,
  bar: function,
  c: 3
}
```

### Line 7：console.log(c)

JS 引擎：bar scope，我這裡有個對 c 的 RHS 引用，你有看過它嗎？  
執行結果：bar scope 說沒有，所以去問上一層的 global scope  
JS 引擎：global scope，我這裡有個對 c 的 RHS 引用，你有看過它嗎？  
執行結果：global scope 說有，所以成功返回 c 的值並且呼叫 console.log

### Line 8：console.log(d)

JS 引擎：bar scope，我這裡有個對 d 的 RHS 引用，你有看過它嗎？  
執行結果：bar scope 說沒有，所以去問上一層的 global scope  
JS 引擎：global scope，我這裡有個對 d 的 RHS 引用，你有看過它嗎？  
執行結果：global scope 說沒有，所以返回錯誤 `ReferenceError: d is not defined`

以上就是 JS 引擎的運作流程，想更詳細了解的話可參考：[You Don't Know JS: Scope & Closures](https://github.com/getify/You-Dont-Know-JS/blob/master/scope%20%26%20closures/ch1.md#enginescope-conversation)、[Chapter 4: Hoisting](https://github.com/getify/You-Dont-Know-JS/blob/master/scope%20%26%20closures/ch4.md#the-compiler-strikes-again)、[Hoisting in JavaScript](https://john-dugan.com/hoisting-in-javascript/)。

# 中場總結

再次回顧一下我們開場放的那十個項目：

1. 你知道什麼是 hoisting
2. 你知道 hoisting 只會提升宣告而非賦值
3. 你知道 function 宣告、function 的參數以及一般變數宣告同時出現時的提升優先順序
4. 你知道 let 跟 const 沒有 hoisting
5. 你知道第五點是錯的，其實有但只是表現形式不一樣
6. 你知道有關第六點，有個概念叫做 TDZ（Temporal Dead Zone）
7. 你看過 ES3 的規格書，知道裡面是怎麼描述的
8. 你看過 ES6 的規格書，知道裡面是怎麼描述的
9. 你知道 hoisting 背後的原理是什麼
10. 你看過 V8 編譯出來的程式碼

我們用了許多篇幅把其中的七點都講完了，剩下的是：

1. 你知道有關第六點，有個概念叫做 TDZ（Temporal Dead Zone）
2. 你看過 ES6 的規格書，知道裡面是怎麼描述的
3. 你看過 V8 編譯出來的程式碼

關於 ES6 的規格那點我不打算詳細講（而且我也還沒詳細看完），因為變化還滿多的但基本上原理不變，就是多了一些專有名詞而已，想知道的可以參考這篇經典好文：[ECMA-262-5 in detail. Chapter 3.2. Lexical environments: ECMAScript implementation.](http://dmitrysoshnikov.com/ecmascript/es5-chapter-3-2-lexical-environments-ecmascript-implementation/)。

上面我們已經講了很多東西，所有跟 hoisting 有關的運作機制全部都講完了，但我相信依然需要一點時間吸收，但我相信吸收完以後你會覺得神清氣爽，想說 hoisting 不過如此而已。

接著呢，我們就要進入到這篇文章最後的部分了，也就是 TDZ 以及 V8。

# Temporal Dead Zone

還記得我們說過 let 與 const 其實有 hoisting 嗎？並且舉了一個小範例來驗證這件事情。

let 與 const 確實有 hoisting，與 var 的差別在於提升之後，var 宣告的變數會被初始化為 undefined，而 let 與 const 的宣告不會被初始化為 undefined，而且如果你在「賦值之前」就存取它，就會拋出錯誤。

在「提升之後」以及「賦值之前」這段「期間」，如果你存取它就會拋出錯誤，而這段期間就稱做是 TDZ，它是一個為了解釋 let 與 const 的 hoisting 行為所提出的一個名詞。

我們用下面的程式碼當做例子：

``` js
function test() {
    var a = 1; // c 的 TDZ 開始
    var b = 2;
    console.log(c) // 錯誤
    if (a > 1) {
      console.log(a)
    }
    let c = 10 // c 的 TDZ 結束
}
test()
```

當你在第八行執行以前試圖存取 c 的話，就會拋出錯誤。要注意的是 TDZ 並不是一個空間上的概念，而是時間，例如說以下程式碼：

``` js
function test() {
    yo() // c 的 TDZ 開始
    let c = 10 // c 的 TDZ 結束
    function yo(){
      console.log(c)
    }
}
test()
```

在你進入 test 這個 function 的時候，就已經是 c 的 TDZ 了，所以當你執行 yo 並且執行到`console.log(c)`時，都還在 TDZ 裡面，要一直等到`let c = 10`被執行 TDZ 才會結束。

所以並不是說我把`console.log(c)`放在`let c = 10`下面就沒問題了，而是在「執行順序」上要在後面。

或是你也可以拋開這些名詞，用一句話總結：

> let 與 const 也有 hoisting 但沒有初始化為 undefined，而且在賦值之前試圖取值會發生錯誤。

# Byte code 閱讀初體驗

上面既然談到了 JS 引擎，如果沒有談到 V8 那就有點可惜，在我研究 hoisting 的時候我一直很想知道一件事情：V8 編譯出來的程式碼到底長怎樣？

感謝 [Understanding V8’s Bytecode](https://medium.com/dailyjs/understanding-v8s-bytecode-317d46c94775) 這一篇精彩的文章，可以讓我們試著用 node.js 把程式碼編譯成 byte code 並且試圖解讀。

在看之前先來介紹什麼是 byte code，它就是一種介於高階語言與機器碼中間的語言，沒有高階語言好懂可是卻比機器碼好懂許多，而執行起來的效率也比較高。

下面這張圖就是文章裡面附的，很清楚地解釋了之間的關係：

![](/img/huli/hoisting/byte.png)

接著我們用這一個簡單的 function 當作範例，來看編譯過後會長怎樣：

``` js
function funcA() {
    var a = 10
    console.log(a)
}
funcA()
```

雖然只有一個 function，但是用 node.js 跑還是會出現一大堆東西，所以我們把結果先放到檔案裡面：`node --print-bytecode test.js > byte_code.txt`

編譯出來的結果長這樣：

``` js
[generating bytecode for function: funcA]
Parameter count 1
Frame size 24
   76 E> 0xeefa4feb062 @    0 : 91                StackCheck 
   93 S> 0xeefa4feb063 @    1 : 03 0a             LdaSmi [10]
         0xeefa4feb065 @    3 : 1e fb             Star r0
  100 S> 0xeefa4feb067 @    5 : 0a 00 02          LdaGlobal [0], [2]
         0xeefa4feb06a @    8 : 1e f9             Star r2
  108 E> 0xeefa4feb06c @   10 : 20 f9 01 04       LdaNamedProperty r2, [1], [4]
         0xeefa4feb070 @   14 : 1e fa             Star r1
  108 E> 0xeefa4feb072 @   16 : 4c fa f9 fb 00    CallProperty1 r1, r2, r0, [0]
         0xeefa4feb077 @   21 : 04                LdaUndefined 
  115 S> 0xeefa4feb078 @   22 : 95                Return 
Constant pool (size = 2)
Handler Table (size = 16)
```

我們把前面一些資訊清空並加上註解，好讓大家知道上面程式碼是什麼意思（我其實也沒有真的很懂，這方面資料好像滿少的，如果有錯請糾正）：

``` js
StackCheck 
LdaSmi [10]                    // 把 10 放到 accumulator 裡面
Star r0                        // 把 accumulator 的值放到 r0 裡，所以 r0 = 10
LdaGlobal [0], [2]             // 載入一個 Global 的東西到 acc 裡
Star r2                        // 把它存到 r2，根據後見之明，r2 應該就是 console
LdaNamedProperty r2, [1], [4]  // 載入一個 r2 的 Property（應該就是 log）
Star r1                        // 把它存到 r1，也就是 r1 = console.log
CallProperty1 r1, r2, r0, [0]  // console.log.call(console, 10)
LdaUndefined                   // 把 undefined 放到 acc
Return                         // return undefined
```

再來我們把順序顛倒，變成這樣：

``` js
function funcA() {
    console.log(a)
    var a = 10
}
funcA()
```

來看看輸出的 byte code 會變什麼樣子，看解釋之前你可以先對照一下上面的，看看差別在哪：

``` js
StackCheck
LdaGlobal [0], [2]             // 載入一個 Global 的東西到 acc 裡
Star r2                        // 把它存到 r2，根據後見之明，r2 應該就是 console
LdaNamedProperty r2, [1], [4]  // 載入一個 r2 的 Property（應該就是 log）
Star r1                        // 把它存到 r1，也就是 r1 = console.log
CallProperty1 r1, r2, r0, [0]  // console.log.call(console, undefined)
LdaSmi [10]                    // 把 10 放到 accumulator 裡面
Star r0                        // 把 accumulator 的值放到 r0 裡，所以 r0 = 10
LdaUndefined                   // 把 undefined 放到 acc
Return                         // return undefined 
```

其實只是順序調換了一下，在輸出的地方直接 log 了 r0，這邊我不確定的是 r0 原本就是 undefined，還是在其他地方被初始化成 undefined。

再來我們看看如果試圖印出一個未宣告的變數會發生什麼事：

``` js
function funcA() {
    console.log(b)
    var a = 10
}
funcA()
```

因為大部分程式碼都跟前面重複我就不再註解了：

``` js
StackCheck 
LdaGlobal [0], [2]
Star r2
LdaNamedProperty r2, [1], [4]
Star r1
LdaGlobal [2], [6]  // 試圖載入 b 的值，出錯
Star r3
CallProperty1 r1, r2, r3, [0]
LdaSmi [10]
Star r0
LdaUndefined 
Return     
```

整段的重點只有`LdaGlobal`那行，看起來應該是去載入 b 的值，在執行的時候應該也就是這行出錯，因為在 global 裡面找不到 b。

看完了基本的之後，我們來看看 let 會編譯成什麼樣子：

``` js
function funcA() {
    console.log(a)
    let a = 10
}
funcA()
```

編譯後的結果：

``` js
LdaTheHole                    // 把 hole 載入到 acc 去
Star r0                       // r0 = hole
StackCheck 
LdaGlobal [0], [2]            
Star r2                       // r2 = console
LdaNamedProperty r2, [1], [4]
Star r1                       // r1 = console.log
Ldar r0                       // 載入 r0
ThrowReferenceErrorIfHole [2] // 拋出錯誤
CallProperty1 r1, r2, r0, [0] // console.log.call(console, r0)
LdaSmi [10]
Star r0
LdaUndefined 
Return
```

你會看到多了一個神秘的東西叫做 hole，這個其實就是我們所說的 TDZ，所以才會有 ThrowReferenceErrorIfHole 那一行，就代表說在 TDZ 結束之前我們如果試圖去存取這個 hole 的值都會拋出錯誤。

至此，也解釋了 TDZ 實際上在編譯階段是如何運作的，就是透過 hole 這個特別的東西。

# 總結

最近開始補齊自己對 JavaScript 的一些基礎知識，不補還好，一補下去發現自己懂的東西比自己想像中還少，我要先感謝兩篇文章：[解读ECMAScript[1]——执行环境、作用域及闭包](http://www.cnblogs.com/leoo2sk/archive/2010/12/19/ecmascript-scope.htm)、[JS 作用域](https://github.com/nightn/front-end-plan/blob/master/js/js-scope.md)，這兩篇是我的啟蒙導師，如果沒看到這兩篇，大概也不會有這篇文章的出現。

JavaScript 常考的幾個點大家都耳熟能詳：this、prototype、clousre 跟 hoisting，而這幾個看似不相關的東西，其實只要你能理解 JavaScript 背後的運作模型，都能夠多少串得起來，成為一個完整的理論。

我在文中也有提到，上面講述執行環境的那段過程，其實補充得更完整以後就可以拿來解釋 clousre，就會發現很多東西其實都能融會貫通。日後有機會可以把這整套變成一系列，一一擊破 JavaScript 那些你以為很難但其實沒有的概念。

寫這篇以前我大概醞釀了一個月，不斷找資料以後消化並且轉化為自己的理解，也很感謝上面那篇 JS 作用域的作者以及 YDKJS 的作者耐心解惑。

最後，也希望這篇文章對你們有幫助，有任何錯誤都可以跟我反映，感謝。

參考資料：

1. [MDN: Hoisting](https://developer.mozilla.org/zh-TW/docs/Glossary/Hoisting)
2. [ECMA-262-3 in detail. Chapter 2. Variable object.](http://dmitrysoshnikov.com/ecmascript/chapter-2-variable-object/#phases-of-processing-the-context-code)
3. [JS 作用域](https://github.com/nightn/front-end-plan/blob/master/js/js-scope.md)
4. [JavaScript Optimization Patterns (Part 2)](http://benediktmeurer.de/2017/06/29/javascript-optimization-patterns-part2/)
5. [danbev/learning-v8](https://github.com/danbev/learning-v8)
6. [Why is there a “temporal dead zone” in ES6?](http://2ality.com/2015/10/why-tdz.html)
7. [exploringjs: Variables and scoping #](http://exploringjs.com/es6/ch_variables.html#_the-temporal-dead-zone)
8. [ES6 中的 TDZ（temporal dead zone）及函数作用域](https://sanster.xyz/2017/01/21/ES6-%E4%B8%AD%E7%9A%84-temporal-dead-zone/)
9. [由阮一峰老师的一条微博引发的 TDZ 思考](https://www.jianshu.com/p/ebc51ce05416)
10. [理解ES6中的暂时死区(TDZ)](https://segmentfault.com/a/1190000008213835)
11. [TEMPORAL DEAD ZONE (TDZ) DEMYSTIFIED](http://jsrocks.org/2015/01/temporal-dead-zone-tdz-demystified)
12. [MDN: let](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Statements/let#Another_example_of_temporal_dead_zone_combined_with_lexical_scoping)
13. [Grokking V8 closures for fun (and profit?)](https://mrale.ph/blog/2012/09/23/grokking-v8-closures-for-fun.html)
14. [解读ECMAScript[1]——执行环境、作用域及闭包](http://www.cnblogs.com/leoo2sk/archive/2010/12/19/ecmascript-scope.htm)


關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好
