---
title: 深入探討 JavaScript 中的參數傳遞：call by value 還是 reference？
date: 2018-06-23 09:20:33
tags:
  - javascript
autohr: huli
---

# 前言

其實這週原本是要來寫淺拷貝跟深拷貝的差異以及實作，但在找資料的時候無意間又看到 call by value 與 call by reference 相關的文章，越研究發現越有趣。原本以為自己已經搞懂了這個問題，但沒想到看的資料越多，卻把自己弄的越糊塗。

要寫這篇文章其實有兩個不同的方式，一個是詳實記錄我研究這個問題的過程以及心中的疑惑，以及最後如何得到解答，簡單來說就是按照時間軸來寫；另外一個是當我研究完以後，再重新以自己的方式整理，並且用更簡單易懂的方式來表達。

以往我的文章大多數都走第二種路線，重新歸納整理過後再寫出一篇相對上更容易理解的文章，用我的方式帶著大家一步步跟著我的脈絡去探討問題最後得出解答。

但這次我想嘗試第一種，帶大家看看我平常寫文章的時候都看了哪些資料，以及發想的過程為何，這樣應該也滿有趣的。

Let's go!

# 美麗的錯誤

開頭有講過了，我會再重新回來研究參數傳遞這個問題完全是個美麗的錯誤，我本來要寫的主題是深拷貝跟淺拷貝。

在找資料的時候，我查到了這篇文：[[Javascript] 關於 JS 中的淺拷貝和深拷貝
](http://larry850806.github.io/2016/09/20/shallow-vs-deep-copy/)，我看了看之後發現如果我要來講深拷貝，我就必須先講解為什麼我們需要深拷貝，就要講到 Object 跟其他 Primitive types 的不同之處。

想到這邊，我就想到了一個老問題：JavaScript 的 Object 到底是 pass by value 還是 pass by referece？

我依稀記得答案是前者，或者兩者都不是，而是有個新的名詞叫作 pass by sharing。

為了驗證自己的印象沒錯，我繼續動手搜尋，最後找到了[[筆記] 談談JavaScript中by reference和by value的重要觀念](https://pjchender.blogspot.com/2016/03/javascriptby-referenceby-value.html)以及[重新認識 JavaScript: Day 05 JavaScript 是「傳值」或「傳址」？](https://ithelp.ithome.com.tw/articles/10191057)，後者我有印象我看過，而且驗證了我的印象是正確的。

好，話說到這裡，必須先跟大家介紹一下這三者以及之間的差異，否則沒辦法繼續往下講。

# Function 的參數傳遞方式

先來一個很簡單的範例：

``` js
function swap(a, b) {
  var temp = a;
  a = b;
  b = temp;
}
  
var x = 10;
var y = 20;
swap(x, y);
console.log(x, y) // 10, 20
```

當你執行完`swap`之後，`x`跟`y`的值並沒有交換，為什麼？因為你傳進去的東西「不是真的 x 跟 y」，而是「x 跟 y 的值的拷貝」。

也就是說`a`跟`b`其實就是另外兩個新的變數，然後存的值跟`x`和`y`一樣，但你改變了`a`不會改變`x`，因為他們是兩個不同的變數。

可以參考下面的精美小動畫：

![](/img/huli/value/value.gif)

上面這種方式就叫做：call by value（或是 pass by value），在呼叫 function 的時候把「值」給複製一份。

到這邊應該還滿好懂的，接下來要開始慢慢進入到複雜的部分了。有另外一種方法，叫做 call by reference，意思是「你傳進去的東西就是真的 x 跟 y，function 裡面的 a 跟 b 只是別名（alias）而已，改變 a 就會改變 x」

很顯然的，在 JavaScript 裡面對於像數字這種的 Primitive type，是沒有 call by reference 的，因為你絕對不可能透過 function 內的引數去改變 function 外面的變數。

對於數字不可能，那對 object 呢？

``` js
function add(obj) {
  obj.number++
}
  
var o = {number: 10}
add(o)
console.log(o.number) // 11
```

哪泥！居然在 function 裡面成功改變外面的東西了！難道這就是 call by reference 嗎？

先別急，乍看之下很像，可是有一個操作會露出破綻：

``` js
function add(obj) {
  // 讓 obj 變成一個新的 object
  obj = {
    number: obj.number + 1
  }
}
  
var o = {number: 10}
add(o)
console.log(o.number) // 10
```

如果是真的 call by reference，那你在 function 裡面把 obj 的值改掉了，外面的 o 也會一起被改掉，變成那個新的 object，可是從上面這段範例看起來並沒有，所以這樣做不是 call by reference。

既不是 call by value 也不是 call by reference，那這樣應該叫做什麼呢？

有人把這種方式叫做 call by sharing，意思就是我們讓 function 裡面的那個`obj`跟外面的`o`「共享」同一個 object，所以透過裡面的 obj，你可以去修改「共享到的那個 object」的資料。

上面都跟 call by reference 看起來沒兩樣，但最大的差異是如果你在 function 裡面把 obj 重新賦值，就代表你要讓這個 obj 指向一個新的 object，所以外面的 o 依舊還是原來的值。

![](/img/huli/value/ref.png)

引入了一個新名詞之後，看起來所有問題都得到了解答，結論就是：「在 JavaScript，primitive types 是 call by value，object 是 call by sharing」

不過，這一切只是我天真的想法而已，某天我看到一句話...

# JavaScript 只有 call by value

這句話乍看之下完全沒道理，剛剛不是說是 call by sharing 嗎？怎麼又變成 call by value 了？

但其實這句話是要這樣解讀的：

當你在宣告一個 object 的時候，在底層實作上，其實這個 object 存的是一個記憶體位置，或如果用 C 的方式來講，object 的底層就是一個指標。

先幫大家複習一下指標，你可以把指標看成是變數型態的一種，差別在於它所儲存的值是「記憶體位置」。

![](/img/huli/value/p1.png)

> o 這個變數的值是什麼？

這個問題的答案是我認為理解「JavaScript 只有 call by value」這句話的關鍵。

如果從上層來看，答案理所當然會是：「o 的值是 {number: 10}」。可是如果你從底層實作的角度來看，答案就會是：「o 的值是 0x01」

我們用第二個答案繼續往下講，假設 o 的值是 0x01 的話，那你在呼叫 function 的時候，傳進去的值其實就是 0x01，所以在 function 裡面的變數才可以透過這個記憶體位置去操作同樣的東西。

就是我們前面那張圖畫的，o 跟 obj 兩個變數會「指向」同一個地方。而底層實作原理就是把 o 的記憶體位置傳給 obj 嘛，不然怎麼能指向同個地方。

如果以這個角度來看，call by sharing（傳記憶體位置進去）其實就是 call by value 的一種，解釋的方式為：其實一樣是傳值的拷貝進去，只是這個值是記憶體位置。

乍聽之下有點道理，可是有個點我怎麼想都想不通：

> 如果你要從底層實作的原理來看，那 call by reference 不也是 call by value 的一種嗎？

因為以底層來看，call by reference 一樣也是傳記憶體位置進去啊，那不就全世界都只有 call by value？

後來我查到了一篇文章跟我有類似的想法：[Re: [問題] 請問傳參考到底是什麼?](https://www.ptt.cc/bbs/C_and_CPP/M.1245595402.A.2A1.html)

不過看完之後還是沒有得到解答，只有個模糊的概念，覺得這可能是一個名詞定義的問題。

抱著追根究柢的精神，我決定來看看 ECMAScript 怎麼說。

# 探索聖經的路程

ECMAScript 的 spec 就是 JavaScript 的聖經，在裡面你可以找到更底層的實作，而且內容絕對不會出錯。

目前能找到的相關文章，大部分的參考資料來源都是這裡：[ECMA-262-3 in detail. Chapter 8. Evaluation strategy.](http://dmitrysoshnikov.com/ecmascript/chapter-8-evaluation-strategy/)

我原本以為這篇是 ECMA-262-3 的節錄，看完之後發現根本不是，其實只是某個人看完 ECMA-262-3 之後的筆記而已。

不過這篇其實寫得很不錯，我們可以直接看結論的部分：

> It can be either “call by value”, with specifying that the special case of call by value is meant — when the value is the address copy. From this position it is possible to say that everything in ECMAScript are passed by value.

> Or, “call by sharing”, which makes this distinction from “by reference”, and “by value”. In this case it is possible to separate passing types: primitive values are passed by value and objects — by sharing.

> The statement “objects are passed by reference” formally is not related to ECMAScript and is incorrect.

但可惜的是沒有說 ECMA-262 裡面到底哪個部分有提到這些，而且我怎麼查都查不到有任何人的文章有附上 ECMA-262 的參考來源。

沒辦法，只好自己找了。

我從[ecma international](https://www.ecma-international.org/publications/standards/Ecma-262.htm)上面下載了`ECMA-262 edition 8`，並且利用幾個關鍵字來找：

1. call by reference
2. call by value
3. pass by reference
4. pass by value

結果呢？結果一無所獲，完全搜尋不到這些字。接著只好把關鍵字縮小一點，利用：`reference`、`sharing`等等的關鍵字去找，找到`6.2.4 The Reference Specification Type`，雖然看似相關，但沒有找到最關鍵的部分。

八百多頁的文章，這樣慢慢找實在是很累，而這樣子找下來，依舊沒有任何收穫。接著我轉個念頭：「那我來搜尋 arguments 好了」，找到兩個看似相關的章節（`9.4.4 ArgumentsExoticObjects` 與 `9.2 ECMAScript Function Objects`），但依舊沒有詳細說明。

用上面的關鍵字都找不到，我決定再換個念頭：「那我來查等號的定義好了，要比較 object 的話，應該會寫說如何比較兩個 object 是否相同，應該就會提到 reference 之類的相關詞彙了！」

最後查到了這段：

![](/img/huli/value/ecma1.png)

> 8. If x and y are the same Object value, return true. Otherwise, return false.

好，有說跟沒說一樣。查了一兩個小時發現幾乎沒進展以後，我決定放棄這個接近九百頁的版本。

後來我去下載了[ECMA-262 的第一版](https://www.ecma-international.org/publications/files/ECMA-ST-ARCH/ECMA-262,%203rd%20edition,%20December%201999.pdf)，篇幅少很多，只有 200 頁不到，在搜尋了幾個關鍵字發現還是沒什麼結果之後，我決定把整本快速掃過一遍。

先講結論，我還是沒有找到任何跟 call by value/reference 有關的地方，可是看到一些滿有趣的東西。

例如說判斷是否相等的地方寫的不太一樣：

![](/img/huli/value/ecma2.png)

> 11.9.3 The Abstract Equality Comparison Algorithm
> 
> 13.Return true if x and y refer to the same object or if they refer to objects joined to each other (see 13.1.2). Otherwise, return false.

提到了一個叫做 joined objects 的東西：

![](/img/huli/value/ecma3.png)

不過跟我們想找的地方還是不太一樣。

於是，我放棄了從 ECMAScript 去找答案的這個想法。

在覺得無助的同時，想起了一個也有著相似問題（到底是 call by value 還是 call by reference）的程式語言：Java。

# Java is always pass-by-value

以前在寫 Java 的時候也有碰過這個問題，而且跟 JavaScript 的其實一模一樣，就是你傳一般的值進去是 by value，可是你傳 object 進去的時候又表現的像 call by reference，但是賦值的時候又不會改變外面的 object。

但看起來 Java 永遠都是 pass by value 已經是個共識了，可參考 [Is Java “pass-by-reference” or “pass-by-value”?](https://stackoverflow.com/questions/40480/is-java-pass-by-reference-or-pass-by-value)、[Parameter passing in Java - by reference or by value?](http://www.yoda.arachsys.com/java/passing.html) 跟 [Java is Pass-by-Value, Dammit!](http://www.javadude.com/articles/passbyvalue.htm)。

理由其實跟我們最開始說的一樣，讓我節錄 Java is Pass-by-Value, Dammit! 的其中一句：

> However, Objects are not passed by reference. A correct statement would be Object references are passed by value.

以及 Parameter passing in Java - by reference or by value? 的其中一段：
 
> Now that we have some definitions of terms we can return to the question. Does Java pass objects by reference or by value?

> The answer is NO! The fact is that Java has no facility whatsoever to pass an object to any function! The reason is that Java has no variables that contain objects.

> The reason there is so much confusion is people tend to blur the distinction between an object reference variable and an object instance. All object instances in Java are allocated on the heap and can only be accessed through object references. So if I have the following:

> StringBuffer g = new StringBuffer( "Hello" );
> 
> The variable g does not contain the string "Hello", it contains a reference (or pointer) to an object instance that contains the string "Hello".

g 這個變數的值並不是字串`Hello`，而是`一個指到字串 Hello 的 reference`，所以你在呼叫 function 的時候，傳進去的就是這個 reference。

> 我傳進去的是 reference，可是這樣並不叫 call by reference？

聽起來超級無敵奇怪，但根本原因其實是「此 reference 非彼 reference」，我節錄一段[Call by value？](https://openhome.cc/Gossip/JavaEssence/CallByValue.html)中的內容：

> Java 中 Call by value，指的是傳遞參數時，一律傳遞變數所儲存的值，無論是基本型態或是類別宣告的型態都一樣，Java 中不允許處理記憶體位址，所以用了「參考」這個名稱來作為解釋類別型態所宣告的變數之行為，但這邊的「參考」與 C++ 中所稱之「參考」，是完全不相同的行為，更不會有 C++ 中參數的傳值、傳參考、return 的傳值、傳參考的 Call by reference 行為。 

就是呢，我們傳進去的的確是 reference，但這個 reference 跟 C++ 裡面所稱的「call by reference」其實是不一樣的，所以不能稱作「call by reference」。

這一段其實跟犀牛書裡面[11.2. By Value Versus by Reference](https://docstore.mik.ua/orelly/webprog/jscript/ch11_02.htm)提到的是差不多的：

> Before we leave the topic of manipulating objects and arrays by reference, we need to clear up a point of nomenclature.

> The phrase "pass by reference" can have several meanings. To some readers, the phrase refers to a function invocation technique that allows a function to assign new values to its arguments and to have those modified values visible outside the function.
> 
> This is not the way the term is used in this book. Here, we mean simply that a reference to an object or array -- not the object itself -- is passed to a function. A function can use the reference to modify properties of the object or elements of the array. But if the function overwrites the reference with a reference to a new object or array, that modification is not visible outside of the function. 
> 
> Readers familiar with the other meaning of this term may prefer to say that objects and arrays are passed by value, but the value that is passed is actually a reference rather than the object itself

不過這個時候我有了另外一個疑問：那 C++ 裡面的 call by reference 到底是怎樣？

嗯，看來是時候複習一下很久沒碰的 C 跟 C++了。

# C 與 C++ 的參數傳遞

先從 C 開始吧，C 裡面就只有一種：call by value。

``` c
#include <stdio.h>
  
void swap(int a, int b) {
  int temp = b;
  b = a;
  a = temp;
}
  
int main(){
  int x = 10;
  int y = 20;
  swap(x, y);
  printf("%d %d\n", x, y); // 10, 20
}
```

就像我們一開始所說的，這樣子並不會把`x`跟`y`的值交換，因為`a`跟`b`只是儲存的值跟`x`與`y`一樣而已，除此之外一點關係都沒有。

可是呢，我們之前有提到，C 裡面有個東西叫做「指標」，能夠儲存記憶體位置。透過指標我們其實可以在 function 裡面更改外部變數的值。

``` c
#include <stdio.h>
  
void swap(int *a, int *b) {
  // 印出 a 跟 b 所存的值
  printf("%ld, %ld", a, b); //0x44, 0x40
  int temp = *b;
  *b = *a;
  *a = temp;
}
  
int main(){
  int x = 10;
  int y = 20;
  // 印出 x 跟 y 的記憶體位置
  printf("%ld %ld\n", &x, &y); // 0x44, 0x40
  swap(&x, &y); // 傳記憶體位置進去
  printf("%d %d\n", x, y); // 20, 10
}
```

我們這次傳進去 function 的不是一個變數，而是一個記憶體位置，在`swap`裡面用指標來接受這個記憶體位置，接著就可以透過指標的操作把外面`x`與`y`的值改掉。

這樣依然叫做 call by value，如果你還是不清楚為什麼，可以參考下面這個範例。跟上面的差別在於我先宣告兩個指標指向`x`跟`y`：

``` c
#include <stdio.h>
  
void swap(int *a, int *b) {
  
  // 印出 a 跟 b 所存的值
  printf("%ld, %ld", a, b); //0x44, 0x40
  int temp = *b;
  *b = *a;
  *a = temp;
}
  
int main(){
  int x = 10;
  int y = 20;
    
  // 兩個指標指向 x 跟 y
  int* ptr_x = &x;
  int* ptr_y = &y;
  
  // 印出 x 跟 y 的記憶體位置（就是 ptr_x 跟 ptr_y 存的值）
  printf("%ld %ld\n", ptr_x, ptr_y); // 0x44, 0x40
  swap(ptr_x, ptr_y); // 傳記憶體位置進去
  printf("%d %d\n", x, y); // 20, 10
}
```


還記得前面說過的 call by value 的定義嗎？就是把變數的值複製一份傳進去。這邊也是一樣的，我們傳進去的兩個變數`ptr_x`跟`ptr_y`儲存了`x`跟`y`的記憶體位置，而我們在呼叫 function 的時候就把這兩個「值」給複製一份傳進去，所以 function 裡面的`a`跟`b`印出來的值就會跟`ptr_x`以及`ptr_y`存的值一樣。

簡單來說就是以前我們 call by value 的「value」可能是數字，可能是字串，而現在的範例這個 value 是「記憶體位置」，也是資料型態的一種。

不過，也有人把這樣子稱為 call by pointer 或是 call by address，但原則上都是 call by value 的一種。

在這邊還有一個可以特別注意的地方，那就是儘管`a`跟`ptr_x`的「值」一樣，但這兩個還是不一樣的變數，有著不同的記憶體位置。

再來我們看 C++ 中的 call by reference 到底是怎樣，只要在 function 的引數那裡加上`&`，就會變成 call by reference：

``` c++
#include <stdio.h>
  
// 注意到這邊多了 &，其他都跟 call by value 一模一樣
void swap(int &a, int &b) {
  
  // 印出 a 跟 b 所存的值與記憶體位置
  printf("%ld, %ld\n", a, b); // 10, 20
  printf("%ld, %ld\n", &a, &b); // 0x44, 0x40
  int temp = b;
  b = a;
  a = temp;
}
  
int main(){
  int x = 10;
  int y = 20;
  
  // 印出 x 跟 y 的記憶體位置
  printf("%ld %ld\n", &x, &y); // 0x44, 0x40
  swap(x, y); // 傳記憶體位置進去
  printf("%d %d\n", x, y); // 20, 10
}
```

在這裡`a`跟`b`的記憶體位置與`x`跟`y`一模一樣，說明了在裡面操作`a`這個變數的時候，就是在操作`x`這個變數，兩者是一模一樣的，只是有了不同的名稱。當`a`重新賦值的時候，也會一併把外面`x`的值一起改掉。

看完了 C 跟 C++ 裡面 pass by value 跟 pass by reference 的區別，我開頭的疑惑：「如果你要從底層實作的原理來看，那 call by reference 不也是 call by value 的一種嗎？」就被解決了。

我認為這兩個最大的差異就在於一件事情：複製。

call by value 會把傳進去的值複製（無論那個值是數字也好，記憶體位置也好，都會複製一份），call by reference 在「最底層的實作」上當然也會有類似的行為，但是你感覺不出來。

就像我上面 call by reference 舉例的那段程式碼一樣，`x`的記憶體位置跟`a`一樣，`y`的記憶體位置跟`b`一樣，因此你可以說他們兩者是「一模一樣」的東西。

可是在 call by value 的範例中，就算你傳的是指標好了，只有「指標裡面存的值（也就是指到的記憶體位置）」是一樣的，但指標本身還是有不同的記憶體位置。

換句話說，在 call by value 的時候我們是「新建了一個變數`a`，並且讓`a`存的值跟傳進來的參數一樣」。在 call by reference 的時候，我們只是「讓`a`作為`x`的 alias，兩個是同樣的變數」，這是我認為這兩間之間最大的差異。


# 結論

我們從各個程式語言裡面看到了每一種程式語言的實現，那到底有沒有一種明確的定義，能夠區分 pass by value 以及 pass by reference 呢？

我想了想，其實可以從「行為」上面來判別到底是屬於哪一種。與其由定義來看，不如直接從行為來加以區分，不同種類能夠達成的行為都不一樣。第一個條件用來區分到底是 pass by value 還是 pass by reference：「在函式裡對引數重新賦值，外面變數是否會改變？」

以 JavaScript 跟 Java 為例，在函式裡面重新賦值，外面的變數都不會變，所以就是屬於 pass by value。

如果你還想分得更細，來可以透過第二個條件來區分這個 pass by value 是真・pass by value 還是一個叫做 pass by sharing 的分支：「能否透過引數，改變外部變數的值」（我們這邊所指的「值」跟地址或引用無關，純粹在講像`{numer:1}`這樣子的值）

在 JavaScript 跟 Java 你都可以透過`obj.number = 10`之類的操作改變外部變數的值（obj.number 從 1 變成了 10），所以也能說是 pass by sharing。

![](/img/huli/value/con.png)

根據第一個定義：「在函式裡對引數重新賦值，外面變數是否會改變？」，有人可能會發現如果是 C 裡面的指標，不是也可以達成嗎？可是 C 又說只有 call by value，不就衝突了嗎？

但其實在指標的範例裡面，我們重新賦值的對象是`*a`而不是`a`（意思就是，我們是讓`*a=10`而不是`a=10`），但後者才叫對引數重新賦值（給`a`一個新的地址），前者是「對指標所指向的記憶體位置重新賦值」。所以照這個定義來看，指標的範例依舊是 pass by value。

依據細分程度的不同，下面幾句話都是正確的：

1. JavaScript 裡面只有 pass by value
2. JavaScript 的 primitive type 是 pass by value，object 是 pass by sharing

# 心得

說實在的，其實我查了這麼一大堆資料之後，發現大家對 call by reference 以及 call by value 的「定義」其實都不盡相同，而且也沒有一個權威性的出處能夠保證這個定義是正確的（或許有但我沒找到，如果你知道的話請一定要告訴我在哪裡，拜託），才造成這麼多的歧異性。

有關技術名詞的解釋，我最喜歡引用這篇：[技術名詞紛爭多](https://www.ithome.com.tw/voice/94877)：

> 程式開發的世界中，名詞的創造經常是隨意的，曾經在 Java 中爭執不斷的考古題之一是：「Java 中有沒有 Pass by reference」，就現今來說，大家公認的答案是沒有，Java 只有 Pass by value，不過還是有人面對 Java 文件中經常出現 reference，而搞不清楚。

> 說穿了，這個名詞與 C++ 中的 reference 定義不同，只不過 Java 最初不知道為什麼，也用了 reference 一詞，重點也不在搞清楚 Pass by value，重點是搞清楚透過參數操作物件時，會有什麼樣的行為。

我們從 JavaScript 研究到 Java，再從 Java 研究到 C 與 C++，為的就是想要搞清楚「pass by reference」的定義為何，但追根究底，會造成這樣子的誤會是因為對於「reference」一詞的定義不同。

如果你把 pass by reference 理解成像 C++ 那樣子的定義，那 Java 跟 JavaScript 都不會有 pass by reference。但如果你把 pass by reference 的「reference」理解成「對於物件的參考」，那 JavaScript 把 object 傳進去，其實就是把「對物件的參考」傳進去，那就可以解釋成是 pass by reference。

都是 reference 這個名詞太好用了，導致不同地方有不同的定義，但那些定義往往相似卻又不全然相同。

可是別忘了，重點其實不在這個，而是搞清楚到底參數在操作的時候會有怎樣的行為。你要知道 JavaScript 傳 object 進去的時候，可以更改原本物件的值，但重新賦值並不會影響到外部的 object。只要知道這一點，其他的我覺得都沒那麼重要了。

這次寫了一個很容易引戰的主題，但也覺得滿有趣的，如果你對這問題有不同的見解，覺得我有哪邊寫錯的話，歡迎指正，感謝。

# 參考資料
1. [[Javascript] 關於 JS 中的淺拷貝和深拷貝
](http://larry850806.github.io/2016/09/20/shallow-vs-deep-copy/)
2. [[筆記] 談談JavaScript中by reference和by value的重要觀念](https://pjchender.blogspot.com/2016/03/javascriptby-referenceby-value.html)
3. [重新認識 JavaScript: Day 05 JavaScript 是「傳值」或「傳址」？](https://ithelp.ithome.com.tw/articles/10191057)
4. [Re: [問題] 請問傳參考到底是什麼?](https://www.ptt.cc/bbs/C_and_CPP/M.1245595402.A.2A1.html)
5. [ECMA-262-3 in detail. Chapter 8. Evaluation strategy.](http://dmitrysoshnikov.com/ecmascript/chapter-8-evaluation-strategy/)
6. [簡單介紹JavaScript參數傳遞](https://www.slideshare.net/YiTaiLin/java-script-63031051)
7. [JavaScript 是传值调用还是传引用调用？](https://github.com/nodejh/nodejh.github.io/issues/32)
8. [Values vs References semantics #160](https://github.com/getify/You-Dont-Know-JS/issues/160)
9. [You Don't Know JS: Types & Grammar Chapter 2: Values
](https://github.com/getify/You-Dont-Know-JS/blob/master/types%20%26%20grammar/ch2.md#value-vs-reference)
10. [Parameter passing in Java - by reference or by value?](http://www.yoda.arachsys.com/java/passing.html)
11. [Is Java “pass-by-reference” or “pass-by-value”?](https://stackoverflow.com/questions/40480/is-java-pass-by-reference-or-pass-by-value)
12. [傳值呼叫](https://openhome.cc/Gossip/Java/PassByValue.html)
13. [Call by value？](https://openhome.cc/Gossip/JavaEssence/CallByValue.html)
14. [java中的经典问题：传值与传引用](https://blog.csdn.net/jiangnan2014/article/details/22944075)
15. [Java is Pass-by-Value, Dammit!](http://www.javadude.com/articles/passbyvalue.htm)
16. [11.2. By Value Versus by Reference](https://docstore.mik.ua/orelly/webprog/jscript/ch11_02.htm)





關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好