---
title: 每天都在 arr.map()，你聽過 functor 嗎？
date: 2019-07-18 15:59:38
tags:
    - javascript
    - functor
---

## 前言

大家都知道程式語言分為兩種，分為 `functional programming` 以及 `object oriented programming`。然而看過不少人有一種迷思，認為**物件導向的語言一定比較好**，無論是維護性，或是物件導向的種種特性，都可以將抽象化的概念更加發揮至淋漓盡致。

不可否認的是物件導向的確有需多概念將抽象化發揮的更加完全，提升軟體工程的效率。但今天我想談的是，那些 `functional programming` 我們忽略掉的特性，這些特性也能幫助我們將程式碼進一步抽象化。

舉例來說，在 JavaScript 中，我們知道 function 有

-   first-class citizen : function 可以像其他 `primitive data type` 般，可以任意傳遞(pass)，指派(assign)
-   higher-order function : function 的回傳值也可以是一個 function

這兩種特性，或是 `閉包(Closure)`, `柯里化(Currying)` 這些概念(在此暫時不解釋，網路上有非常多解釋)。

但在茫茫 JavaScript 教學文當中，鮮少看到有人解釋 `functor` 這個特性，於是產生今天這篇文章。

## 什麼是 functor ？

講了這麼多，那什麼是 functor ？

> So, What are Functors? Functors are the containers that can be used with ‘map’ function. ---引述自 [functors-in-javascript](https://hackernoon.com/functors-in-javascript-20a647b8f39f)

簡單來說，一個容器裡面，含有 `map()` 這個屬性，就可以稱為 functor。最簡單的例子就是 JavaScript 中的 Array。

`map()` 屬性又是什麼？根據 [MDN](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Array/map) 的解釋

> The map() method creates a new array with the results of calling a provided function on every element in the calling array.

相信大家都用過 `map()`。以 Array.prototype.map() 為例，在 `map()` 的處理過程中，會先把 Array 中的每一個 element 單獨丟進 function 處理，並且將各別結果放至新的 Array 後回傳。

引用 MDN 的範例:

```javascript
var numbers = [1, 4, 9];
var roots = numbers.map(Math.sqrt); //map會return一個新的array
// roots 現在是 [1, 2, 3]
/* numbers 還是 [1, 4, 9]，這證明了 map() 不會去變動到 numbers 的值，
   map 內部是做了 immutable 的機制，Array.prototype 底下的這些高階函式
   大多都具有這樣函數式編程裡非常注重的特性 - immutable，不會去改變資料
   來源本身原有的值 
*/
```

在 [functors-in-javascript](https://hackernoon.com/functors-in-javascript-20a647b8f39f) 中有一張非常可愛且簡單明瞭的圖片說明了這件事

![example_img](https://cdn-images-1.medium.com/max/1600/1*GbH6_EAvPrtQGc9fiVZpMA.gif)

圖中的序列，分別取出每一個元素(小雞)，並且加以操作，最後將結果放置新的序列後回傳(注意！圖片中並沒有將小雞放回原本的盒子中)。

說到這裡，應該有人發現它的行為與 `Array.prototype.forEach()` 非常類似。沒錯！他們兩個差異就只是*是否修改原本的值*。

functor 有一個重要的特性就是不會修改原本的值，然而 forEach 則會直接修改原本的值。

## 結論

講了這麼多，functor 對我的生活有什麼影響嗎？

我想表達一個概念，function programming 其實也能實現 OOP 的種種功能。只是我們對 function programming 的了解不夠深入而已。

再往下討論可以討論到 `範疇論(Category Theory)`，裡面涉及了一些最根本的原始數學定義。如果覺得這篇文章不夠深入，還想對 `functional programming` 有更深入的掌握，也可以了解一下 `Monad`。

或許我的下一篇文章就是簡單的講解何謂 `Monad` 😋

## 關於作者

大家好，我是 yiyu。在網路上都用 [yiyu0x](https://github.com/yiyu0x) 作為我的 id。目前是資訊工程學系大三的學生，平常喜歡寫程式，用程式解決日常生活中的問題。
JavaScript 是一門近期我非常熱愛的語言，相較於其他語言，豐富的套件以及異步的特性是我熱愛它的原因。此篇文章也同時刊登於我的個人 [blog](https://blog.yiyu0x.tk/)。
