---
title: 每天都在 arr.map()，你聽過 functor 嗎？
date: 2019-07-18 15:59:38
tags:
    - javascript
    - functor
---

## 前言

程式設計的方法論大體來說可以分為兩種， `functional programming (FP)` 以及 `object oriented programming (OOP)`。
然而不同的程式語言可能也會提倡不同的設計方法，舉一個明確的例子，在 Java 的 Hello, world 程式：

```java
public class HelloWorld {
    public static void main(String[] args) {
        System.out.println("Hello! World!");
    }
}
```

整個程式必須包覆在一個與檔案名稱同名的物件中。

Java 這個語言可以說先天就是一門物件導向式的語言(這個說法可能有些爭議)。但這兩種設計方式都只是方法，並沒有說哪一種語言就只能歸類於哪一種設計方式中。

所以，C 語言當然也可以實現物件導向 ([你所不知道的 C 語言：物件導向程式設計篇](https://hackmd.io/@sysprog/HJLyQaQMl?type=view))。

然而看過不少人有一種迷思，認為**物件導向才是比較進步的設計方式**，無論是維護性，或是整個程式之間的架構都優於 `FP`。

不可否認的是物件導向的確有許多概念將抽象化發揮的更加完全，提升軟體工程的效率。但今天我想談的是，那些 `functional programming` 我們忽略掉的特性，這些特性也能幫助我們將程式碼進一步抽象化。

## Functional programming

在物件導向設計中我們知道有 `Class`, `Object`, `Inheritance` 等等概念。

而在 `FP` 的概念則是有 `Immutable object`, `Lambda calculus`, `First class` 等等。

### Immutable object

代表創造了該物件之後，內容就無法再被修改。這樣的特性先天就具備了在多個執行緒下的安全性。想想看，某個物件如果被多個 threads 共用，不會因為 thread A 改了某物件的內容，進而影響了 thread B。這也代表不用使用像 Lock 之類的機制來保證安全性。

舉例來說：

```javascript
var numbers = [1, 4, 9];
var roots = numbers.map(Math.sqrt); //map會return一個新的array
// roots 現在是 [1, 2, 3]
// numbers 還是 [1, 4, 9]
```

我們可以說 map() 這個函式確保了 Immutable，甚至還有套件庫可以讓全部資料型態都保持 Immutable ([immutable-js](https://github.com/immutable-js/immutable-js))。

### First class

在 JavaScript 中，我們知道 function :

1. 可以像其他 `primitive data type` 般，可以任意傳遞(pass)，指派(assign)
2. function 的回傳值也可以是一個 function

以上第一種特性可以細稱為 `first-class citizen`，第二種稱為 `higher-order function`。

而這個特性又衍伸出像 `閉包(Closure)`, `柯里化(Currying)` 等等更多的特性。

## 為什麼要提這些？

在講解 `functor` 之前，適度的理解 `functional programming` 是必要的。

`functional programming` 說穿了就是以數學的角度思考，使用 `範疇論 (Category Theory)` 的思想來解決問題。

`functor` 常與 `Applicative`, `Monad` 這三共同被提及，這三者都是 `FP` 重要的概念。

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

說到這裡，應該有人發現它的行為與 `Array.prototype.forEach()` 非常類似。沒錯！他們兩個差異就只是*是否修改原本的值*。(就是前面提過的 Immutable 特性)

functor 有一個重要的特性就是確保 Immutable，然而 forEach 則會直接修改原本的值。

## 結論

講了這麼多，`functor` 對我的生活有什麼影響嗎？

`functor` 不但可以增加程式碼的可讀性，更是 `FP` 的設計思想其中的一環。

FP 的表達能力其實不遜色於 OOP。大部分時候，只是我們對 FP 的了解不夠深入而已。

像是此篇未提及的 `applicative` 就是 `functor` 的進化版，如果說 `functor` 是把箱子(容器)中的每一個元素各別丟進 function 處理，那麼 `applicative` 可以想像為連 function 都在箱子(容器)中。我們必須從兩的不同的箱子拿出我們要的內容來做運算。

希望這篇文章不但能讓你了解何謂 `functor`，還可以進而知道 `FP` 還有其他你平常沒有發現的特性。

## 關於作者

大家好，我是 yiyu。在網路上都用 [yiyu0x](https://github.com/yiyu0x) 作為我的 id。目前是資訊工程學系大三的學生，平常喜歡寫程式，用程式解決日常生活中的問題。
JavaScript 是一門近期我非常熱愛的語言，相較於其他語言，豐富的套件以及異步的特性是我熱愛它的原因。此篇文章也同時刊登於我的個人 [blog](https://blog.yiyu0x.tk/)。
