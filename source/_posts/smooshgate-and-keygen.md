---
title: Don’t break the Web：以 SmooshGate 以及 keygen 為例
date: 2019-11-30 19:49:03
tags:
  - Web
autohr: huli
---

## 前言

最近 YDKJS（You Don't Know JS 的縮寫，中譯版翻成：你所不知道的JS）有了第二版，名叫 [YDKJSY](https://twitter.com/ydkjsy)，Y 是 Yet 的意思（中文版可能可以翻叫：你還是不知道的 JS）。這個第二版還沒全部完成，但在 [GitHub](https://github.com/getify/You-Dont-Know-JS) 上面已經公開了最前面的一些章節。

搶先讀了一下第一章，在講與 JS 相關的歷史，其中提到一段讓我很感興趣的議題：

>  As such, sometimes the JS engines will refuse to conform to a specification-dictated change because it would break that web content.

> In these cases, often TC39 will backtrack and simply choose to conform the specification to the reality of the web. For example, TC39 planned to add a contains(..) method for Arrays, but it was found that this name conflicted with old JS frameworks still in use on some sites, so they changed the name to a non-conflicting includes(..). The same happened with a comedic/tragic JS community crisis dubbed "smooshgate", where the planned flatten(..) method was eventually renamed flat(..).

大意是在說有時候 JS 的規格必須跟現實（已經存在的那些舊的實作）妥協。例如說原本 Array 要加上一個叫做 contains 的 method，但因為會有問題所以改叫 includes，flatten 也改名叫做 flat。

還有一個上面特別標起來的詞「smooshgate」，用這個當關鍵字去找才發現是去年三月左右發生的事件，至於發生了什麼，底下會詳述，跟上面提的 flatten 有關。看到有這件事的時候我第一個反應是：「咦，我怎麼什麼都不知道？」，查了一下繁體中文的資料，大概也只有這篇有提到：[SmooshGate](https://blog.othree.net/log/2018/05/28/smooshgate/)，以及[[筆記] 3 種 JavaScript 物件屬性的特性](https://medium.com/@liuderchi/%E7%AD%86%E8%A8%98-3-%E7%A8%AE-javascript-%E7%89%A9%E4%BB%B6%E5%B1%AC%E6%80%A7%E7%9A%84%E7%89%B9%E6%80%A7-3b982f4c5695)這篇有擦到邊而已。

在仔細研究了一下事情的來龍去脈之後，覺得是個滿有趣的議題，因此寫了這篇跟大家分享。

<!-- more -->

## SmooshGate 事件

有關這個事件以及這篇文章的靈感，大多數來自於：[#SmooshGate FAQ](https://developers.google.com/web/updates/2018/03/smooshgate) 這篇文章，裡面其實解釋得很好，建議大家可以去看這篇。

但懶得看也沒關係，底下我簡單講一下事情的來龍去脈。

有一個組織叫做 TC39，全名為 Technical Committee 39，第 39 號技術委員會，負責與 ECMAScript 規範相關的事項，例如說決定哪些提案可以過關之類的，而最後那些提案就會被納入新的 ECMAScript 標準之中。

提案一共分成五個 stage，從 stage0 到 stage4，詳情我就不多說明了，可以參考 [Championing a proposal at TC39](https://github.com/tc39/how-we-work/blob/master/champion.md) 或是 [The TC39 Process](https://tc39.es/process-document/)。

TC39 之前有一個提案是 [Array.prototype.{flatten,flatMap}](https://github.com/tc39/proposal-flatMap)（flatten 現在已經改為 flat）。

這邊先幫不清楚什麼是 flatten 的讀者簡單介紹一下它的功用，簡單來說就是把巢狀的東西攤平。

例如說底下範例：

``` js
let arr = [1, 2, [3], [4], [5, 6, 7]]
console.log(arr.flatten()) // [1, 2, 3, 4, 5, 6, 7]
```

原本巢狀的陣列會被攤平，這就是 flatten 的意思，跟 lodash 裡面的 [flatten](https://lodash.com/docs/4.17.15#flatten) 是差不多的。

詳細的使用方法可以參考 [MDN](https://developer.mozilla.org/zh-TW/docs/Web/JavaScript/Reference/Global_Objects/Array/flat)，就只是多了一個參數 depth 可以讓你指定展開的深度。

而 [flatMap](https://developer.mozilla.org/zh-TW/docs/Web/JavaScript/Reference/Global_Objects/Array/flatMap) 就是先 map 之後再 flat，熟悉 RxJS 的朋友們應該會感到滿親切的（在 RxJS 裡面又稱作 mergeMap，而且 mergeMap 比較常用，有興趣的朋友也可以參考這篇：[concatAll and concatMap rather than flatten and flatMap](https://github.com/tc39/proposal-flatMap/issues/60)）。

好，這個提案看似很不錯，但到底會有什麼問題呢？

問題就出在一個前端新鮮人可能沒聽過的工具：[MooTools](https://mootools.net/)，而我也只有聽過而已，完全沒用過。想要快速知道它可以幹嘛的，請看這篇十年前的比較文：[jQuery vs MooTools](http://www.jqueryvsmootools.com/index_cn.html)。

在 MooTools 裡面，他們定義了自己的 flatten method，在 code 裡面做了類似下面的事：

``` js
Array.prototype.flatten = /* ... */;
```

這聽起來沒什麼問題，因為就算 flatten 正式列入標準並且變成原生的 method，也只是把它覆蓋掉而已，沒事兒沒事兒。

但麻煩的事情是，MooTools 還有一段 code 是把 Array 的 method 都複製到 Elements（MooTools 自定義的 API）上面去：

``` js
for (var key in Array.prototype) {
  Elements.prototype[key] = Array.prototype[key];
}
```

for...in 這個語法會遍歷所有可列舉的（enumerable）屬性，而原生的 method 並不包含在裡面。

例如說在 Chrome devtool 的 console 執行以下這段 code：

``` js
for (var key in Array.prototype) {
  console.log(key)
}
```

會發現什麼都沒有印出來。

但如果你加上了幾個自定義的屬性之後：

``` js
Array.prototype.foo = 123
Array.prototype.sort = 456
Array.prototype.you_can_see_me = 789
for (var key in Array.prototype) {
  console.log(key) // foo, you_can_see_me
}
```

會發現只有自定義的屬性會是 enumerable 的，而原生的方法你就算覆寫，也還是不會變成 enumerable。

那問題是什麼呢？問題就出在當 flatten 還沒正式變成 Array 的 method 時，它就只是一個 MooTools 自定義的屬性，是 enumerable 的，所以會被複製到 Elements 去。但是當 flatten 納入標準並且被瀏覽器正式支援以後，flatten 就不是 enumerable 的了。

意思就是，`Elements.prototype.flatten` 就會變成 undefined，所有使用到這個 method 的 code 都會掛掉。

此時天真的你可能會想說：「那就把 flatten 變成 enumerable 的吧！」，但這樣搞不好會產生更多問題，因為一堆舊的 for...in 就會突然多出一個 flatten 的屬性，很有可能會造成其他的 bug。

當初發現這個 bug 的討論串可以看這裡：[Implementing array.prototype.flatten broke MooTools' version of it.](https://bugzilla.mozilla.org/show_bug.cgi?id=1443630)

確認有了這個問題以後，大家就開始討論要把 flatten 換成什麼詞，有人在 Issues 裡面提議說：[rename flatten to smoosh](https://github.com/tc39/proposal-flatMap/pull/56)，引起了廣大討論，也就是 #SmooshGate 事件的起源。除了討論改名以外，也有人認為乾脆就讓那些網站壞掉好了。

smoosh 這個字其實跟 flatten 或是其他人提議的 squash 差不多，都有把東西弄平的意思在，不過這個字實在是非常少見，聽到這事件以前我也完全沒聽過這個單字。不過這個提議其實從來沒有正式被 TC39 討論過就是了。

TC39 在 2018 年 5 月的會議上，正式把 flatten 改成 flat，結束了這個事件。

這個提案的時間軸大概是這樣：

1. 2017 年 7 月：stage 0
2. 2017 年 7 月：stage 1
3. 2017 年 9 月：stage 2
4. 2017 年 11 月：stage 3
5. 2018 年 3 月：發現 flatten 會讓 MooTools 壞掉
6. 2018 年 3 月：有人提議改名為 smoosh
7. 2018 年 5 月：flatten 改名為 flat
8. 2019 年 1 月：stage 4

我因為好奇去找了 V8 的 commit 來看，V8 是在 2018 年 3 月的時候實作這個功能的：[[esnext] Implement Array.prototype.{flatten,flatMap}](https://github.com/v8/v8/commit/697d39abff90510523f297bb8577d5c64322229f)，其中我覺得最值得大家學習的其實是測試的部分：

``` js
const elements = new Set([
  -Infinity,
  -1,
  -0,
  +0,
  +1,
  Infinity,
  null,
  undefined,
  true,
  false,
  '',
  'foo',
  /./,
  [],
  {},
  Object.create(null),
  new Proxy({}, {}),
  Symbol(),
  x => x ** 2,
  String
]);
  
for (const value of elements) {
  assertEquals(
    [value].flatMap((element) => [element, element]),
    [value, value]
  );
}
```

直接丟了各種奇形怪狀的東西進去測。

在 flatten 改名為 flat 的隔天，V8 也立刻做出修正：[[esnext] Rename `Array#flatten` to `flat`](https://github.com/v8/v8/commit/72f1abfbec0b8c798bc4cf150c774b5411d522ae)。

簡單總結一下，總之 #SmooshGate 事件就是：

1. 有人提議新的 method：`Array.prototype.flatten`
2. 發現會讓 MooTools 壞掉，因此要改名
3. 有人提議改名 smoosh，也有人覺得不該改名，引起一番討論
4. TC39 決議改成 flat，事情落幕

其中的第二點可能有些人會很疑惑，想說 MooTools 都是這麼古早的東西了，為什麼不直接讓它壞掉就好，反正都是一些老舊的網站了。

這就要談論到制定 Web 相關標準時的原則了：Don't break the web。

## Don’t break the Web

這個網站：[Space Jam](https://www.spacejam.com/archive/spacejam/movie/jam.htm) 過了 22 年，依舊可以順利執行，就是因為在制定網頁相關新標準時都會注意到「Don’t break the Web」這個大原則。

仔細想想，好像會發現 Web 的領域沒有什麼 breaking change，你以前可以用的 JS 語法現在還是可以用，只是多了一些新的東西，而不是把舊的東西改掉或者是拿掉。

因為一旦出現 breaking change，就可能會有網站遭殃，像是出現 bug 甚至是整個壞掉。其實有很多網站好幾年都沒有在維護了，但我們也不應該讓它就這樣壞掉。如果今天制定新標準時有了 breaking change，最後吃虧的還是使用者，使用者只會知道網站壞了，卻不知道是為什麼壞掉。

所以在 SmooshGate 事件的選擇上，比起「flatten 就是最符合語義，讓那些使用 MooTools 的老舊網站壞掉有什麼關係！」，TC39 最終選擇了「把 flatten 改一下名字就好，雖然不是最理想的命名，但我們不能讓那些網頁壞掉」。

不過話雖如此，這不代表糟糕的設計一旦出現以後，就完全沒有辦法被移除。

事實上，有些東西就悄悄地被移除掉了，但因為這些東西太過冷門所以你我可能都沒注意到。

[WHATWG 的 FAQ](https://whatwg.org/faq#removing-bad-ideas) 有寫到：

> That said, we do sometimes remove things from the platform! This is usually a very tricky effort, involving the coordination among multiple implementations and extensive telemetry to quantify how many web pages would have their behavior changed. But when the feature is sufficiently insecure, harmful to users, or is used very rarely, this can be done. And once implementers have agreed to remove the feature from their browsers, we can work together to remove it from the standard.

底下有提到了兩個範例：`<applet>` 與 `<keygen>`。

也是因為好奇，所以我又去找了一些相關資料來看。

## 被淘汰的 HTML 標籤

有聽過`<keygen>`這個標籤的請舉手一下？舉手的人麻煩大家幫他們鼓鼓掌，你很厲害，封你為冷門 HTML 標籤之王。

我就算看了 [MDN](https://developer.mozilla.org/en-US/docs/Web/HTML/Element/keygen) 上面的範例，也沒有很清楚這個標籤在幹嘛。只知道這是一個可以用在表單裡的標籤，人如其名，是用來產生與憑證相關的 key 用的。

從 MDN 給的資料 [Non-conforming features](https://html.spec.whatwg.org/multipage/obsolete.html#non-conforming-features) 裡面，我們可以進一步找到其他也被淘汰的標籤，例如說：

1. applet
2. acronym
3. bgsound
4. dir
5. isindex
6. keygen
7. nextid

不過被標示為 obsolete 不代表就沒有作用，應該只是說明你不該再使用這些標籤，因為我猜根據 don't break the web 的原則，裡面有些標籤還是可以正常運作，例如說小時候很愛用的跑馬燈 marquee 也在 Non-conforming features 裡面。 

在另外一份 [DOM 相關的標準](https://html.spec.whatwg.org/multipage/dom.html#elements-in-the-dom)當中，有說明了該如何處理 HTML 的 tag，我猜這些才是真的被淘汰而且沒作用的標籤：

> If name is applet, bgsound, blink, isindex, keygen, multicol, nextid, or spacer, then return HTMLUnknownElement.

如果你拿這些標籤到 Chrome 上面去試，例如說這樣：

``` html
<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8">
  </head>
  <body>
    <bgsound>123</bgsound>
    <isindex>123</isindex>
    <multicol>123</multicol>
    <foo>123</foo>
  </body>
</html>
```

就會發現表現起來跟`<span>`差不多，猜測 Chrome 應該會把這些不認識的 tag 當作 span 來看待。

再來因為好奇，所以也去找了一下 chromium 裡相關的程式碼，我以前都是直接在 GitHub 上面去搜尋 code 的內容，但因為這次要搜的關鍵字重複性太高，因此改成搜 commit message。這個時候就完全突顯 commit message 的重要性了，發現 chromium 的 commit message 寫得滿好的。

例如說這個 commit：[Remove support for the obsolete <isindex> tag.](https://github.com/chromium/chromium/commit/dfd5125a0002df42aa6c6133b3aa591953880f4e)

```
This patch removes all special-casing for the <isindex> tag; it
now behaves exactly like <foo> in all respects. This additionally
means that we can remove the special-casing for forms containing
<input name="isindex"> as their first element.
  
The various tests for <isindex> have been deleted, with the
exception of the imported HTML5Lib tests. It's not clear that
we should send them patches to remove the <isindex> tests, at
least not while the element is (an obsolete) part of HTML5, and
supported by other vendors.
  
I've just landed failing test results here. That seems like
the right thing to do.
  
"Intent to Remove" discussion: https://groups.google.com/a/chromium.org/d/msg/blink-dev/14q_I06gwg8/0a3JI0kjbC0J
```

有附上當初的討論串，資訊給的很詳細。而 code 的改動除了測試的部分以外，就是把有關這個 tag 的地方都刪掉，當作是一個不認識的 tag，所以 message 才會說：「it now behaves exactly like `<foo>` in all respects.」

再來我們看另外一個 commit：[Remove support for the keygen tag](https://github.com/chromium/chromium/commit/5d916f6c6b47472770e03cb483f06a18ca79a0c2)

```
This removes support for <keygen> by updating it
to be an HTMLUnknownElement. As a result, it's
no longer a form-associated element and no
longer has IDL-assigned properties.
  
The <keygen> tag is still left in the parser,
similar to <applet>, so that it maintains the
document parse behaviours (such as self-closing),
but is otherwise a neutered element.
  
Tests that were relying on <keygen> having its
own browser-created shadow root (for its custom
select element) have been updated to use
progress bars, while other tests (such as
<keygen>-related crash tests) have been
fully removed.
  
As Blink no longer treats this tag as special,
all the related IPC infrastructure is removed,
including preferences and enterprise flags,
and all localized strings, as they're all now
unreachable.
  
This concludes the "Intent to Remove" thread
for <keygen> at
https://groups.google.com/a/chromium.org/d/msg/blink-dev/z_qEpmzzKh8/BH-lkwdgBAAJ
```

因為`<keygen>`這個 tag 原本的處理就比較複雜，比起剛剛的`<isindex>`，改動的檔案多了很多，看起來是把相關的東西全部都拿掉了。

最後來看這一個：[bgsound must use the HTMLUnknownElement interface](https://github.com/chromium/chromium/commit/98bc944d07152ab42d41eca79de79c207f7f0f29)

```
As specified here:
https://html.spec.whatwg.org/#bgsound
  
This causes one less fail on:
http://w3c-test.org/html/semantics/interfaces.html
```

裡面給的測試連結：[Test of interfaces](http://w3c-test.org/html/semantics/interfaces.html) 滿有趣的，會去測試一大堆元素的 interface 是不是正確的，在 [interfaces.js](http://w3c-test.org/html/semantics/interfaces.js) 裡面可以看到它測試的列表：

``` js
var elements = [
  ["a", "Anchor"],
  ["abbr", ""],
  ["acronym", ""],
  ["address", ""],
  ["applet", "Unknown"],
  ["area", "Area"],
  ["article", ""],
  ["aside", ""],
  ["audio", "Audio"],
  ["b", ""],
  ["base", "Base"],
  ["basefont", ""],
  ["bdi", ""],
  ["bdo", ""],
  ["bgsound", "Unknown"],
  ["big", ""],
  ["blink", "Unknown"],
  ["blockquote", "Quote"],
  ["body", "Body"],
  ["br", "BR"],
  ["button", "Button"],
  ["canvas", "Canvas"],
  ["caption", "TableCaption"],
  ["center", ""],
  ["cite", ""],
  ["code", ""],
  ["col", "TableCol"],
  ["colgroup", "TableCol"],
  ["command", "Unknown"],
  ["data", "Data"],
  ["datalist", "DataList"],
  ["dd", ""],
  ["del", "Mod"],
  ["details", "Details"],
  ["dfn", ""],
  ["dialog", "Dialog"],
  ["dir", "Directory"],
  ["directory", "Unknown"],
  ["div", "Div"],
  ["dl", "DList"],
  ["dt", ""],
  ["em", ""],
  ["embed", "Embed"],
  ["fieldset", "FieldSet"],
  ["figcaption", ""],
  ["figure", ""],
  ["font", "Font"],
  ["foo-BAR", "Unknown"], // not a valid custom element name
  ["foo-bar", ""], // valid custom element name
  ["foo", "Unknown"],
  ["footer", ""],
  ["form", "Form"],
  ["frame", "Frame"],
  ["frameset", "FrameSet"],
  ["h1", "Heading"],
  ["h2", "Heading"],
  ["h3", "Heading"],
  ["h4", "Heading"],
  ["h5", "Heading"],
  ["h6", "Heading"],
  ["head", "Head"],
  ["header", ""],
  ["hgroup", ""],
  ["hr", "HR"],
  ["html", "Html"],
  ["i", ""],
  ["iframe", "IFrame"],
  ["image", "Unknown"],
  ["img", "Image"],
  ["input", "Input"],
  ["ins", "Mod"],
  ["isindex", "Unknown"],
  ["kbd", ""],
  ["keygen", "Unknown"],
  ["label", "Label"],
  ["legend", "Legend"],
  ["li", "LI"],
  ["link", "Link"],
  ["listing", "Pre"],
  ["main", ""],
  ["map", "Map"],
  ["mark", ""],
  ["marquee", "Marquee"],
  ["menu", "Menu"],
  ["meta", "Meta"],
  ["meter", "Meter"],
  ["mod", "Unknown"],
  ["multicol", "Unknown"],
  ["nav", ""],
  ["nextid", "Unknown"],
  ["nobr", ""],
  ["noembed", ""],
  ["noframes", ""],
  ["noscript", ""],
  ["object", "Object"],
  ["ol", "OList"],
  ["optgroup", "OptGroup"],
  ["option", "Option"],
  ["output", "Output"],
  ["p", "Paragraph"],
  ["param", "Param"],
  ["picture", "Picture"],
  ["plaintext", ""],
  ["pre", "Pre"],
  ["progress", "Progress"],
  ["q", "Quote"],
  ["quasit", "Unknown"],
  ["rb", ""],
  ["rp", ""],
  ["rt", ""],
  ["rtc", ""],
  ["ruby", ""],
  ["s", ""],
  ["samp", ""],
  ["script", "Script"],
  ["section", ""],
  ["select", "Select"],
  ["slot", "Slot"],
  ["small", ""],
  ["source", "Source"],
  ["spacer", "Unknown"],
  ["span", "Span"],
  ["strike", ""],
  ["strong", ""],
  ["style", "Style"],
  ["sub", ""],
  ["summary", ""],
  ["sup", ""],
  ["table", "Table"],
  ["tbody", "TableSection"],
  ["td", "TableCell"],
  ["textarea", "TextArea"],
  ["tfoot", "TableSection"],
  ["th", "TableCell"],
  ["thead", "TableSection"],
  ["time", "Time"],
  ["title", "Title"],
  ["tr", "TableRow"],
  ["track", "Track"],
  ["tt", ""],
  ["u", ""],
  ["ul", "UList"],
  ["var", ""],
  ["video", "Video"],
  ["wbr", ""],
  ["xmp", "Pre"],
  ["\u00E5-bar", "Unknown"], // not a valid custom element name
];
```

像是 applet、bgsound、blink 等等這些元素，就應該回傳 [HTMLUnknownElement](https://developer.mozilla.org/zh-TW/docs/Web/API/HTMLUnknownElement)。

## 總結

這一趟旅程一樣收穫滿滿，從一個議題持續向外延伸，就能挖到更多有趣的東西。

例如說我們從 SmooshGate 事件，學到了 TC39 的運作流程、flatten 壞掉的原因以及 V8 當初實作 flatten 的 commit 還有學到怎麼寫測試。也學習到了 don't break the web 的原則，再從這個原則去看了 HTML 的規格，看到了那些被淘汰的 tag，最後去看了在 chromium 裡面怎麼做處理。

制定規格的人要注重的層面以及要考慮的問題真的很多，因為一旦做下去，就很難再回頭了；規格書也要寫得清楚又明白，而且不能有錯誤。

真心佩服那些制定標準的人。

參考資料：

1. [You Don't Know JS Yet: Get Started - 2nd Edition Chapter 1: What Is JavaScript?](https://github.com/getify/You-Dont-Know-JS/blob/2nd-ed/get-started/ch1.md)
2. [SmooshGate](https://blog.othree.net/log/2018/05/28/smooshgate/)
3. [#SmooshGate FAQ](https://developers.google.com/web/updates/2018/03/smooshgate)
4. [Non-conforming features](https://html.spec.whatwg.org/multipage/obsolete.html#non-conforming-features)
5. [3.2.2 Elements in the DOM](https://html.spec.whatwg.org/multipage/dom.html#elements-in-the-dom)

關於作者： 
[@huli](https://blog.huli.tw) 野生工程師，相信分享與交流能讓世界變得更美好