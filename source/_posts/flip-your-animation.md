---
title: FLIP 技巧總複習
date: 2018-10-20 17:46:57
tags:
  - css
  - javascript
  - flip
  - animation
author: arvinh
---

# 前言

在前端的世界中，如何優化 performance 一直都是很重要的議題，也常常被拿來當作評斷前端工程師能力的一個依據。效能調教有許多方面可以探討，但最主要的重點就是希望使用者在操作網站時，不會感受到頁面跳動，尤其是當網站有動畫效果時，些微的跳動就會引起使用者注意。

大概在去年的時候，同事為了解決 CSS Animation 在頁面上的不順暢，研究了不少 hack 技巧，其中一個讓我覺得很特別的就是今天的主角：FLIP。

FLIP 技巧已經出來好幾年了，但我也是去年才知道，今天這篇算是個複習與回顧，順便分享給還不知道此技巧的讀者。

## 什麼是 FLIP？為什麼我們需要它？

在開始介紹 FLIP 之前，必須先說明一下我們在製作 CSS 動畫時，可能會遭遇的效能問題。

舉例來說，今天我們想要製造一個如下效果的動畫：

![Bad animation](/img/arvinh/flip-bad-sample.gif)
[code link](https://codepen.io/arvin0731/pen/wYXJdv)

我們通常會需要調整 DOM 元素的位置與大小，而這類的操作會造成瀏覽器偵測到潛在的 Layout 改變，而重新從 pixel pipeline 的 Layout 階段開始 reflow 與 repaint，若同時在這階段進行其他 js 操作或是頁面互動，就會降低 fps，也就代表著頁面可能會掉幀，造成視覺上的跳動。

當然在目前硬體設備越來越強大的狀態下，其實像上圖中的動畫很難會有問題，但若是背後有其他繁重的 js 在運作，還是有可能出現下面這樣的 fps 狀態：

![Jank FPS](/img/arvinh/flip-bad-jank.png)

解決方法通常就是要想辦法將這類花費昂貴的動畫改用 `transform`、`opacity` 來製作，讓動畫的進行能在 Composite 階段進行即可，不須動到 Layout。

這時候就可以引入 FLIP 技巧來幫忙了！透過 FLIP 可以將那些會導致 Layout 更動的動畫，轉成以 `transform` 這樣的屬性來達成。

## FLIP

如果你直接 google 搜尋 FLIP，可能會查到滑板教學；搜尋 FLIP CSS，應該會出現卡片翻轉的動畫教學。

那到底 FLIP 跟網頁動畫的優化有什麼關係呢？

FLIP 其實是四個字的組合：**F**irst, **L**ast, **I**nvert, **P**lay：

* **F**irst：在 FLIP 技巧中，我們需要先記錄下動畫元件的初始狀態。

* **L**ast：接著進行一些運算後，套用動畫的最終狀態在動畫元件上，並且將完成動畫後的狀態記錄下來。

* **I**nvert：FLIP 最主要的 hack 就是發生在這個階段。根據前兩個步驟，我們可以知道該動畫物件在動畫期間的位置變化，接著利用 `transform` 與 `scale`，將物件從動畫結尾位置移動回初始狀態的地點。

* **P**lay：在最後的步驟時，元件已經被我們 `transform` 回起始點了，這時只要將 `transform` 屬性移除，並加上 `transition` 的效果，我們就能完美的消除原先昂貴的 Layout change，改以能擁有獨自 Layer 的 `transform` 來處理動畫效果。

文字敘述可能不夠好懂，[David Khourshid](https://twitter.com/davidkpiano) 做的這個範例應該就能非常清楚的說明 FLIP 原理：

<p data-height="265" data-theme-id="dark" data-slug-hash="EbwrQQ" data-default-tab="result" data-user="davidkpiano" data-pen-title="How the FLIP technique works" class="codepen">See the Pen <a href="https://codepen.io/davidkpiano/pen/EbwrQQ/">How the FLIP technique works</a> by David Khourshid (<a href="https://codepen.io/davidkpiano">@davidkpiano</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

解釋起來其實會發現跟 FLIP 這個字其實也是很吻合的，把元件從結束狀態 ”翻“ 回初始狀態。

看到這邊可能會有些人覺得困惑，為什麼要這麼麻煩，不能直接用 `transform` 搭配 `transition` 將元件移動至結束位置嗎？

的確是可以，但如果你今天是要優化原先寫好的動畫，變成要重新手動計算 `transform` 的位置與 `scale` 的比例，是件麻煩的事，倒不如讓瀏覽器幫你處理這部分的計算，只要算出起點與終點位置的差異即可。

此外，將元件先移至結束位置，再 `transform` 回去起點的方式，可以讓瀏覽器先知道這個動畫會走的過程，據說有助於更快的處理動畫。（但我沒有實際測試過，並非百分百確定有這種效果，也歡迎大家幫忙補充！）

## 接著我們實際做做看

依照 FLIP 原則，我們將一開始的動畫做點調整：

* **F**irst：

```js
// Fisrt: 記錄初始狀態
const collapsed = elem.getBoundingClientRect();
```

* **L**ast：

```js
// 進行運算, 套用動畫結尾的 css style
elem.classList.add('expanded');
// Last: 紀錄結束狀態
const expanded = elem.getBoundingClientRect();
```

* **I**nvert：

```js
// INVERT
// 根據在 F 與 L 紀錄的位置，我們算出其位置差距 (top 與 left)
const invertedTop = collapsed.top - expanded.top;
const invertedLeft = collapsed.left - expanded.left;
// 大小的部分則是用寬高差去計算 scale 的比例
const invertedWidth = collapsed.width / expanded.width;
const invertedHeight = collapsed.height / expanded.height;
// 設定 transform origin，代表動畫要從哪裡開始
elem.style.transformOrigin = 'top left';
// 將算好的差距 apply 到 transform 屬性上
// 這樣就能將元件 ”翻“ 回初始位置
elem.style.transform = 'translate(' + invertedLeft + 'px, ' + invertedTop + 'px) scale(' + invertedWidth + ', ' + invertedHeight + ')';
```

* **P**lay：

```js
// PLAY
// 利用 Raf，在下個 frame 開始時再做動畫
requestAnimationFrame(function(){
  // 設定 transition，並移除 transform，讓他再 "翻" 到結尾位置
  elem.style.transition = '550ms ease-out';
  elem.style.transform = '';
});
elem.addEventListener('transitionend', function() {
  elem.style.transition = '';
});
```

經由 FLIP 技巧調整過後的動畫：

<p data-height="713" data-theme-id="dark" data-slug-hash="EdRwmr" data-default-tab="result" data-user="arvin0731" data-pen-title="FLIP - good" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/EdRwmr/">FLIP - good</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

我一開始看到這個實作時，稍微有點困惑，覺得為什麼可以直接先加上結尾狀態的 css，而不會讓元件真正更動？

原因很簡單，就在於這些動作是在瀏覽器繪製下一個 frame 之前就做完的，在瀏覽器真正過了 16ms，要開始繪圖時，我們已經 apply 了另一個繪圖需求，也就是 `transform`。所以並不會看到元間跑到結束位置的狀況。

從下面這張圖應該會更好理解：

![FPS](/img/arvinh/flip-web-fps.png)
[圖片來源](https://blog.camel2243.com/2017/01/31/javascript-requestanimationframe-%E5%84%AA%E5%8C%96%E5%8B%95%E7%95%AB%E6%95%88%E7%8E%87%E8%88%87%E8%B3%87%E6%BA%90/)

上排黑色箭頭代表每一次的 frame render，下排則是繪圖需求，而紅色可以想做是我們計算結尾位置的動作。可以明顯看到，在每次紅色箭頭的後面，真正開始繪圖前，都有新的繪圖需求會蓋過去，因此瀏覽器並不會渲染紅色箭頭的部分。

另外這邊要注意兩件事，第一個是 F、L、I 的時間需要掌控在 **100ms** 之內，根據 [Paul Lewis 的 FLIP 介紹文章](https://aerotwist.com/blog/flip-your-animations/)，使用者在做了互動後，到感知動畫的發生，這中間可以有 `100ms` 的空隙，我們只要能在這 100ms 中將初始與結尾位置計算完，並翻回初始點，使用者就不會感受到任何差異。

![user perception](/img/arvinh/flip-user-perception.jpg)
[圖片來源](https://aerotwist.com/blog/flip-your-animations/)

最後，在這個例子中，我需要利用 `transitionend` 這個事件，在動畫的最後把 `transition` 拿掉，否則下一次要計算動畫終點位置時 `elem.classList.add('expanded');`，就會產生 `transition` 效果，那不是我們要的。

## 結論

FLIP 技巧的概念很簡單，但實作起來的確會增加不少程式碼，但好在有不少套件可以使用，也能搭配 Web Animations API ([polyfill](https://github.com/web-animations/web-animations-js)) 來開發。比較推薦的是 [David Khourshid 的 flipping.js](https://github.com/davidkpiano/flipping)，他在去年也有寫了一篇文章來介紹 [Animating Layouts with the FLIP Technique](https://css-tricks.com/animating-layouts-with-the-flip-technique/)，其中還有講到如何將 FLIP 應用在兩個獨立的元件上，寫得很棒很清楚，推薦大家直接去閱讀！

## 資料來源

1. [Animating Layouts with the FLIP Technique](https://css-tricks.com/animating-layouts-with-the-flip-technique/)
2. [[javascript] requestAnimationFrame 優化動畫效率與資源](https://blog.camel2243.com/2017/01/31/javascript-requestanimationframe-%E5%84%AA%E5%8C%96%E5%8B%95%E7%95%AB%E6%95%88%E7%8E%87%E8%88%87%E8%B3%87%E6%BA%90/)
3. [FLIP Your 60 FPS Animations, FLIP ’Em Good](https://medium.com/outsystems-experts/flip-your-60-fps-animations-flip-em-good-372281598865)
4. [FLIP Your Animations](https://aerotwist.com/blog/flip-your-animations/)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化