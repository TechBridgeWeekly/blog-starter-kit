---
title: 用 CSS Grid 創造蒙德里安藝術
date: 2018-11-15 00:18:37
tags:
  - css
  - grid
  - mondrian
  - react
author: arvinh
---

# 前言

前陣子 netflix 上了最新一季的夜魔俠，其中的反派角色很愛在家中擺設畫作，有了藝術品襯托，壞人在我的腦海裡突然就變成看似很有深度的角色。這讓我覺得應該也該擺點畫作在家裡，看看能不能提高自己的層次。

而宅宅如我當然無法做如此的投資，不過如果能夠自己用 Web 技術產生一些藝術作品，然後投影在家中呢？應該很酷吧！
然後就在 codepen 上發現了一個有趣的東西：
<p data-height="416" data-theme-id="29194" data-slug-hash="XZqwaq" data-default-tab="result" data-user="jh3y" data-pen-title="Randomly generate Mondrian Art with CSS Grid + Vue 🎨" class="codepen">See the Pen <a href="https://codepen.io/jh3y/pen/XZqwaq/">Randomly generate Mondrian Art with CSS Grid + Vue 🎨</a> by Jhey (<a href="https://codepen.io/jh3y">@jh3y</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

看起來煞有其事啊！原來簡單利用 CSS Grid 就能排出這樣的藝術圖畫！ 查了一下這叫做蒙德里安 (mondrian art)。

臨摹也算是一種學習，原作者是使用 Vue.js 與 Stylus 製作，今天我們就簡單利用 React 來重新實作一次並說明原理吧！

## CSS Grid 小複習

如果有不了解 CSS Grid 的讀者，可以先去看我先前寫的介紹 - [與 CSS Grid 的第一次接觸](https://blog.techbridge.cc/2017/02/03/css-grid-intro/) 或是很完整的 [A Complete Guide to Grid](https://css-tricks.com/snippets/css/complete-guide-grid/)。

這邊簡單總結一下 CSS Grid 的特性：

相比於 `Flexbox` 是 `one-dimensional layout solution`，`CSS Grid` 可以理解為 `two-dimentional layout solution`。

也就是說，你能夠在二維平面上，透過網格系統來擺放你的元件位置。

現在最新版的 Chrome 與 Firefox 都已經支援 CSS Grid Layout 了，並且在 devtool 中能清楚看到網格狀態：

![css grid in devtools](/img/arvinh/css-grid-in-devtools.png)

## 實作前的小分析

不管你對 CSS Grid 有沒有概念，看了上面的圖應該也很快可以猜出其原理。

主要就是讓整個 `container` 切割成數個小方塊，接著隨機產生出多個區塊，每個區塊隨機橫跨不同的行數與列數，並填上不同顏色，將 `container` 填滿即可。

有點像是七巧板的感覺，只是我們的板子都是矩形~ 不過聽起來簡單，實作上還是有些細節需要注意的。

# 開始實作！

第一步是先定義出我們這幅畫作的畫框，也就是 CSS Grid 中的 `container`，透過這個 `container` 來定義好整個網格系統：

```css
.MondrainArt {
  background-color: "#070908",
  border: "10px solid #070908",
  display: "grid",
  grid-gap: "10px",
  grid-template-columns: "repeat(auto-fit, 50px)",
  grid-template-rows: "repeat(auto-fit, 50px)",
  height: "300px",
  overflow: "hidden",
  width: "250px"
}
```

這裡面發生了幾件事情：

我們用 `grid-template-columns` 與 `grid-template-rows` 先規範我們的 container 內要有多少欄與列，以及每格的長寬。

由於每個格子的大小我們要固定，所以使用 `repeat()`，並且採用 `<auto-repeat> values` 的語法，這樣做的好處是我們只要固定好 `container` 的寬高，不用去限制 grid 的 column 數與 row 數，讓 CSS Grid 幫我們計算出不會 overflow 的數目。[MDN: repeat()](https://developer.mozilla.org/en-US/docs/Web/CSS/repeat)

至於要用 `auto-fit` 或是 `auto-fill`， 在這邊的例子中其實都可以，因為這兩者的差別主要在於寬度改變時，針對多出的空間在運用上有不同的方式：

以 `auto-fit` 來說，視覺上的效果是會將原本 column 內的 item 寬度都平均拉長，但實際上還是有新增 column 數，只是因為 content 是空的，所以空間被壓縮了：

![auto-fit](/img/arvinh/auto-fit.gif)
[source](https://css-tricks.com/auto-sizing-columns-css-grid-auto-fill-vs-auto-fit/)

而 `auto-fill` 則是不管是否有新的 content 在，新增的 column 數都會佔有一樣的空間大小，並不會去延伸原有 item 的寬度：

![auto-fill](/img/arvinh/auto-fill.gif)
[source](https://css-tricks.com/auto-sizing-columns-css-grid-auto-fill-vs-auto-fit/)

接著，利用 `gap: 10px 10px` 設定好每個格子間的空隙；然後給定一個固定的寬高並將 overflow hidden 起來。

到這邊為止，我們可以先在 `container` 內放入數個 grid item，然後每個 item 的 `grid-column` 與 `grid-row` 皆設置為 `span 1`，也就是都只占 grid 的中的一個單位。如此一來就能看到目前的格子樣式：

<iframe src="https://codesandbox.io/embed/ww21ow2nvl" style="width:100%; height:500px; border:0; border-radius: 4px; overflow:hidden;" sandbox="allow-modals allow-forms allow-popups allow-scripts allow-same-origin"></iframe>

## 隨機產生不同大小的區塊

有了基礎的格子後，看起來我們接著只要把目前 grid item 的 `grid-column` 與 `grid-row` 改造一下就大功告成了吧!?

馬上來試試！

首先，先做點數學，我們剛剛設定每個格子是 `50px`，而每個格子的間距 `gap` 為 `10px`，而寬度為 `250px`，所以每一個 `row` 最多就是 `4` 個格子。（用肉眼看上圖其實就知道了...）

為什麼要算這個呢？

因為我們要讓 grid item 的 `grid-column` 與 `grid-row` 隨機分配，但要在正確的範圍內，更改上面範例內的 `generateBlocks()` 如下，並加上個 `colorMap` 來隨機分配顏色：

```js
colorMap = {
  0: "rgb(248, 217, 45)",
  1: "rgb(248, 217, 45)",
  2: "rgb(242, 245, 241)",
  3: "rgb(11, 84, 164)",
  4: "rgb(214, 0, 20)",
  5: "rgb(11, 84, 164)"
};
generateBlocks = () => {
  return Array.from(Array(20).keys()).map(i => (
    <div
      key={`blocks-${i}`}
      style={{
        gridColumn: `span ${Math.floor(Math.random() * 3 + 1)}`,
        gridRow: `span ${Math.floor(Math.random() * 3 + 1)}`,
        backgroundColor: this.colorMap[Math.floor(Math.random() * 5)]
      }}
    />
  ));
};
```

可以得到以下結果：

<iframe src="https://codesandbox.io/embed/934p8px7pw" style="width:100%; height:500px; border:0; border-radius: 4px; overflow:hidden;" sandbox="allow-modals allow-forms allow-popups allow-scripts allow-same-origin"></iframe>

疑？好像跟想像不太一樣？

的確是隨機產生了橫跨不同欄位數的區塊，但是大小好像不太對啊！

那是因為我們沒辦法保證隨機產生的 grid-area 都能剛剛好接續著各自的起始點排滿滿的，所以可能發生下列情況：

![auto-col-row](/img/arvinh/auto-col-row.png)

在右下角實際產生的黃色區塊前，`row5` 與 `row6` 的位置實際上是空的，所以 grid layout 會將其當作為高度 0 的 item。

好在，我們可以利用 `grid-auto-columns` 與 `grid-auto-rows` 來解決！[css-tricks](https://css-tricks.com/snippets/css/complete-guide-grid/#article-header-id-24)

我們在 `container` 加上以下設定：

```css
grid-auto-columns: "50px",
grid-auto-rows: "50px",
```

這樣就會讓 grid layout 知道空的欄位我們想要自動填補上 `50px` 的大小。

基本上到此為止就完成了一個不錯的版本：

![no-fill-algo](/img/arvinh/no-fill-algo.png)

但總還是覺得怪怪的，好像“不夠密”。

有些區塊如果能換個位置似乎會好看一點？

由於我們並沒有明確指定每個區塊在 grid 中的確切位置，grid layout 會採用一個 `auto-placement algorithm` 來擺放，而我們能透過 `grid-auto-flow` 這個屬性來更改其排放的規則。[css-tricks](https://css-tricks.com/snippets/css/complete-guide-grid/#article-header-id-25)

`grid-auto-flow` 有 `row`, `column` 與 `dense` 這三種值可以設置，從字面上來看就能理解各自代表的意思，如果你設定為 `row` 或 `column`，那 grid layout 會盡量幫你的 item 照著列與行的方向照順序排放；而 `dense` 則是會盡量把空間塞滿，小的 item 就可能會先排在大的 item 之前，跟你在 html dom 上的排放位置可能有所落差，對於 accessibility 並不好，但以我們的 case 來說，不需要考慮那些，就大膽採用 `dense` 就對了！

## 最終成果

最後我們可以再加上一點點動畫效果，讓 react component 每五秒重新 render 一次新的圖案，這樣投放出來以後，就好像你有一幅不斷自行變化的藝術品一般了！

<iframe src="https://codesandbox.io/embed/420rv1v86w" style="width:100%; height:500px; border:0; border-radius: 4px; overflow:hidden;" sandbox="allow-modals allow-forms allow-popups allow-scripts allow-same-origin"></iframe>

# 結論

有趣的作品實作起來可能原理很簡單，但也是有一些眉眉角角需要注意。透過這次的實作，重新複習了一次 CSS Grid 的好處與用法，也期許自己未來能夠有更多的時間去思考與創造這樣的作品！

## 資料來源

1. [A Complete Guide to Grid](https://css-tricks.com/snippets/css/complete-guide-grid)
2. [Randomly generate Mondrian Art with CSS Grid + Vue 🎨](https://codepen.io/jh3y/pen/XZqwaq/)
3. [Grid by Example](http://gridbyexample.com/)
4. [css-grid-polyfill](https://github.com/FremyCompany/css-grid-polyfill/)

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化