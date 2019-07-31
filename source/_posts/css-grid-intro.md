---
title: 與 CSS Grid 的第一次接觸
date: 2017-02-03 23:44:37
tags:
  - CSS
  - WEB
  - CSS-GRID
---

## 前言

建構網頁佈局是製作網站的基本動作之一，隨著設計多樣性與功能複雜度的提升，開發者必須利用 `table`, `inline`, `float`, `clear`, 或是 `positioning` 等方式來 hack 頁面配置，但不管是哪種方式都各自有其缺乏的功能，也因此催生出許多 CSS Framework 提供的 Grid System 來加速佈局開發。

幾年前出現的 `FlexBox` 宛如神兵利器，讓我們可以擺脫這些困擾，快速進行排版，而隨著 `CSS Grid` 規範的出現，我們即將有更好的武器了！

`CSS Grid` 不是要來取代 `Flexbox`，他們是相輔相成的角色，`flex` 可以看作是 `one-dimensional layout solution`，`CSS Grid` 則是 `two-dimentional layout solution`，用來解決我們所有先前需要各種 hack 才能完成的頁面佈局。

不過可惜的是，`CSS Grid` 還在 [W3C working draft](https://www.w3.org/TR/css-grid-1/) 中，並且直到 2017 的今天，還是有些人使用不支援 `flex` 的舊瀏覽器...

<img src="/img/arvinh/caniuse-grid.png" alt="Can I use Css grid" style="width: 500px;"/>

從[上圖](http://caniuse.com/#search=grid)可以得知，Chrome 跟 Firefox 的下一個版本都會預設支援 `CSS Grid`，而 IE/Edge 目前則是支援舊版本的規格實作的。如果想要先嚐鮮，可以直接下載 [Chrome Canary](https://www.google.com/chrome/browser/canary.html), [Firefox Nightly](https://www.mozilla.org/en-US/firefox/channel/desktop/) 來使用，或是開啟目前版本的實驗功能：

* Chrome: `chrome://flags` -> 啟用 `experimental Web Platform features`
* Firefox: `about:config` -> 啟用 `layout.css.grid.enabled`

當然也有 [Polyfill](https://github.com/FremyCompany/css-grid-polyfill/) 可以使用。

雖然生不逢時，但這一切都是過程，活著不難，最難的是做人...啊啊蛋堡的歌真好聽

這篇文章主要是介紹 `CSS Grid` 的用法，並為我自己做個學習紀錄，畢竟網路上已經有許多關於 `CSS Grid` 的資源，我也條列了一些在文章最下方，懶得看我廢言的讀者可以自行參考！

## CSS Grid Layout

以往我們在進行頁面切版佈局的時候，即便是使用 `flexbox`，由於他是 `one-dimensional layout`，我們排版的方式無非就是從上往下排，或是由左到右等方向性來把各種 `one-dimensional layout` 組合成我們要的 `two-dimensional layout`。

而 `CSS Grid` 不同的地方就在於，他讓我們先定義好一個 `container` 就夠了，一個 `container` 來設定好所有底下元件**可以擺放的位置**。

什麼是**“可以擺放的位置“**呢？先跳過實作過程與這些屬性值的意思，以下面這張結果圖來解釋一下，你設定好 `CSS Grid` 的 container 後，該 container 會長成這副德性：

```css
.container {
  grid-template-columns: 100px 10px 0.3fr 0.7fr;
  grid-template-rows: 25% 100px auto;
}
```

<img src="/img/arvinh/grid-container.png" alt="Grid Container" style="width: 300px;"/>

然後你就只要把 `container` 內的元素，一一定義好各自的 CSS 屬性，來決定他們要擺在這圖中的哪個空格內即可。
只要兩行 css，一個 div 就排好了，是不是很讓人興奮啊啊啊！

好的，接下來認真說明 CSS Grid 的其他基本用法。

## CSS Grid Terminology

再開始嘗試實作之前，我們先來了解並記憶一下相關的術語。

* Grid Line: 分隔元素的線，可以是垂直與水平，如下圖的紅色線

* Grid Track: 兩條分隔線中間的區域，簡單想就是 Grid 中的 Columns 或 Rows，如下圖的綠色區塊

* Grid Cell: Grid 中的基本單位，四條線組成的區域，如下圖的藍色區塊。

* Grid Area: 由數個 Cell 組成的區域，如下圖的紅色區塊。

* fr: track-size 的單位，通常用於分配 row 或 column 的非彈性尺寸設定完後之剩餘空間。以下圖的 column為例，意思即：將去掉 100px 與 10px 後的剩餘空間，分配為 30% 與 70%。

<img src="/img/arvinh/grid-term.png" alt="Grid Terminology" style="width: 300px;"/>

## Simple CSS Grid Example

接著我們把上面的圖片，用 CSS Grid 來 layout 出來。

CSS Grid 的 properties 可以切成兩大塊，一塊是給 Parent 的，也就是 `Grid Container`，另一塊給 Children，`Grid items`。

詳細的 properties 介紹可以看這篇 [Complete Guide Grid](https://css-tricks.com/snippets/css/complete-guide-grid/)
（ 強力推薦閱讀，各種圖片文字輔以 css 說明，完全可以直接左轉出去看這篇就好 XD ）

要使用 CSS Grid 就必須先定義好 Grid Container：

```html
<div class="grid-container">
  <div class="grid-item item-a">Tack</div>
  <div class="grid-item item-b">Cell</div>
  <div class="grid-item item-c">Area</div>
</div>

```
在 grid container 底下的 children 是沒有順序差別的，我們都可以透過定義 grid items 的 properties 來設定他的擺放位置。當然，如果你沒有設定的話，就會照順序放置，如果該列放不下就會自動放到下一行。

但基本上不會這麼做，畢竟我們就是想要能好好掌控位置啊！

Grid Container 的 css 可以用下面兩種方式來寫，`grid-template-columns` 用來定義這個 Grid 有幾個 columns，每個 column 可以有多少寬度；換而言之，`grid-template-rows` 就是用來定義 row 的屬性。

`grid-template-columns` 和 `grid-template-rows` 有兩個主要 Values 可以設定：

* track-size：可以是任何長度(px)、百分比(%)或是先前提到的 `fr`
* line-name：組成 column 與 row 的線的名字

```css
.container {
  grid-template-columns: <track-size> ... | <line-name> <track-size> ...;
  grid-template-rows: <track-size> ... | <line-name> <track-size> ...;
}
```

也就是說，你可以用純數字的方式來給予屬性質：

```css
.grid-container {
  height: 100vh;
  display: grid;
  grid-template-columns: 100px 10px 0.3fr 0.7fr;
  grid-template-rows: 25% 100px auto;
}
```

也可以安插 `Grid Line` 的名字在屬性值的設定中，這個名字在後續我們要安置 children 時可以用到！
[Note] 要注意，因為是定義 **線** 的名字，所以最後一個數字的後面也可以定義名字喔！

```css
.grid-container {
  height: 100vh;
  display: grid;
  grid-template-columns: [line1] 100px [line2] 10px [line3] 0.3fr [line4] 0.7fr [end];
  grid-template-rows: [row-1-start] 25% [row-2-start] 100px [row-3-start] auto [row-end];
}
```

這樣我們就定義好需要的 Grid Container 了。

接著，我們說明一下如何將前面的圖片中的 ** Area ** 這個 `Grid Item` 放到我們剛剛定義好的 Grid Container 中，因為他的位置比較特殊，在右下角的位置，較好展現 CSS Grid 方便之處。

`Grid Item` 的屬性基本上就是四個：`grid-column-start`, `grid-column-end`, `grid-row-start` 和 `grid-row-end`，你也可以簡化成兩個：`grid-column` 與 `grid-row`。

`grid-column-start`, `grid-column-end`, `grid-row-start` 和 `grid-row-end` 可以有四種 Values 設定：

* line： 對照到 Grid Container 中定義的線，可以是數字或名字。
* span [number]：該 item 會橫跨 [number] 個 tracks。
* span [name]：該 item 會橫跨到名字是 [name] 的線為止。
* auto：自動放置，類等於 span 1。

```css
.item-area {
  background-color: #C57474;
  grid-column-start: line2;
  grid-column-end: end;
  grid-row-start: row-3-start;
  grid-row-end: row-end;	
}
```

若只用兩個屬性，中間以 / 隔開：

```css
.item-area {
  background-color: #C57474;
  grid-column: line2 / end;
  grid-row: row-3-start / row-end;
}
```

以此類推，完成的範例如下：

<p data-height="300" data-theme-id="0" data-slug-hash="ZLodmz" data-default-tab="css,result" data-user="arvin0731" data-embed-version="2" data-pen-title="CSS-Grid-Sample" class="codepen">See the Pen <a href="http://codepen.io/arvin0731/pen/ZLodmz/">CSS-Grid-Sample</a> by Arvin (<a href="http://codepen.io/arvin0731">@arvin0731</a>) on <a href="http://codepen.io">CodePen</a>.</p>
<script async src="https://production-assets.codepen.io/assets/embed/ei.js"></script>


## Another CSS Grid Example

Container 不只有上面的寫法，還有一個叫做 `grid-template-areas` 的東西，搭配 Grid-Item 的 `grid-area` 可以有更簡單的寫法來進行排版。

我們這邊換個例子，以一般網頁較常見的配置來說明：

<p data-height="423" data-theme-id="0" data-slug-hash="xgjvRr" data-default-tab="result" data-user="arvin0731" data-embed-version="2" data-pen-title="CSS-Grid-Sample-Area" class="codepen">See the Pen <a href="http://codepen.io/arvin0731/pen/xgjvRr/">CSS-Grid-Sample-Area</a> by Arvin (<a href="http://codepen.io/arvin0731">@arvin0731</a>) on <a href="http://codepen.io">CodePen</a>.</p>
<script async src="https://production-assets.codepen.io/assets/embed/ei.js"></script>

用了 `grid-template-areas` 後，在 container 的 css 中，我們可以直接定義每個元素的位置：

```css
.grid-container {
  height: 100vh;
  display: grid;
  grid-template-columns: 0.25fr 0.25fr 0.25fr 0.25fr;
  grid-template-rows: auto;
  grid-template-areas: "header header header header"
                       "main main . sidebar"
                       "footer footer footer footer";
  grid-column-gap: 10px;
  grid-row-gap: 15px;
}
```

裡面的 `header`, `main`, `sidebar` 與 `footer` 就是在 Grid-Item 中我們要指定的名稱：

`grid-template-areas` 的每一行定義了 Grid-Item 內容：

* 第一行代表：header 要佔據四行。
* 第二行代表：main 佔兩行，而 sidebar 佔一行，其中 `·` 類似 placeholder 的作用。
* 第三行代表：footer 佔四行。

```css
.header {
  background-color: #8BC574;
  grid-area: header;
}

.main {
  background-color: #748CC5;
  grid-area: main;
}

.sidebar {
  background-color: #C57474;
  grid-area: sidebar;
}

.footer {
  background-color: #FCE052;
  grid-area: footer;
}
```
此外，`grid-column-gap` 與 `grid-row-gap` 可用來設定行距。

## One more thing

除了上述的基本屬性運用外，最前面有說過 CSS Grid 跟 Flex 是相輔相成，而實際上他們可以調配的屬性也有類似的地方。

Grid Container 可以用 `justify-items` 與 `align-items` 來控制每個 Grid-item 的 **垂直** 與 **水平** 位置。

Grid Item 則是可以用 `justify-self` 與 `align-self` 來控制自己在網格中的位置。

像這個範例中的 Sidebar

<p data-height="265" data-theme-id="0" data-slug-hash="NdMQXK" data-default-tab="css,result" data-user="arvin0731" data-embed-version="2" data-pen-title="CSS-Grid-Sample-align" class="codepen">See the Pen <a href="http://codepen.io/arvin0731/pen/NdMQXK/">CSS-Grid-Sample-align</a> by Arvin (<a href="http://codepen.io/arvin0731">@arvin0731</a>) on <a href="http://codepen.io">CodePen</a>.</p>
<script async src="https://production-assets.codepen.io/assets/embed/ei.js"></script>

歡迎大家參考這幾個個超詳細圖解說明：

[Complete guide grid - justify-items](https://css-tricks.com/snippets/css/complete-guide-grid/#prop-justify-items)
[Complete guide grid - align-items](https://css-tricks.com/snippets/css/complete-guide-grid/#prop-align-items)
[Complete guide grid - justify-self](https://css-tricks.com/snippets/css/complete-guide-grid/#prop-justify-self)
[Complete guide grid - align-self](https://css-tricks.com/snippets/css/complete-guide-grid/#prop-align-self)

## Conclusion

用幾個簡單的範例來學習並介紹 CSS Grid，但還是希望大家都能直接去 Codepen 上玩玩看 CSS Grid，一定都會被其方便性給驚豔到！
前端發展雖然百家爭鳴，但是很多基本的規範如果能越來越統一越來越強大，開發者就更能省去各種 hacking 的時間，用來創造更多有創意又有美感的作品！

最後再推薦一個網站 [Grid by Example](http://gridbyexample.com/)，裡面用許多範例來進行教學，頁面也很清楚乾淨。
有任何問題都歡迎大家指教！	

## 資料來源
1. [A Complete Guide to Grid](https://css-tricks.com/snippets/css/complete-guide-grid)
2. [Chris House Guide to Grid](http://chris.house/blog/a-complete-guide-css-grid-layout/)
3. [CSS Grid 介紹](http://andyyou.github.io/2016/05/04/css-grid/)
4. [Grid by Example](http://gridbyexample.com/)
5. [css-grid-polyfill](https://github.com/FremyCompany/css-grid-polyfill/)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化
