---
title: 十分鐘、五步驟，SVG 動起來！
date: 2019-11-06 08:56:40
tags:
  - web
  - svg
  - animation
  - TimelineMax
  - GSAP
---

# 前言

在 Codepen 上常常看到很多會設計又懂寫前端的高手，用 SVG 畫出很漂亮的圖案後，還能讓他們產生可愛的動畫，我一直很好奇他們是怎麼實作的，總覺得很困難，好像得對 SVG 透徹了解，並且自己畫出那些 SVG 圖案，才有辦法實作動畫。
但其實不然，今天就來分享一個簡單的小技巧，讓你在短短十分鐘內就能讓一個靜態的 SVG 圖案活躍起來！

簡單看個成品，你可以點選下面範例中的 `build` 按鈕，可以看到原本靜止的 SVG 房子動了起來！

<p class="codepen" data-height="639" data-theme-id="29194" data-default-tab="result" data-user="arvin0731" data-slug-hash="qBBRYjp" style="height: 639px; box-sizing: border-box; display: flex; align-items: center; justify-content: center; border: 2px solid; margin: 1em 0; padding: 1em;" data-pen-title="SVG Animation House">
  <span>See the Pen <a href="https://codepen.io/arvin0731/pen/qBBRYjp">
  SVG Animation House</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>)
  on <a href="https://codepen.io">CodePen</a>.</span>
</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

## 第一步，先找到你喜歡的 SVG 圖案

如果你跟我一樣，畫不出漂亮的圖案或 Icon，可以到 [Flaticon](https://www.flaticon.com/)、[Iconfinder](https://www.iconfinder.com/) 或 [icons8](https://icons8.com/) 等網站找些免費的 svg icon 下載使用（注意授權即可）。

![Find a good icon](/img/arvinh/flaticon-desktop.png)

前陣子到美國出差發現他們好像很流行滑板車（scooter），那就到 [Flaticon](https://www.flaticon.com/) 上找個類似的圖案來玩玩！（一開始的房屋範例也是從 [Flaticon](https://www.flaticon.com/) 取得的）：

<div style="margin: 0 auto; display: block; width: 256px;">
<svg id="Capa_1" enable-background="new 0 0 512.004 512.004" height="256" viewBox="0 0 512.004 512.004" width="256" xmlns="http://www.w3.org/2000/svg"><path d="m175.669 463.803c-8.283 0-15-6.716-15-15 0-53.743-43.723-97.467-97.467-97.467-14.622 0-28.673 3.153-41.762 9.371-7.483 3.555-16.432.371-19.986-7.112-3.555-7.482-.37-16.431 7.113-19.985 17.143-8.143 35.525-12.273 54.635-12.273 70.286 0 127.467 57.182 127.467 127.467 0 8.283-6.714 14.999-15 14.999z" fill="#c5e1e6"/><path d="m442.768 321.476c-63.027 2.945-113.414 51.086-120.563 112.327h-210.801c-8.285 0-15 6.716-15 15s6.715 15 15 15h224.932c8.285 0 15-6.716 15-15 0-52.162 40.777-94.928 92.832-97.36 8.275-.387 14.67-7.408 14.283-15.684-.387-8.275-7.402-14.684-15.683-14.283z" fill="#008adf"/><path d="m442.768 321.476c-63.027 2.945-113.414 51.086-120.563 112.327h-66.204v30h80.335c8.285 0 15-6.716 15-15 0-52.162 40.777-94.928 92.832-97.36 8.275-.387 14.67-7.408 14.283-15.684-.387-8.275-7.402-14.684-15.683-14.283z" fill="#0065a3"/><path d="m448.787 415.604c-7.721 0-14.279-5.923-14.932-13.755l-28.796-345.572c-1.291-15.484-11.852-26.275-20.521-26.275-8.283 0-15-6.716-15-15s6.717-15 15-15c12.9 0 25.295 5.971 34.9 16.811 8.852 9.99 14.361 23.12 15.518 36.972l28.797 345.573c.688 8.256-5.447 15.506-13.703 16.194-.425.035-.847.052-1.263.052z" fill="#8db9c4"/><circle cx="63.203" cy="448.803" fill="#c5e1e6" r="48.2"/><path d="m63.203 512.002c-34.848 0-63.199-28.351-63.199-63.199 0-34.849 28.352-63.199 63.199-63.199 34.85 0 63.201 28.35 63.201 63.199 0 34.848-28.352 63.199-63.201 63.199zm0-96.398c-18.306 0-33.199 14.893-33.199 33.199 0 18.307 14.894 33.199 33.199 33.199 18.307 0 33.201-14.893 33.201-33.199s-14.895-33.199-33.201-33.199z" fill="#1d4659"/><circle cx="448.803" cy="448.803" fill="#8db9c4" r="48.2"/><g fill="#0e232c"><path d="m448.803 512.002c-34.848 0-63.199-28.351-63.199-63.199 0-34.849 28.352-63.199 63.199-63.199 34.85 0 63.201 28.35 63.201 63.199 0 34.848-28.352 63.199-63.201 63.199zm0-96.398c-18.307 0-33.199 14.893-33.199 33.199 0 18.307 14.893 33.199 33.199 33.199 18.307 0 33.201-14.893 33.201-33.199s-14.895-33.199-33.201-33.199z"/><path d="m352.402.002c-8.283 0-15 6.716-15 15s6.717 15 15 15h32.135v-30h-32.135z"/></g></svg>
Icons made by <a href="https://www.flaticon.com/authors/freepik" title="Freepik">Freepik</a> from <a href="https://www.flaticon.com/" title="Flaticon">www.flaticon.com</a></div>

## 第二步，分析一下你的 SVG 長什麼樣子

找到喜歡的 svg icon 後，用瀏覽器打開，接著開啟 devtool，你會看到下面的結果：

![open svg from browser](/img/arvinh/svg-in-browser.png)

裡面的 `path` 跟 `circle` 都是 svg 的 DOM 元件，跟字面上意思一樣，就是 svg 圖案內的線條與圓形。

svg 的 `path`，基本上就是一連串的 svg mini language：

```html
<path d="M 10 25
         L 10 75
         L 60 75
         L 10 25">
```

上面程式碼中的 d 的內容：M 代表將 筆 移動到 (10, 25)，接著 L 畫一條線到 (10, 75)，以此類推。詳細介紹可看[這裡 - SVG Paths and D3.js](https://www.dashingd3js.com/svg-paths-and-d3js)。

透過 devtool，我們可以知道每個 `path` 是對應到畫面上的哪個部分：

![devtool-find-parts](/img/arvinh/animate-svg-devtool.gif)

到這邊大概就會有個方向了，既然我們可以知道每個元件對應到圖案上的哪個部分，我們就能夠針對想要套上動畫的 DOM 元件來操作！

## 第三步，好用的動畫工具 - TimelineLite & TweenMax

當然你可以給予每個 svg 內的 DOM 元件一個獨特的 id，或是 class name，然後用 CSS 或 JavaScript 來自行處理動畫，但這樣難度還是頗高，更重要的是，就沒辦法在十分鐘內做完 XD

所以我們得借用工具，Timeline(Lite|Max) 跟 TweenMax 是知名的 [GreenSock Animation Platform（簡稱 GSAP）](https://greensock.com/)推出的套件，從名稱就可以猜出，主要是提供 **時間軸** 與 **補間** 動畫的 API：

```html

<!-- index.html -->
<div class="example">
  <div class="example__ball"></div>
  <h1 class="example__title">Taiwan No.1!</h1>
  
  <button onclick="animateCircle()"> Build! </button>
</div>
<!-- JavaScript -->
<script>
// 使用 TimelineMax 物件
const tl = new TimelineMax();
// 利用 .set 設定擁有 class name .example__title 的元素
// 將其 scale 縮小成 0.2，透明度設為 0
tl
  .set(".example__title", {
    scale: 0.2,
    autoAlpha: 0
  })
  // 將擁有 class name .example__ball 的元素
  // scale 縮小成 0.2
  .set(".example__ball", {
    scale: 0.2
  })
  // tween 1: 在 1 秒內旋轉 360 度，並翻轉 180 度
  // 接著回復 scale 成原始大小，中間以 Elastic.easeIn 這個 ease function 來控制動畫速度
  .to(".example__ball", 1, {
    rotationX: 360,
    rotationY: 180,
    scale: 1,
    ease: Elastic.easeIn.config(2, 1)
  })
  // tween 2: 跟第一個補間動畫雷同，只是這次是控制文字，並在 0.5 內完成
  .to(".example__title", 0.5, {
    autoAlpha: 1,
    scale: 1,
    ease: Back.easeOut.config(4)
  });
</script>
```

GSAP 甚至提供一個 [Ease Visualizer](https://greensock.com/ease-visualizer/) 讓你可以看看每種 Ease function 的效果，更順帶附上程式碼：

![gsap ease visualizer](/img/arvinh/GSAP-ease-visualizer.png)

上述短短的程式碼就能達到如下效果：

<p class="codepen" data-height="447" data-theme-id="29194" data-default-tab="result" data-user="arvin0731" data-slug-hash="pooKOxo" style="height: 447px; box-sizing: border-box; display: flex; align-items: center; justify-content: center; border: 2px solid; margin: 1em 0; padding: 1em;" data-pen-title="GSAP Tutorial: Simple Timeline">
  <span>See the Pen <a href="https://codepen.io/arvin0731/pen/pooKOxo">
  GSAP Tutorial: Simple Timeline</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>)
  on <a href="https://codepen.io">CodePen</a>.</span>
</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

## 第四步，結合 GSAP 工具與你喜愛的 SVG

GSAP 的 API 文件蠻齊全的，還有論壇可以搜尋相關問題，使用上有任何問題幾乎都能在上面找到答案：[官網文件](https://greensock.com/docs/)、[論壇](https://greensock.com/forums/forum/11-gsap/)

在一開始的房屋範例中，我主要使用的是 TimelineMax 的 `from` 與 `staggerFrom`，跟前步驟內的範例不同，這兩個 API 只需要設定初始值，他會在指定時間內將補間動畫完成：

```js
tl.from('#House > rect:nth-child(24)', 1, {scaleX: 0, transformOrigin: "center", ease: Power2.easeOut})
```

將 CSS Selector `#House > rect:nth-child(24)` 這個元素，從 scaleX 為 0 開始，以 center(中心) 為變形起點，利用 Power2.easeOut 的 ease function，在一秒內回復到原始狀態，並執行補間動畫。

```js
.staggerFrom(['#House > path:nth-child(34)', '#House > path:nth-child(32)'], 0.8, {scaleY: 0, transformOrigin: "bottom", ease: Bounce.easeOut, stagger:0.2}, 0, "scene1+=0.5")
```

與 `from` 雷同，只是 `staggerFrom` 可以一次放入多個 CSS Selector，用 `stagger` 這個屬性來設定陣列中的 Selector 要以怎樣的時間差出現。

詳細 API 參數可以參考[官方文件](https://greensock.com/docs/v2/TimelineMax)

接著回到我們的 SVG，在 devtool 的幫助下，要取出 svg 內部元素的 CSS Selector 非常容易，在 DOM 元件上按右鍵，選擇 `Copy -> Copy selector`，就能直接複製到該元件的 CSS Selector：

![copy selector directly](/img/arvinh/svg-animation-copy-selector.png)

現在我們能取得 svg 中任何部分的 CSS Selector，也知道我們能用 GSAP API 來進行補間動畫，現在是時候將其結合起來！

第一步先調整一下基本 Layout，一般在空白 Html 內直接放入 svg 時，圖案大多會緊靠頁面左上角，因為 svg 本身通常不會有任何關於 layout 的 CSS，所以可以套用個 `margin: 0 auto` 將其置中，看起來會順眼一些，你也能額外加些 padding。此外，為了方便測試動畫效果，也可以放個 button 在頁面上，用來呼叫你的動畫函式：

```html
<!--html part-->
<button onclick="animateBike()"> Build! </button>
<!--css part-->
<style>
#Capa_1 {
  margin: 0 auto;
  display: block;
  width: 256px;
  height: 100%;
}
</style>
```

接著我們使用 `TimelineMax` 提供的 `staggerFrom` 函式，利用 devtool 將滑板車的輪子部分找出來，複製它們的 CSS Selector，放入 `staggerFrom` 函式參數中，設定 x 與 y 軸的 `scale` 都從 0 開始，由 `center` 增長，採用 `Bounce.easeOut` 的 ease function，而四個 Selector 間以 `stagger: 0.2` 的屬性值作為補間動畫出現的時間差：

```js
const tl = new TimelineMax();
  tl
    .staggerFrom(['#Capa_1 > g > path:nth-child(1)', '#Capa_1 > circle:nth-child(7)', '#Capa_1 > path:nth-child(6)', '#Capa_1 > circle:nth-child(5)'],
      1, {scaleY:0, scaleX: 0, transformOrigin: "center", ease: Bounce.easeOut, stagger:0.2})
```

簡單幾行程式碼，就能讓我們的滑板車有了動畫！

<p class="codepen" data-height="417" data-theme-id="29194" data-default-tab="js,result" data-user="arvin0731" data-slug-hash="MWWXPVm" style="height: 417px; box-sizing: border-box; display: flex; align-items: center; justify-content: center; border: 2px solid; margin: 1em 0; padding: 1em;" data-pen-title="Scooter - demo1">
  <span>See the Pen <a href="https://codepen.io/arvin0731/pen/MWWXPVm">
  Scooter - demo1</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>)
  on <a href="https://codepen.io">CodePen</a>.</span>
</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

## 第五步，發揮創意，將整體動畫完成

TimelineMax 所提供的 API 都是 chainable 的，你可以想像成是時間軸一般，動畫跟著 chain 一步一步照順序執行，而 `staggerFrom` 則可以同時讓多個 DOM 元件以些微時間差的順序啟動，另外也能設置一些 Flag 來指定要等到哪幾個動畫完成後，才接續其他動畫，這些細節當你在思考要如何“動畫”你的 SVG 時，再去 GSAP 的文件與論壇查詢，相信都能找到解答。

最後，發揮自己的創意，把 svg 的各個部分都補上動畫，搭配 GSAP 提供的文件，組合各種 API，一個簡單的作品就完成了：

<p class="codepen" data-height="403" data-theme-id="29194" data-default-tab="result" data-user="arvin0731" data-slug-hash="XWWYYWM" style="height: 403px; box-sizing: border-box; display: flex; align-items: center; justify-content: center; border: 2px solid; margin: 1em 0; padding: 1em;" data-pen-title="Scooter - final-steps">
  <span>See the Pen <a href="https://codepen.io/arvin0731/pen/XWWYYWM">
  Scooter - final-steps</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>)
  on <a href="https://codepen.io">CodePen</a>.</span>
</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

## 結論

雖然這個小技巧說破真的不值錢，最困難的技術都交給了 GSAP 處理，SVG 還不用是自己畫的，但加在自己的部落格或是放在投影片中，可以瞬間讓畫面更加豐富，無聊沒事拿來娛樂自己也是很不錯的選擇，像是出國旅行回來有時差睡不著，不如就來畫個動畫調養身心 XD
總之，我自己覺得蠻有趣的，希望或多或少對讀到這篇文章的人有點幫助。

### 資料來源

1. [GreenSock Animation Platform](https://greensock.com/)
2. [How to Create Beautiful SVG Animations Easily](https://medium.com/@LewisMenelaws/how-to-create-beautiful-svg-animations-easily-610eb2690ac3)

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化
