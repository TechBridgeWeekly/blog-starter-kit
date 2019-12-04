---
title: 一小時內製作自己的 Pebble 錶面
date: 2016-11-19 22:02:14
tags:
  - pebble
  - wearable
  - rocky.js
  - pebblekit.js
author: arvinh
---

筆者「最近」買了一隻 Pebble 2，大概是半年多前的事情...當初看上他便宜信譽好，又可以用 javascript 撰寫自己的 watch face，所以就刷下去了，算是第一次在 kickstarter 上面贊助產品，等待的時間久到我都快忘了....才怪，我大概每個月都會想起來一次 XD

總之，經過幾個月漫長等待，以及網站上跟我一樣想趕快拿到產品的 baker 們不斷地詢問下，終於還是送到我手上了！

Tada ~!
<img alt="Pebble 2" style="width: 500px; height: 500px; object-fit: cover" src="/img/arvinh/pebble.jpg" />

...不對這篇不是開箱文

我今天要來簡介一下如何用 Pebble 提供的 Rocky.js Javascript API 來開發 Pebble watchface，讓你可以安裝到自己的手錶上！

# Build your very first pebble watchface

[Pebble 的官網](https://developer.pebble.com/tutorials/)內容蠻豐富的，從簡單的教學、API Doc 到使用者論壇與 blog 都有。

<img alt="Pebble developer site" style="object-fit: cover" src="/img/arvinh/pebble-officialsite.png" />

Pebble 上頭的 app 基本上都是以 C 語言撰寫為主，而距離我上次寫 C 大約是臉書還沒在台灣出現的時代...不過沒關係，Pebble 沒有捨棄 Javascript 這個強大的語言，我們還能利用 Javascript API 與 官方出的 Rocky.js 來撰寫 Pebble watchface！
(只是目前不支援 ES6)

然而，工欲善其事，必先利其器，開發 iOS 我們會想到 Xcode，開發 Android 我們有 Andriod studio，開發 Pebble 呢？

## IDE - [Cloudpebble](https://cloudpebble.net)

<img alt="Pebble Cloud IDE" style="object-fit: cover" src="/img/arvinh/cloudpebble-index.png" />

雖然你也可以透過 command line 去編譯 Pebble 的程式，但我還是要推薦跟介紹他們的雲端 IDE，因為整合得蠻好的，只要以 Pebble 的帳號登入（一開始設定你的手錶時就會要求你註冊），就可以無痛連接你的手機與 Pebble watch。即便開發體驗上面還有許多可以加強的部分，但是已經是非常方便的工具了，而且還有中文！

## Start a new project

<img alt="login" style="object-fit: cover" src="/img/arvinh/cloudpebble-buildproject.png" />

當你登入後可以看到很簡單的介面，列出目前擁有的專案，按下建立按鈕來新增。

<img alt="Create New Project" style="object-fit: cover" src="/img/arvinh/cloudpebble-create.png" />

這邊你會看到好幾種選項，`Pebble C SDK` 應該可以算是最主要的開發工具，可以寫 watchface 也可以寫 app，而 `Pebble.js` 似乎是較為舊版的 javascript SDK，我們今天要介紹的 `Rocky.js`也還在持續更新中。

選了 `Rocky.js` 後，就可以開始進行我們今天的開發了！

先給大家看一下開發完成後，從 IDE 上的模擬器看起來會長什麼樣子。

<img alt="Final result" style="object-fit: cover" src="/img/arvinh/cloudpebble-result.png" />

左邊大大秀出時間，右上角顯示星期幾，右下角則是股票資訊。

## Something you should know before you start to code

Pebble watchface 的 js 開發大致上分為兩塊：Rocky JS 與 PebbleKit JS

Rocky JS 負責**手錶端**的程式，包含 UI 繪製以及與手機端的溝通。

PebbleKit JS 我們先前並沒有提到，他是運行在**手機端**的程式，會安裝在你手機的 Pebble app 內，主要負責與其他 Web Service 溝通，並將訊息傳給手錶作畫面上的顯示與更新。

基本上兩者都是 Javascript，只是多了 Pebble 提供的 API。

## Coding Time!

IDE 左邊的列表中有許多選項，我們會先需要新增一個 index.js，按下 `ADD NEW` button

<img alt="Add new resource" style="object-fit: cover" src="/img/arvinh/cloudpebble-addnew.png" />

有三種類型的 javascript 檔案可以選擇，我們先開發 Rocky.js，繪製基本的時間出來。

先 include `rocky.js`
```js
var rocky = require('rocky');
```
RockyJS 的 API 很簡單，主要是一種 Event-based 的感覺，我們可以透過註冊一個 `minutechange` 的 event 來監聽 **分鐘** 的變化。

```js
rocky.on('minutechange', function(event) {
  // Request the screen to be redrawn on next pass
  rocky.requestDraw();
});
```

每分鐘我們都呼叫一次 `rocky.requestDraw()`，透過這個 function 我們可以發出一個 `Draw` 的 event，而該 event 會帶著一個包含 `CanvasRenderingContext2D` 物件的參數，根據這個參數我們可以有許多 Canvas 相關的 API 可以使用，來繪製我們想要的畫面。

既然是觸發事件，理所當然就是註冊一個 Listener 來處理。

```js
rocky.on('draw', function(event) {
  var ctx = event.context;
  // Clear the screen
  ctx.clearRect(0, 0, ctx.canvas.clientWidth, ctx.canvas.clientHeight);

  // Determine the width and height of the display
  var w = ctx.canvas.unobstructedWidth;
  var h = ctx.canvas.unobstructedHeight;

  drawAPI.drawDigital(ctx, w * (1/5) + 10, h * (1/5) - 5, 'white', '49px Roboto-subset' );
  drawAPI.drawDay(ctx,  w * (4/5) - 5, h * (1/5) - 15);
  // Draw Stock on the bottom of the screen, if available
  if (stockData) {
    drawAPI.drawStock(ctx, stockData, w * (4/5) - 15, h * (3/5) - 10);
  }
});
```

在這段 Listener 中，我做了幾件事情：

1. 從 `event` 中將 `context` 取出來，也就是先前提到的 `CanvasRenderingContext2D`，並且先透過 `clearRect()` 將螢幕清乾淨，因為每次被呼叫到的時候，都代表我們要重新繪製畫面。

2. 將螢幕畫布的寬高暫存起來，用於之後繪製其他圖形時計算各自要擺放的座標位置。

3. 呼叫 `drawAPI` 來繪製需要的內容。`drawAPI` 是我另外寫的一個 js file，將繪圖邏輯與主要 API 做個分離。在這邊我們一樣能夠透過 `require()`將我們自己寫的 js 檔案匯入。

4. 最後是當有股票資料存在的時候，繪製股票資訊，這部分稍後講到 `PebbleKit.js` 的時候會再提到。

主要的邏輯就這麼簡單，註冊 listener，分發 `requestDraw` 事件，然後重繪畫面！

### How to draw?

接著就是要發揮你們藝術家天份的時刻了，利用 `CanvasRenderingContext2D` 提供的介面，我們可以輕易地畫出長方體、圓形、路徑與文字，並且設定顏色與字型大小等等，詳細的 API 參數可以直接參考官網，因為能用的 API 其實不多，所以看起來也很清楚。

[CanvasRenderingContext2D API Doc](https://developer.pebble.com/docs/rockyjs/CanvasRenderingContext2D/)

這邊講解我用到的部分：

```js
 drawAPI.drawDigital(ctx, w * (1/5) + 10, h * (1/5) - 5, 'white', '49px Roboto-subset' );
 ```

 `drawDigital` 是用來繪製畫面上的數位時間，也就是螢幕左邊大大的數字，參數很簡單，就是 context object、x, y 的座標位置、顏色與字型大小。

 這邊有兩點要注意，座標位置跟一般 web 上的標準一樣，螢幕左上角為 (0, 0)，計算每個圖形的位置會是初期開發簡單的 watchface 中較為麻煩的地方，因為畫面很小，你要好好調整才行。

另外一個要小心的是，雖然你看這邊的字型大小寫法，似乎跟一般 CSS 的格式一樣，但你可不能自己隨意亂加大小或是 font-family，需要使用它們定義好的才能生效。
[可用字型列表-link](https://developer.pebble.com/docs/rockyjs/CanvasRenderingContext2D/#font)

實際的 drawDigital function
```js
var hourAndMin = new Date()
                .toLocaleTimeString()
                .split(':')
                .splice(0,2);
var hourText = hourAndMin[0];
var minutesText = hourAndMin[1];
ctx.fillStyle = color;
ctx.textAlign = 'center';
ctx.font = font;
ctx.fillText(hourText, cx, cy - 20, 70);
ctx.fillText(minutesText, cx, cy + 35, 70);
```
透過 `ctx` 可以設定 fillStyle、textAlign、font style，並使用 `fillText()` 來將文字繪製在畫面，參數分別是：文字內容, x 座標, y 座標, 最大寬度(option)，若你有設定**最大寬度**，當你的文字大於這寬度時，會自動使用較小的字體。

繪製星期的部分其實與時間大同小異，直接看程式碼：

```js
var day = '';
switch (new Date().getDay()) {
  case 0:
      day = "SUN";
      break;
  case 1:
      day = "MON";
      break;
  case 2:
      day = "TUE";
      break;
  case 3:
      day = "WED";
      break;
  case 4:
      day = "THU";
      break;
  case 5:
      day = "FRI";
      break;
  case 6:
      day = "SAT";
  }

  ctx.fillStyle = 'lightgray';
  ctx.textAlign = 'center';
  ctx.font = '24px bold Gothic';

  ctx.fillText(day, cx, cy, 30);
```

當你寫完這些後，你就可以先試著執行看看。

點擊右邊選項中的播放鍵，就會自動編譯並啟動模擬器。若是沒有問題，你會看到模擬器開啟，並出現一條 progress bar 顯示正在安裝，第一次會比較久一點。

成功後就會出現畫面，並有個彈跳視窗出來，這邊可能是翻譯問題，所謂的 **解除** 其實就只是取消這個彈跳視窗罷了，我一開始還不太敢按，怕他把我的 App 解除安裝 XD

<img alt="run project" style="width: 800px;" src="/img/arvinh/cloudpebble-run.gif" />

## Fetch Stock Data

既然是智慧手錶，當然不能只有單純的顯示時間，接下來說明怎麼樣搭配 `PebbleKitJS` 透過你的手機來獲取 Web 資料，並傳送給手機顯示。

PebbleKit JS 是運行在你手機端的 Pebble app 裡面，一樣也是 Event-based 的方式：

```js
Pebble.on('message', function(event) {
    ...
});
```
因為是運行在 Pebble 自己的環境下，所以我們這邊不需要 require 什麼 library，
直接註冊 `message` event 的 listener 即可。

手錶端的 RockyJS 可以透過 `postMessage()` 傳送訊息給 PebbleKit，當 PebbleKit 的 listener 監聽到傳送過來的 `message` 事件後，就可以採取相對應的措施，像是發送 Ajax 抓取 web 資料，並同樣透過 `postMessage()` 將資料傳回給手錶端的 RockyJS。

完整的手機端 PebbleKit JS 
```js
// Get the message that was passed
var message = event.data;

// we random pick one of these stock symbol to show
var stockSymbols = ['YHOO', 'GOOGL', 'AAPL'];
var randStockSymbol = stockSymbols[Math.floor(Math.random() * stockSymbols.length)];
if (message.fetch) {
// use yql to fetch data (don't use in productin or sell)
var url = 'https://query.yahooapis.com/v1/public/yql' + 
    '?q=select * from yahoo.finance.quotes where symbol in ' +
    '("'+randStockSymbol+'")&format=json&env=store://datatables.org/alltableswithkeys&callback=';

  request(encodeURI(url), 'GET', function(respText) {
    var stockData = JSON.parse(respText);

    Pebble.postMessage({
      'stockData': {
        'symbol': stockData.query.results.quote.symbol,
        'Ask': stockData.query.results.quote.Ask,
        'Bid': stockData.query.results.quote.Bid
      }
    });
  });
}
```

註1: 因為不能用 es6 來寫，所以也沒有 `fetch` 可以用，所以這邊的 request 是自己寫的 XMLHttpRequest function。
註2: 記得 encodeURI 一下 url，否則有可能會出現 invalid url error

接著我們回到手錶端的 RockyJS，在原本的程式下加入這兩段：

```js
rocky.on('hourchange', function(event) {
  // Send a message to fetch the weather information (on startup and every hour)
  rocky.postMessage({'fetch': true});
});


rocky.on('message', function(event) {
  // Receive a message from the mobile device (pkjs)
  var message = event.data;

  if (message.stockData) {
    // Save the stockData data
    stockData = message.stockData;

    // Request a redraw so we see the information
    rocky.requestDraw();
  }
});
```

在 `hourchange` 事件發生時，我們 `postMessage()` 告訴 PebbleKitJS 需要去抓取新資料，並且註冊 `message` 的 event listener，當資料傳送回來時，我們重繪製畫面。
（這邊直接把資料存在全域變數，方便讀取）

## Final Result!

Tada~ 股票資訊就出現了！

<img alt="run project" style="width: 800px;" src="/img/arvinh/cloudpebble-run-stock.gif" />

當然不能一直在模擬器上跑，我們要安裝到手錶上！

<img alt="install to watch" style="width: 800px;" src="/img/arvinh/cloudpebble-build-phone.gif" />

點選左邊選項列表的 **編製**，選擇 **PHONE**，並安裝執行即可！

記得打開手機上 Pebble app 內的 developer mode

<img alt="open developer mode" style="width: 300px;" src="/img/arvinh/cloudpebble-developmode.jpg" />

編譯成功的話就可以在手錶上看到畫面，若失敗有問題，可以檢視編譯結果，從 log 中去找問題，像我一開始就忘記 encodeURI，導致 ajax 出問題。

<img alt="check build result" style="width: 800px;" src="/img/arvinh/cloudpebble-log.png" />

上圖內的 PBW，是可以讓你下載下來，之後要 publish 到 store 的時候所使用的，至於怎麼 publish？
這邊再講下去篇幅有點長，會另外寫一篇來介紹。

手錶上看起來長這樣~

<img alt="watch" style="width: 300px;" src="/img/arvinh/cloudpebble-watch.jpg" />

本文中介紹的程式碼在此：
https://github.com/ArvinH/clean-stock-pebble-wf

若想看官網上指針時鐘與天氣的範例，我也有個修改版本的：
https://github.com/ArvinH/pebble-watchface-starterkit

## 資料來源
1. [Pebble build with JS](https://developer.pebble.com/tutorials/js-watchface-tutorial/part1/)
2. [Pebble forum](https://forums.pebble.com/)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化