---
title: 一起用 Google Calendar 與 D3.js 進行年終回顧吧！
date: 2017-12-12 21:10:27
tags:
  - google
  - google calendar api
  - d3
---

# 前言

大概是在 2016 年底的時候，我整個人的心理狀態很糟，一直覺得自己對生活的掌控力非常低落，庸庸碌碌的過著每一天，卻不曉得自己的目標在哪，對什麼都興致缺缺，似乎把生活遺失了。

我試著想從過往日常生活中的所作所為來找出一些癥結點，結果發現記憶有限，越想越覺得自己好像什麼都沒完成，接著就進入無止盡的負面迴圈...

為了打破這樣的心理狀態，我從 2017 年的一月開始記錄自己每天的生活，將一整天所做的事項記錄在 Google Calendar 中，有了這樣的紀錄後，我每週都會回顧一下自己這一整個星期所做的事情，花在工作、運動、娛樂、學習等等的時間如何，讓自己在負面情緒滿漲時，能看看自己其實還是達成了不少事，同時也能隨時警惕自己在時間管理與各種事項間的分配是否有需要調整的部分。

好啦，說穿了就是用 Google Calendar 寫日記。

而現在又接近年底了，在這近乎一年的紀錄中，我想應該可以來做個 Year End Review，利用 D3 來幫助我將記錄在 Google Calendar 上的資料視覺化出來！

大致的想法是將不同分類的資料整理出來，然後看看 **<em>我在這一年當中，每一天花在每一個類別的事項的 Heat Map</em>** 會是長什麼樣子，並加上簡單的 Select box 方便切換。

成果截圖如下：

![YERR](/img/arvinh/arvin-yerr-calendar.png)

有興趣的可以連結到 [bl.ocks 看 Demo 與完整的程式碼](https://bl.ocks.org/ArvinH/56909246bd584743fa5ee7ad148ad1ac)

# 資料視覺化的第一步，當然是取得資料

Google 存了多少資料想必大家都心知肚明，而他老大哥也是願意把我們的資料還給我們，只要連結到 [Google Takeout](https://takeout.google.com/settings/takeout) 就可以將你在 Google 服務上的資料副本下載下來：

![Google Takeout](/img/arvinh/google-takeout.png)

既然我要視覺化我在 Google Calendar 上的資料，當然就是選取 Calendar 來下載：

![Google Calendar download](/img/arvinh/google_calendar_details.png)

或著，你也能直接從 Google Calendar 匯出你的日曆：

![Export from Google Calendar](/img/arvinh/google_calendar_export.png)

仔細看一下，這兩種方式下載到的資料其格式都是 [`iCalendar`](https://zh.wikipedia.org/wiki/ICalendar)。

是 `iCalendar` 又如何呢？通用的標準格式不是很好嗎？代表我之後若換成 Apple 的 iCal，我這次的專案也能套用上去耶！

But!

在我當初紀錄日曆時，我每一項 item 的資訊很少，就只是單純的填入 Summary，並且依照分類給予顏色。

而在 `iCalendar` 的標準中，並沒有 Google Calendar 的顏色資訊，所以我只好忍痛放棄這條方便的道路...（這告訴我們做事還是不能太懶）

iCalendar 的內容大概長這樣，以 `BEGIN:VEVENT` 開始，`END:VEVENT` 為止：

```ics
BEGIN:VEVENT
DTSTART;VALUE=DATE:20171209
DTEND;VALUE=DATE:20171210
DTSTAMP:20171210T075750Z
UID:839skadkfcd03812j3f0c030s0s@google.com
CREATED:20171210T024308Z
DESCRIPTION:
LAST-MODIFIED:20171210T024310Z
LOCATION:
SEQUENCE:0
STATUS:CONFIRMED
SUMMARY:運動（主：背）
TRANSP:TRANSPARENT
BEGIN:VALARM
ACTION:DISPLAY
DESCRIPTION:This is an event reminder
TRIGGER:-F0AB0H30C0D
END:VALARM
BEGIN:VALARM
ACTION:EMAIL
DESCRIPTION:This is an event reminder
SUMMARY:Alarm notification
ATTENDEE:mailto:arvin0731@gmail.com
TRIGGER:-F0AB0H30C0D
END:VALARM
END:VEVENT
```

## 既然是 Google，就會有 API

沒辦法直接載到資料，那就從 API 取得吧！

Google 最棒的一點就是有一堆 API，而為人詬病的剛好是這些 API 常常沒有完善的 Doc。

幸運的是，Google Calendar API 的文件還蠻好懂的，因為給了一個[直接可以複製來用的範例 Sample](https://developers.google.com/google-apps/calendar/quickstart/nodejs)（笑

直接照著步驟做就可以了，他有很多實作版本，我是用熟悉的 Node.js 來幫我爬取資料。

唯一要注意的是步驟一中的 [this wizard](https://console.developers.google.com/start/api?id=calendar) 要好好照著說明執行，即便他步驟看起來有些匪夷所思，像是要你到某一頁後按下 `Cancel` button (eg. On the Add credentials to your project page, click the Cancel button.)

當你設定好一切，取得 OAuth2 需要的 credentials 後，可以參照他下面的範例依據你想要的資料作修改，比較關鍵的程式是這幾行：


```js
    var calendar = google.calendar('v3');
    var queryOptions = {
        auth: auth,
        calendarId: 'primary',
        timeMax: (new Date()).toISOString(),
        singleEvents: true,
        orderBy: 'startTime',
        maxResults: 2500 // 不給 maxResults 的話，預設值就是 2500
    };
    if (pageToken) {
        queryOptions.pageToken = pageToken;
    }
    calendar.events.list(queryOptions, function (err, response) {
        // ... 從 response 中取得需要的資料
        // var events = response.items;
    }
```

我們使用的是 v3 的 google calendar api，利用 `calendar.events.list` 可以取得某個 Calendar 的 events 列表，在 `queryOptions` 中，比較重要的有下面幾個:

`auth` 就是你在 [this wizard](https://console.developers.google.com/start/api?id=calendar) 中產生的 credentials，你在頁面上就能找到下載 json 的連結：

![download credentials](/img/arvinh/googleapi-credentials.png)

`calendarId` 是你想抓取的 Calendar 名稱，primary 指的是你登入帳號的主要日曆。

`timeMax` (or `timeMin`) 可以用來規範你想抓取哪段時間內的 events。

最後是 `pageToken`，由於 Google Calendar API 有限制，你一次最多只能抓取 2500 個 events，即便你有設定 `maxResults` 這個參數也一樣。因此你會需要透過每次 API request 回傳的 `nextPageToken` 來進行下一頁的 Query。（沒錯，response 回來的是 `nextPageToken`，但你下 Query 時帶的是 `pageToken`，請注意！）

可以設定的參數還非常多，可以從 Google 提供的 [API Explorer](https://developers.google.com/apis-explorer/#p/) 中選取 `Calendar API` 來測試看看哪樣的參數是你需要的！每個參數都有對應的說明，還算清楚。

我在這部分的實作放在 [Gist](https://gist.github.com/ArvinH/a46f9af2df4316651d2056e610ba68b1) 上，有興趣可以參考，
寫得很暴力簡陋，畢竟這不是這次的重點～

主要是依照日曆中每筆 Event 的時間與顏色做分類與統計，依照不同顏色（也就是不同類別）分別存放到不同檔案。

資料格式也很簡單：

```csv
date,colorIdNum
2017-10-10,1
2017-10-11,1
2017-10-12,1
2017-10-13,1
2017-10-14,1
2017-10-15,1
...
..
.
```

# 資料有了終於能開始畫圖了！

主要參考 [Mike Bostock 大神的作品](https://bl.ocks.org/mbostock/4063318)，改成只繪製單一年度圖表，並加上切換資料後的動畫。

接下來是手把手說明：

我們總共需要三個 Function：`DrawCalendar`、`UpdateCalendar` 與 `changeDataSrc`，分別用來繪製圖表、更新圖表與更新資料。

最主要的當然就是 `DrawCalendar`。繪製日曆又可分為三個步驟：

### Steps I: 設定日曆基本外觀（大小、顏色、svg container）

```js
function DrawCalendar(dataSrc) {
  var width = 960,
      height = 136,
      cellSize = 17;

  var formatPercent = d3.format("d");

  var color = d3.scaleQuantize()
      .domain([0, 10])
      .range(["#006837", "#1a9850", "#66bd63", "#a6d96a", "#d9ef8b", "#ffffbf", "#fee08b", "#fdae61", "#f46d43", "#d73027", "#a50026"]);
  // ...
  // ..
  // .
}
```

上面這段很單純的就是設置好圖表的基本資訊，包含寬、高與日曆中每"天"的格子大小，利用 `d3.scaleQuantize()` 創建一個 color 函數，用來將之後的資料映射到對應的顏色。

這邊有個 `d3.format("d")` 的函數，其實在這邊用不到，但還是想介紹一下。
`d3.format()` 是個方便你正規化資料數值的函式，例如你想將數值侷限在小數點後兩位，就可以用 `d3.format("2f")`，或是想轉換成百分比，可以用 `d3.format(".2%")`，這樣會將你輸入的數值乘上 100 後，再加上百分比符號，並依照 % 前的數字來決定小數點後的位數。

eg. d3.format(".2%")(0.234) === 20.34% [(可參考此網站說明)](http://www.oxxostudio.tw/articles/201501/svg-d3-12-formatting.html)

接著是繪製主要的 calendar chart svg container：

```js
var svg = d3.select("#calendar-chart")
      .selectAll("svg")
      .data(d3.range(2017, 2018))
      .enter().append("svg")
      .attr("width", width)
      .attr("height", height)
      .append("g")
      .attr("transform", "translate(" + ((width - cellSize * 53) / 2) + "," + (height - cellSize * 7 - 1) + ")");

  svg.append("text")
      .attr("transform", "translate(-6," + cellSize * 3.5 + ") rotate(-90)")
      .attr("font-family", "sans-serif")
      .attr("font-size", 10)
      .attr("text-anchor", "middle")
      .text(function (d) { return d; });
```

[`d3.range()`](https://github.com/d3/d3-array#range) 會回傳根據你傳入的區間，回傳一段連續數值陣列。

我們利用 `d3.range(2017, 2018)` 取得陣列資料，會回傳長度為一，包含 2017 這個數值的陣列 `[2017]`。

你可能會想，為什麼我要這樣做？

只需要一年的資料，就直接 append svg 就好呀，何必 binding 2017 這個值到 svg 的 `__data__` 裡面呢？

賣個關子，待會看到 Steps II 你就會知道了！

而剩下的部分就是利用 `transform` 來位移 svg，為整個圖表保留空間，並加上文字標記。

### Steps II: 繪製日曆中的每一天（也就是每個小格子啦～）

```js
var rect = svg.append("g")
  .attr("fill", "none")
  .attr("stroke", "#ccc")
  .selectAll("rect")
  .data(function (d) { return d3.timeDays(new Date(d, 0, 1), new Date(d + 1, 0, 1)); })
  .enter().append("rect")
  .attr("width", cellSize)
  .attr("height", cellSize)
  .attr("x", function (d) { return d3.timeWeek.count(d3.timeYear(d), d) * cellSize; })
  .attr("y", function (d) { return d.getDay() * cellSize; })
  .datum(d3.timeFormat("%Y-%m-%d"));

  svg.append("g")
    .attr("fill", "none")
    .attr("stroke", "#000")
    .selectAll("path")
    .data(function (d) { return d3.timeMonths(new Date(d, 0, 1), new Date(d + 1, 0, 1)); })
    .enter().append("path")
    .attr("d", pathMonth);
```

要繪製出一年中的 "每一天"，我們可以直接 for loop 跑 365 次，或是產生 365 筆資料，但更聰明的做法是利用 [d3-time 提供的 timeDays 函數](https://github.com/d3/d3-time#timeDays) 來幫我們產生出特定日期區間的日期資料！

`d3.timeDays(new Date(d, 0, 1), new Date(d + 1, 0, 1));`

其中 `new Date()` 的第一個參數 `d` 代表年，`0` 代表第一個月（月份從 0 開始），`1` 則是第一天，因此這邊會回傳 d 年到 d + 1 年間的每一天。

好那問題來了，這個 `d` 是什麼呢？

要回答這個問題，必須先了解一下 d3 中的 `selection.data()`。

我以前都以為 `data()` 只能夠丟入資料陣列，但是在這次的實作過程中才發現，原來他可以接受函數！

但他是怎麼運作的呢？我們偷看一下 [d3-selection 的原始碼](https://github.com/d3/d3-selection/blob/master/src/selection/data.js#L94)

![d3-selection src](/img/arvinh/d3-selection-src.png)

真的蠻有趣的，他其實是先判斷你傳入的 `value` 是不是函數，若不是，會先利用 `constants()` 將你的值包裹過，讓他變成可執行的函數：

```js constants.js
export default function(x) {
  return function() {
    return x;
  };
}
```

他會將 `parent.__data__` 傳入該函數中執行，回傳的就會是該 `selector` 的 `__data__`。所以直接傳入 function 也是可行的！

以我們的例子來說，`rect` 的 parent 就是 `svg`，而你們記得在 Steps I 時，我們 binding 陣列 `[2017]` 到 `svg` 的 `__data__` 中嗎？這邊就派上用場了！在 rect 的 `data()` 中的函數所接收到的 `d` 值就是 `2017`。

而上述的 `d3.timeDays(new Date(d, 0, 1), new Date(d + 1, 0, 1));` 就會回傳 2017 到 2018 中間 365 筆的日期資料！我們也因此得到 365 個小方塊啦！

接著，利用類似的概念，使用 `d3.timeMonths()` 產生 12 筆月份資料，接著實作 `pathMonth()` 來畫出 svg path：

```js pathMonth()
function pathMonth(t0) {
  var t1 = new Date(t0.getFullYear(), t0.getMonth() + 1, 0),
    d0 = t0.getDay(), w0 = d3.timeWeek.count(d3.timeYear(t0), t0),
    d1 = t1.getDay(), w1 = d3.timeWeek.count(d3.timeYear(t1), t1);
  return "M" + (w0 + 1) * cellSize + "," + d0 * cellSize
    + "H" + w0 * cellSize + "V" + 7 * cellSize
    + "H" + w1 * cellSize + "V" + (d1 + 1) * cellSize
    + "H" + (w1 + 1) * cellSize + "V" + 0
    + "H" + (w0 + 1) * cellSize + "Z";
}
```

這段邏輯看似複雜，其實很直覺。

[`d3.timeWeek.count(start, end)`](https://github.com/d3/d3-time#interval_count) 可以計算出 start 到 end 這兩個日期間有幾週。

`t0` 是目前傳入的時間，依照 d3.timeMonths() 的產出，會是一個月的第一天，`t1` 則是該月的月底。
`d0` 與 `d1` 就是單純的 `t0`、`t1` 對應的 day，就是星期幾。
`w0` 算出從 `t0` 該年的第一天到 `t0` 有幾週，依此類推 `w1` 就是相對於 `t1`。

用這些數值就能夠 `M(移動)` 到月初的位置，並往下移動 7 天 `V 7*cellSize`，接著在水平位移(H) 到月底的位置，然後垂直位移到月底日期的星期位置`(d1 +1) * cellSize`，最後為到原點把整個月份框起來。

svg path 的繪製方法可以[參考這篇](http://www.oxxostudio.tw/articles/201406/svg-04-path-1.html。

到目前為止可以畫出如下的圖：

![outline](/img/arvinh/months-outline.png)

### Steps III: 該開始 Binding 資料了

再來算是最後一步了，我們將資料綁定並填入剛剛產生的格子中！

```js
d3.csv(dataSrc, function (error, csv) {
  if (error) throw error;

  var data = d3.nest()
    // 將 date 抽到外層當 key
    .key(function (d) { return d.date; })
    // 透過 rollup 將 value 設為 colorId
    .rollup(function (d) { return +d[0].colorIdNum; })
    .object(csv);

  rect.filter(function (d) {
    return d in data; // 找出有 match 到的日期，表示那天我有做事 XD
  })
    .attr("fill", function (d) { return color(data[d]); })
    .append("title")
    .text(function (d) { return d + ": " + formatPercent(data[d]); });
});
```

這邊主要就兩個步驟，第一步先用 [d3.nest](https://github.com/d3/d3-collection#nests) 來整理資料，我們需要把陣列中每筆資料的 "date" 抓出來當 Key，如此一來，在第二步中，我們可以直接利用 `d in data` 的語法在 `selection.filter()` 中進行過濾，選出 365 個格子中，哪些有跟我們真正的 csv 日期資料 match，有 match 到的格子，我們填入顏色！（使用最前面的 color 函數來做映射）

填入和顏色後：

![填入資料顏色後的 calendar](/img/arvinh/color-calendar.png)

### Steps IV: 加入動態切換資料的功能

最後我們實作一下綁定在 `<select>` 元素上的 `onchange` 函數來做資料切換，而在圖表方面，我們只需要選取出 `rect` 並重新 `filter` 資料即可：

```js
// 切換資料
function changeDataSrc() {
  var x = document.getElementById("dataSrcSelector").value;
    UpdateCalendar(`calendarData-${x}.csv`);
}

// 更新圖表，加上 `transition()` 與 `duration()` 讓過程滑順一些
function UpdateCalendar(dataSrc) {
  // ...
  // ..
  d3.selectAll('rect').filter(function (d) {
    return d in data;
  })
    .transition()
    .duration(500)
    .attr("fill", function (d) { return color(data[d]); });
  
  d3.selectAll('rect').filter(function (d) {
    return !(d in data);
  })
    .transition()
    .duration(500)
    .attr("fill", function () { return '#fff'; });
}
```

## 成果

![final demo](/img/arvinh/calendar-d3.gif)
[Demo 與完整的程式碼](https://bl.ocks.org/ArvinH/56909246bd584743fa5ee7ad148ad1ac)

嗯～看起來我工作頗認真，不過等等...怎麼好像週末顏色都比較深啊哈哈哈...哈哈..哈..嗚嗚嗚

# 結論

當初開始在 Google Calendar 上紀錄日誌時，並沒有想到可以在年終的時候進行整年的 review，所以每天的紀錄都很隨性，並沒有很完整的進行分類，有些可惜，但還是能透過資料視覺化的結果，看出我在時間分配的掌控度有相當的進步空間，像是花在工作與培養額外興趣（娛樂）的時間有蠻大的落差。

不管怎麼說，有了這次的經驗，相信明年的 Year End Review 會更好！
如果你也覺得這樣做有趣，或許可以加入我的行列，也或許想想有什麼方式可以為你自己進行一次年終 Review，相信都會有所收穫！

### 資料來源
1. [d3 formatting](http://www.oxxostudio.tw/articles/201501/svg-d3-12-formatting.html)
2. [d3-time](https://github.com/d3/d3-time)
3. [d3-array](https://github.com/d3/d3-array)
4. [d3-collection](https://github.com/d3/d3-collection)
5. [MDN Date](https://developer.mozilla.org/zh-TW/docs/Web/JavaScript/Reference/Global_Objects/Date)
6. [mbostock calendar view](https://bl.ocks.org/mbostock/4063318)
7. [Google Calendar API](https://developers.google.com/google-apps/calendar/quickstart/nodejs)

關於作者： 
[@arvinh](https://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化

