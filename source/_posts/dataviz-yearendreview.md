---
title: 用 Google Calendar 與 nivo 製作自己的年終檢討報告
date: 2018-12-15 15:15:09
tags:
  - data
  - visualization
  - d3
  - nivo
  - google calendar
  - yearend
---

# 前言

從 2017 年開始，我每天都會用 Google Calendar 紀錄生活，也在年底的時候利用 D3.js 與 Google api 將紀錄的資料視覺化出來做個年終回顧。（沒看過的讀者可以往這裡走：[一起用 Google Calendar 與 D3.js 進行年終回顧吧！](https://blog.techbridge.cc/2017/12/12/d3v4-calendar-yearendreview/)）

2018 當然也不例外，我依然持續記錄每天的日常，透過每週回顧自己的時間花費來調整目標與心理狀態。

而既然我有了兩年的資料，不拿來比較看看就太可惜了，因此決定在我今年的最後一篇文章中，將 2017 年與 2018 年的資料做個視覺化的比較，看看我在工作、生活、娛樂以及自我進修等上面的時間調配是否有照著我去年所希望的步調進行。在一年的尾聲，好好面對自己！

一樣先附上成果與 [demo link](https://blog.arvinh.info/YearEndReview-2018/)：

![YearEndReview-2018](/img/arvinh/YearEndReview-2018_.png)

## 工具

工具的使用上，這次並沒有使用 D3.js，畢竟以需求來說，D3.js 太大材小用了，加上我前陣子發現一個基於 D3.js 與 React 的視覺化 library - [nivo](https://nivo.rocks)，看起來非常不錯，所以這次就拿這套來試試看，透過這套工具，整個作品大概不用花費一小時就可以完成了，大多的時間都是在處理資料格式上。

而抓取 Google Calendar 資料的部分，一樣使用 Google 的 api，只不過一年過去，Google api 也是有所更新，不過問題不大，稍微修改一些小地方即可，稍後會帶到。

## 視覺化的第一步，取得資料

在[一起用 Google Calendar 與 D3.js 進行年終回顧吧！](https://blog.techbridge.cc/2017/12/12/d3v4-calendar-yearendreview/)中我有提到過，雖然 Google 有提供一個 [Google Takeout](https://takeout.google.com/settings/takeout) 的功能，你也能直接匯出日曆，但是下載到的資料其格式都是 [`iCalendar`](https://zh.wikipedia.org/wiki/ICalendar)，除非你在每個紀錄的項目上都有很詳細的紀錄內容，才有辦法去將每一項 task 都進行分類，像我只用顏色來區分的就沒辦法。

![Google Takeout](/img/arvinh/google-takeout.png)

所以還是得靠 Google 提供的 Calendar API 來取得資料。

而 Google Calendar API 的使用方式很簡單，只要照著[說明](https://console.developers.google.com/start/api?id=calendar)拿到憑證後，就可以從他們提供的[範例](https://developers.google.com/google-apps/calendar/quickstart/nodejs) 來修改實作。

基本上跟我[去年寫的](https://blog.techbridge.cc/2017/12/12/d3v4-calendar-yearendreview/)差不多，主要是在 auth 的套件（拿 cretential 的方式）以及 api 回傳的 schema 有所變動：

```diff
- var googleAuth = require('google-auth-library');
+ const { google } = require('googleapis');
// If modifying these scopes, delete token.json.
// The file token.json stores the user's access and refresh tokens, and is
// created automatically when the authorization flow completes for the first
// time.
var SCOPES = ['https://www.googleapis.com/auth/calendar.readonly'];
- var TOKEN_DIR = (process.env.HOME || process.env.HOMEPATH ||
-  process.env.USERPROFILE) + '/.credentials/';
- var TOKEN_PATH = TOKEN_DIR + 'calendar-nodejs-quickstart.json';
+ const TOKEN_PATH = 'token.json';
```

```diff
function listEvents(auth, pageToken) {
- var calendar = google.calendar('v3');
+ const calendar = google.calendar({version: 'v3', auth});
const queryOptions = {
  auth: auth,
  calendarId: 'primary',
  timeMax: (new Date()).toISOString(),
  singleEvents: true,
  orderBy: 'startTime'
};
if (pageToken) {
  queryOptions.pageToken = pageToken;
}
calendar.events.list(queryOptions, function (err, response) {
if (err) {
  console.log('The API returned an error: ' + err);
  return;
}
-var events = response.items;
+const resData = response.data || {};
+const events = resData.items;
if (events.length == 0) {
  console.log('No upcoming events found.');
} else {
  // ...
}
// ...
}
}
```

完整的程式碼我一樣有放在 [gist](https://gist.github.com/ArvinH/5960a33b54fb03d504bcba7a4d5982e5#file-fetchcalendardata-js-L86-L151) 上，如果你也想要用 Google API 下載自己的 Calendar，基本上除了 `listEvents` 函數內的部分以外，都可以直接照抄，`listEvents` 內的程式碼是 API 的 callback，主要是負責後續資料的處理，將之轉化成我們視覺化所需要的格式。而關於 Calendar api 相關參數的介紹，可以參考[去年的文章](https://blog.techbridge.cc/2017/12/12/d3v4-calendar-yearendreview/)。

# nivo

接下來介紹一下 [nivo](https://nivo.rocks) 這套工具。

它是 [Raphaël Benitte](https://twitter.com/benitteraphael) 所製作，基於 D3.js 與 React.js 的視覺化套件。

![nivo](/img/arvinh/nivo-cover.png)

比起市面上其他套件，我自己認為 nivo 的優勢有幾點：

* 對於 Server-side render 的支援度很高。

* 由於支援 React SSR，nivo 提供了 api 介面，讓你能透過 http request 產生圖表 - 詳情可看 [nivo-api](https://github.com/plouc/nivo-api)。

* Mono repo 的方式管理 packages，可以依照需求，只安裝你需要的圖表 package。

* 宣告式的寫法，加上他互動式的文件範例，套用他的圖表幾乎沒有學習曲線。

![nivo 文件網站](/img/arvinh/nivo-bar-doc.png)

* 部分元件提供 Canvas 實作的版本，大量數據也不怕！

* 動畫部分使用 `react-motion`，寫過 react 的人應該不陌生，`react-motion` 可說是 react 動畫界的翹楚啊！

* 每個元件都有對應的 `Storybook` 可以看，非常的 developer-friendly!

當然也是有缺點的：

* 不是每個元件都支援動畫，這時候會知道 D3.js 的好(?

* 圖表種類固定，比較難有創意的發揮。

基於以上的優缺點，不難看出，如果是對於不熟悉 D3.js 或是想要快速產出圖表，但又想要有動畫效果或是互動性的人來說，直接採用 nivo 這類的 library 會是很不錯的選擇，也是我這次採用的原因。

# 開始進行我們的圖表製作吧！

基本的想法一樣是將資料用 Calendar 圖表呈現，觀察每種類別的 task 在一年中的分佈狀況，並且加入去年的資料來做對比。

接著，想利用一個折線圖來觀察單一類別在每個月的波動，是不是在某些月份我比較勤奮工作、某些月份很認真在進修或玩樂。當然這些資訊從 Calendar 圖表也看得出來，但是用月加總的數值呈現在折線圖上，感覺會更直接一點。

最後在把每種類別的資料集結起來，利用 Waffle 圖表，一目瞭然各個類別的比例關係。

要達成上面三個想法，我們只需要使用 nivo 的 [@nivo/calendar](https://github.com/plouc/nivo/tree/master/packages/calendar)、[@nivo/line](https://github.com/plouc/nivo/tree/master/packages/line) 與 [@nivo/waffle](https://github.com/plouc/nivo/tree/master/packages/waffle) 三種套件即可。

使用方法都很簡單，基本上只要到對應的 doc 頁面，就直接有現成的程式碼可以~~複製~~參考。

像是 [Calendar](https://nivo.rocks/calendar):

![calendar doc demo](/img/arvinh/nivo-calendar-doc.gif)

重點反而是資料格式，即便我們只是想繪製三種圖表，但就需要整理出三種資料格式，才能夠符合其各自的需求，一樣可以從 nivo 的 doc 中找到每個圖表所需要的資料格式（data tab）。

我製作範例的 parser 放在 [github](https://github.com/ArvinH/YearEndReview-2018/tree/master/src/dataParser) 上，有興趣可以參考，不過就只是一些髒髒的資料轉換，是個耗時費力的工作啊...

值得一提的是，這邊我除了轉化資料格式外，還花了不少時間在做資料的整理與同步。

為什麼呢？

因為每天的日誌是我自己手動輸入，顏色 label 也是我自己標記，有些事項的類別在 2017 年與 2018 年我用的是不同的顏色，但卻應該是分在同類中，這在做對比時，就會有問題了，因此花了不少時間從 2017 年一月開始過到 2018 年十二月...所以如果有人想跟我一樣這樣紀錄的話，或許可以用 Calendar 內建的分類功能，而不要像我一樣用手動標記顏色的方式。

總之，當資料處理好以後，照著網站範例將我們想要的元件放到頁面上即可，這邊我是直接用 CRA 製作 React SPA，其中 `Calendar` 的例子：

```js
import React from 'react';
import { ResponsiveCalendar } from '@nivo/calendar'
const Calendar = ({data}) => (
  <ResponsiveCalendar
        data={data}
        from="2017-01-01"
        to="2018-12-31"
        emptyColor="#eeeeee"
        colors={[
            "#61cdbb",
            "#97e3d5",
            "#e8c1a0",
            "#f47560"
        ]}
        margin={{
            "top": 100,
            "right": 30,
            "bottom": 60,
            "left": 30
        }}
        yearSpacing={45}
        monthBorderColor="#ffffff"
        monthLegendOffset={10}
        dayBorderWidth={2}
        dayBorderColor="#ffffff"
        legends={[
            {
                "anchor": "bottom-right",
                "direction": "row",
                "translateY": 36,
                "itemCount": 4,
                "itemWidth": 34,
                "itemHeight": 36,
                "itemDirection": "top-to-bottom"
            }
        ]}
    />
);
export default Calendar;
```

從上面的範例看得出來，你有許多 `options` 可以設定，像是資料的起始日期(`from`、`to`)、資料顏色的區間或是各種 margin 等都能調整，非常方便好用。

唯一要注意的是，如果採用支援 RWD 的元件（像是這邊用的 `ResponsiveCalendar`），記得要給定這個 Component 的 Parent component 固定的高度，否則 nivo 的元件偵測到高度為零時，就不會 render 了，所以記得要多加個有高度的 Wrapper 在 nivo 的元件上。

其他的實作細節基本上就是 React 而已，真的有興趣~~（想看看 code 有多鳥）~~可以到 [github](https://github.com/ArvinH/YearEndReview-2018/) 瞧瞧。

## 最後分享一下我的年終檢討

我記錄在 Google Calendar 上的 task，基本上分為六類：

* English - 包含上英文線上課程、背單字、唸英文等
* Entertainment - 所有娛樂事項
* Sick/Rehabilitation - 生病、做物理復健
* Exercise - 運動
* Training - 一切我認為跟加強我自身實力有關的事，有可能是看文章、寫 code 或是刷題
* Work - 上班

![waffle](/img/arvinh/nivo-waffle-explain.png)

Waffle 圖的數據是我將每個月的分類 task 加總後做平均所算出的比例。

而從 Waffle 圖可以很明顯看到，2018 年我對於 `Training` 與 `English` 這兩部分的分類，加重了不少。回想過去半年，很急切的想要有所突破，雖然我不覺得有達到我心中的努力程度，但以結果論來說，這些多出的 `Training` 或許真的幫助了我，讓我在今年達成了人生中一個小小的目標，有了~~逃離鬼島~~前往海外的機會。

若是從 Calendar 圖與折線圖來做兩個年度的對比，感受會更深一點：

![Review](/img/arvinh/review-explain.gif)

首先，去年做完 year end review 後，我就發現假日也在工作的時間有點太多，除了有些時候的確是 project 較為忙碌，但大多是自己平日上班專注力沒有好好發揮。

因此今年開始，我調整作息，除了早上運動外，也盡量在公司時保持專注，降低與社群媒體的接觸，從圖表結果來看，成效挺好的，明顯看到 2018 年的假日在 Work 這個分類，是比較空的。

另外，`Training` 與 `English` 分類的差異從 Calendar 圖表非常明顯啊！

2018 我幾乎每天都有抽出一點時間學習工作外的事，持續練英文的時間也變多了！

# 結論

在製作第一年的 Year End Review 時，只是覺得好玩，可以把紀錄一年的資料做個整理跟視覺化呈現，但到了第二年，有了對照後，好像真的可以從中看出自己一年來的成長，也在整理資料的過程中，回顧了這一年所做的事情，並反省自己的缺失。雖然整體上來說，還是覺得自己浪費了許多時間，但套句李笑來的話：『所有對於現狀的解脫，最終都只能靠積累來實現。』從現在開始繼續努力、繼續積累，活在未來！

## 資料來源

1. [nivo](https://nivo.rocks)
2. [一起用 Google Calendar 與 D3.js 進行年終回顧吧！](https://blog.techbridge.cc/2017/12/12/d3v4-calendar-yearendreview/)

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化