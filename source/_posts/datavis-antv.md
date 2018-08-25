---
title: 使用 AntV 製作資料圖表-台灣老年人口與長照機構供需比
date: 2018-08-25 20:20:58
tags:
  - data visualization
  - AntV
  - G2
  - RWD
  - javascript
---

# 前言

隨著年齡增長，多少開始會遇到家人或親戚需要長期照護，入住療養院或醫院的狀況，接者就會發現許多照護中心可是一位難求，院中的照服人員或是醫護人員也得以一擋百，讓我很想知道目前台灣整體來說，老年人口、長照機構與照服人員的比例失衡有多嚴重。而剛好在前陣子 [@huli](https://medium.com/@hulitw) 大大介紹了[Ant design](https://blog.techbridge.cc/2018/04/28/antd-and-admin-website/)，讓我再次注意到同樣為螞蟻金服出品的 [AntV](https://antv.alipay.com/zh-cn/index.html)，稍微研究之下發現它使用起來非常簡單快速，並且一樣有 React、Angular 與 Vue 的版本。所以今天這篇文章想藉由實作台灣老年人口與長照機構供需比的資料圖表，順便介紹 AntV 這套資料視覺化的套件。

對了，報導者有發表過一系列專輯 - [長照機構裡的大象——10萬老人被照顧的真相](https://www.twreporter.org/topics/nursing-home-truth)，說明台灣長照產業的現況，如果懶得看我長篇大論的技術文章，拜託至少在離開前去看一下報導者的專輯，這樣我也算功德一件，讓更多人知道長照產業的重要性，或許也就有那麼一些人有辦法解決這個鮮少被提起的問題。

先看個成果：

<p data-height="379" data-theme-id="29194" data-slug-hash="OZxNbO" data-default-tab="result" data-user="arvin0731" data-embed-version="2" data-pen-title="總體台灣老年人口與長照機構供需比 - AntV - demo" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/OZxNbO/">總體台灣老年人口與長照機構供需比 - AntV - demo</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

<!--
解釋範例內容
-->
我依照 [Wiki 上的台灣地理區劃分](https://zh.wikipedia.org/wiki/%E5%8F%B0%E6%B9%BE%E5%9C%B0%E7%90%86%E5%8C%BA%E5%88%92)將資料北、中、南、東與外島五個區塊，並加上兩個按鈕，可以把資料量較少的外島去除，用來展示 G2 在處理資料切換時的順暢感。

看得出來其實就是個非常簡單用 Excel 也畫得出來的分組長條圖，但實際上你會發現，透過 AntV 製作完全不會比用 Excel 麻煩，還支援 RWD 與資料的切換動畫！真的非常適合我這次想要快速拉出一個比較圖表的狀況！

# AntV

![AntV 官網](/img/arvinh/antv-cover.png)

AntV 其實包含了三個不同應用情境的套件：G2、G6、F2。

1. G2: 包含各種圖表元素的集成，大多數的應用場景都可以從 G2 中找到對應合適的元件。
2. G6: 主要是針對流程圖與關聯性分析的圖表元件，甚至能利用 G2 繪製資料庫的 ER Diagram 與時序圖。
3. F2: 針對 Mobile 的使用情境來特別加強圖表在 performance 上的表現，主要繪製在 `canvas` 上。

## 今天我們使用 G2 來開發

這幾套由螞蟻金服開發的視覺化套件，除了遵照 Antd 的設計語言外，針對圖表的設計製作上，深受 [The Grammar of Graphics](https://www.amazon.com/Grammar-Graphics-Statistics-Computing/dp/0387245448) 這本巨作的影響，也是 G2 的名稱的由來。

> G2 的強大是由其背後的一套圖形語法所支撐的，它基於《The Grammar of Graphics》(Leland Wilkinson 著)一書，是一套用來描述所有統計圖形深層特性的語法規則，該語法回答了『什麼是統計圖形』這一問題，以自底向上的方式組織最基本的元素形成更高級的元素。由此，G2 所建構出的圖表是由一系列獨立的圖形語法元素組合而成的 -- [AntV G2 官網](https://antv.alipay.com/zh-cn/g2/3.x/tutorial/the-grammar-of-graphics.html)

我並沒有看過那本書，也沒有仔細研究 G2 的原始碼，但這敘述聽起來就超厲害的，整個工具是建構在一套完整的理論基礎上頭。

在 G2 的世界中，並沒有明確定義一般的圖表類型，像是長條圖、折線圖等等，是依照一系列的圖形語法元素組合成的結果來決定其類型。

所謂的圖形語法元素大概就是包含：

* **DataSet 資料集操作**
* **Scale 度量**
* **Geom 幾何標記（point, line, area, shape, etc）**
* **Attr 圖形屬性**
* **Coord 座標系**
* **Axis 座標軸**
* **Legend 圖例**
* **Tooltip 提示訊息**
* **Guide 輔助元素**
* **Facet 分面 （將一份資料按照某個維度分隔成若干子集）**
* **Label 標籤**
* **Theme 主題**
* **Event 圖表事件**

在[官方文件上](https://antv.alipay.com/zh-cn/g2/3.x/tutorial/index.html)上針對不同的元素都有非常詳細的說明與範例，而且都有中文文件（雖然是簡體...），是 AntV 的絕大優點之一。

透過操作這些不同元素的組合，可以很容易的切換圖表類型，以 **Coord** 為例：

`chart.coord('coordType'[, cfg]);`

在同樣的資料集上，透過上述的方式來轉換座標軸，馬上就可以從層疊長條圖切換為圓餅圖：

![Different coord on same chart](/img/arvinh/g2_coord.png) [圖片來源](https://antv.alipay.com/zh-cn/g2/3.x/tutorial/coord.html)

或是透過自定義 `Shape`，快速將一般的長條圖，改變成三角形的形狀：

<p data-height="458" data-theme-id="29194" data-slug-hash="deVMmN" data-default-tab="js,result" data-user="arvin0731" data-embed-version="2" data-pen-title="AntV-G2-demo" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/deVMmN/">AntV-G2-demo</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

由於這樣的設計理念與基礎，G2 比起其他一樣提供較為高階視覺化語法的套件來說，彈性大了不少。

## 開始進行實作，視覺化總要有個方向，走，我們先從資料搜集開始！

首先，要想知道老年人口以及長照機構的比例，我們需要找到依照年齡分組的人口統計資料，這可以從[內政部的人口資料庫](https://www.ris.gov.tw/346)中找到「年底人口數按性別及五歲年齡組分」的資料表，從民國 35 年到 106 年都有，算是蠻齊全的。我們選取 85 ~ 100+ 以上的資料當作老年人口。

2017 年的部分計算出來後大約是：368,757 人

![年底人口數按性別及五歲年齡組分](/img/arvinh/age_population.png)

接著，到[衛福部統計處](https://dep.mohw.gov.tw/DOS/lp-3550-113-xCat-T02.html)找尋長期照顧機構的相關資料。

可以找到 2017 年長照與安養機構總數的可供進住人數，大約 57,147 人：

![長照與安養機構總數的可供進住人數](/img/arvinh/number_of_nursy_building.png)

以及 2017 年老人長期照顧、安養機構工作人員人數，大約 19,064 人（只計算護理人員與照顧服務人員）：

![老人長期照顧、安養機構工作人員人數](/img/arvinh/number_of_workers.png)

上述的資料來源都有個別縣市的統計資料，上面數字是我將各縣市加總的結果，可能多少會有誤差，大家別介意啊...整體比例應該不會差太多。

## 接著使用 G2 來進行視覺化

要使用 G2 很簡單，只要在 HTML 中載入 `<script src="https://gw.alipayobjects.com/os/antv/assets/g2/3.0.9/g2.min.js"></script>` 即可。

接著將剛剛的資料定義好：

```js
const SimpleAll = [
  {name: '可供進住人數', value: 57147},
  {name: '長照、養護人員', value: 19064},
  {name: '老年人口數(85~100+)', value: 368757}
];
```

我們就可以透過 `G2.Chart` 來定義圖表：

```js
const chart = new G2.Chart({
  container: 'mountNode',
  forceFit: true,
  height: window.innerHeight
});
```

我們 new 一個 `G2.Chart` 的 instance，並且同時傳入三個參數給予建構：

* container: 指定你的圖表要掛載在 DOM 中的哪個位置，對應於 HTML 中元素的 id 值。
* forceFit: 超方便的參數，只要設定為 true，你的圖表就能 Responsive，因此也不用再設定寬度。
* height: 可以額外定義需要的圖表高度。

再來我們把資料與圖表做綁定：

```js
  chart.source(SimpleAll);
```

接著便能開始設定我們的圖表長相：

```js
chart
  .interval()
  .position('name*value')
  .color('name')
  .adjust([{
    type: 'dodge',
    marginRatio: 1 / 5
  }]);
```

先前說過，G2 中沒有區分圖表類型，利用的是各種不同的**幾何標記**來組成圖表，如果要製作長條圖，需要用到的就是 `ìnterval()` 這個幾何標記，G2 還支援：`point()`, `path()`, `line()`, `area()`, `polygon()`, `edge()`, `schema()`, `heatmap()` 這幾種類型，[官網有更詳細的介紹](https://antv.alipay.com/zh-cn/g2/3.x/tutorial/geom.html)

宣告我們需要的幾何圖形後，接著就會想知道我們要怎麼將資料映射到對的位置，並且給予不同的顏色區別。

而這一切在 G2 中都非常的簡單直覺。

透過 `position('name*value')` 這個 API，我們指定資料欄位的 `name` 要對應到圖表座標軸上的 `x`，而 `value` 對應到 `y`，到這邊為止，我們就能畫出一個擁有完整資訊的圖表：

<img src="/img/arvinh/g2-simple-bar.png" width="500" height="300">

當然這樣還不夠，至少也要用顏色來區別一下不同的資料類別。利用 `color(name)` 來告訴 G2，我們要根據資料的 `name` 欄位，用不同的顏色來區分。

<img src="/img/arvinh/g2-simple-bar-color.png" width="500" height="300">

G2 的 API 通常都能接收一個以上的參數，以 color 為例，除了直接傳入顏色要對應的資料欄位外，也可以直接輸入某個顏色，讓他 apply 到整個圖表；或是傳入 callback，在 callback 中根據欄位數值做邏輯上的著色動作。

```js
// 可以參考官網 API Doc https://antv.alipay.com/zh-cn/g2/3.x/api/geom.html#_color

chart.point().position('x*y').color('z'); // 使用默認的顏色
chart.point().position('x*y').color('z', [ 'red', 'blue' ]); // 使用傳入的指定顏色
chart.point().position('x*y').color('z', (value) => {
  if(value === 1) {
    return 'red'
  }

  return 'blue';
});
```

### 接著我們還可以加點東西讓圖表活潑一點。

```js
chart.point()
  .position(['name', 'value'])
  .size(50)
  .shape('name', function (name) {
    return ['image', imageMap[name]];
  });
```

我們在原本的 `interval` 幾何圖形上加上另一個幾何圖形 `point`，位置一樣將 `(name, value)` 映射到 `(x, y)`。

相信眼尖的你會發現，`position` 的參數除了一開始的 `(name*value)` 外，也可以傳遞 array 形式。

接著設定 size 為 50；在幾何圖形 `point` 中，size 代表原點的半徑，如果是 `interval` 則代表柱體寬度，`line` 則是線段寬度。

還有個特殊的 `shape()` 函式，他讓我們可以指定特定資料欄位（在這邊我們用 `name`），將其轉換成不同的型態，像是這邊我們就把單純的 `point` mapping 成圖片，而這一切只要透過一個 callback 函數，回傳對應的結果即可！

<p data-height="518" data-theme-id="29194" data-slug-hash="jxZGPV" data-default-tab="js,result" data-user="arvin0731" data-embed-version="2" data-pen-title="2017 台灣總體老年人口與長照機構供需比 - AntV - demo" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/jxZGPV/">2017 台灣總體老年人口與長照機構供需比 - AntV - demo</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

短短幾行 code 就作出一個比一般 Excel 還要特別一些的圖表，真的蠻方便使用的對吧！

## 但只看全台灣總體的資料好像不太夠啊，能不能看一下各縣市的呢？

當然可以。

只是每個縣市如果都要放三根長方體好像太密集了點，不是很適合，所以如同最前面的成品圖，將資料北、中、南、東與外島五個區塊，這樣圖表會比較清晰一點。

```js
const data = [ { name: '可供進住人數',
    '北部': 25499,
    '中部': 11027,
    '南部': 18684,
    '東部': 1609,
    '外島': 328 },
  { name: '長照、養護人員',
    '北部': 8994,
    '中部': 3518,
    '南部': 5997,
    '東部': 482,
    '外島': 73 },
  { name: '老年人口數(85~100+)',
    '北部': 162443,
    '中部': 92071,
    '南部': 101846,
    '東部': 10197,
    '外島': 2200 } ];
```

### DataSet

要想在同一圖表中呈現多組長條圖，我們需引進 G2 中 `DataSet` 的這個概念。

![DataSet 架構圖（取自官網）](/img/arvinh/antv-g2-data-set-structure.svg)
[官網介紹](https://antv.alipay.com/zh-cn/g2/3.x/tutorial/data-set.html#)

這張圖看似複雜，但其實很清楚的介紹了 `DataSet` 在使用 G2 製圖的過程中所扮演的角色：操作資料。

資料可以經由 `Connector` 傳入 `DataSet` 中，接著利用 `Transforms` 針對數據做處理（排序、統計、資料補齊），最後渲染到 `DataView` 中。

突然看到一堆名詞感覺有點嚇人？別擔心，待會的範例會慢慢講解，只要先把 `DataSet` 想成是存放資料的資料集，而 `DataView` 就是我們要繪製出來的資料視圖。

另外，一份 `DataSet` 可以連接多個 `DataView`，並透過更動其中的共用 `State` 來進行連動變更，在我們的範例中沒有使用到，但可以參考[官網的清楚範例](https://antv.alipay.com/zh-cn/g2/3.x/tutorial/data-set.html#_%E5%9B%BE%E8%A1%A8%E8%81%94%E5%8A%A8%E7%A4%BA%E4%BE%8B)

![官網範例示圖](/img/arvinh/antv2-g2-dataset-state.gif)

### 回到我們的範例來

跟剛剛的範例不同，我們在進行資料綁定前（`chart.source(data)`），需要先對數據做一些操作，所以要利用 `DataSet` 與 `transform` 兩個 API：

```js
const ds = new DataSet();
const dv = ds.createView().source(data);
dv.transform({
  type: 'fold',
  fields: [ '北部', '中部', '南部', '東部', '外島'], // 展開資料
  key: '區域', // key, 設置新的 key/value 值，來對應新數據的含義。
  value: '區域人數', // value
  retains: [ 'name' ] // 想要保留在 transform 後的資料欄位
});

chart.source(dv);
```

透過 `new DataSet()` 與 `createView()` 創建出一個擁有 **資料集 <-> 資料狀態連接 <-> 資料視圖** 關聯性的物件 `dv`，接著利用 `transform()` 函數針對資料進行處理。

我們的資料是根據 `可供進住人數'`、`長照、養護人員` 與 `老年人口數`來分類，但我們實際上希望的是能夠先依照區域劃分，在每個區域中再分別用這三種類別來比較資料。

因此在傳入 `transform` 的 option 中，我們選定 `type` 為 `fold`，其意義為：**以指定字段集作為 Key，展開數據。**並且設置新的 key/value 值，來對應新數據的含義。

直接拿我們的資料來作為例子，比較一下前後結果就會很清楚了：


```js
// 原始資料
const data = [{
  name: '可供進住人數', '北部': 25499,'中部': 11027,'南部': 18684,'東部': 1609,'外島': 328
}, {
  name: '長照、養護人員', '北部': 8994, '中部': 3518, '南部': 5997, '東部': 482, '外島': 73
}, ...];

// 經過 transform 後

const dataBeenTransformed = [{
  key: '北部', value: 25499, name: '可供進住人數',
}, {
  key: '北部', value: 8994, name: '長照、養護人員',
}, {
  key: '中部', value: 11027, name: '可供進住人數',
}, {
  key: '中部', value: 3518, name: '長照、養護人員',
} ...];
// 基本上，transform 後，原有的資料欄位會不見，這邊的 `name` 還保留是因為我有加入 `retains: [ 'name' ]` 這個選項。
```

如此一來，我們在繪製圖表的時候，他就會依照目前的 Key, 將相同的 Key 所對應的值 Group 在一起，讓`北部`這個 Key 對應的三個欄位數值（可供進住人數、長照人員數、老年人口數）一起在同個分類（北部）中顯示。

不過，還沒有結束。

將資料分組後，依照先前的繪製方式，會畫出下面這樣的結果：

```js
chart.interval().position('區域*區域人數').color('name');
```

<img src="/img/arvinh/antv2-g2-multiple-set.png" width="500" height="300">

雖然分組了，但資料重疊在一起了...囧

這是因為每個分組中的每個數據都套用到了同樣的 position 設定（區域*區域人數），我們必須要調整一下！

透過 `adjust()` 函數可以方便做到：

```js
.adjust([{
  type: 'dodge',
  marginRatio: 1 / 5
}])
```

設定 type 為 `dodge`，代表我們要調整的是分組數據，然後給予 margin 比例為 1/5。

<p data-height="297" data-theme-id="29194" data-slug-hash="JvZWQE" data-default-tab="result" data-user="arvin0731" data-embed-version="2" data-pen-title="2017 台灣老年人口與長照機構供需比 - AntV - demo1" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/JvZWQE/">2017 台灣老年人口與長照機構供需比 - AntV - demo1</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

看起來就正常多了！

關於 `transform` 的運用，可以參考官方詳解 - [Transform](https://antv.alipay.com/zh-cn/g2/3.x/api/transform.html#)
關於 `adjust` 的運用，目前[支援四種 type](https://antv.alipay.com/zh-cn/g2/3.x/api/geom.html#_adjust)：`stack`, `dodge`, `jitter`, `symmetric`。

神奇的是，這些不同類型的差異，在擁有豐富文檔的官方網站竟然找不到介紹，所以我其實也不太懂。重點是，不同的資料型態會有各自適合的 type，大家在製作時要記得都嘗試看看效果。

### 看起來蠻完整的了，但圖表沒辦法動態變化怎麼可以！

沒錯，我也覺得不可以。

還好 AntV G2 讓我們輕鬆利用 `chart.changeData()` 就能同時更新 `DataSet` 與 `DataView`。更棒的是，除了提供 RWD 設計外，資料轉換過程中的動畫他也幫你照顧好好的。

除此之外，我們也能夠直接操作原有的 `DataSet`，像開頭的範例一樣，點選 button 時，我們透過 `chart.filter` 來過濾原有資料，並且在最後呼叫 `chart.repaint()` 進行 `DataView` 的重繪（一定要重繪才會 trigger `DataSet` 與 `DataView` 間的連動）：

```js
const changeData = (status) => {
  chart.filter('區域', val => {
    if (status === 'all') {
      chart.scale('區域人數', {
        minLimit: 73
      });
      return true;
    }
    chart.scale('區域人數', {
      minLimit: 400
    });
    return val !== '外島';
  });
  chart.repaint();
};
```

如此一來就能完成開頭的範例啦！

<p data-height="379" data-theme-id="29194" data-slug-hash="OZxNbO" data-default-tab="result" data-user="arvin0731" data-embed-version="2" data-pen-title="總體台灣老年人口與長照機構供需比 - AntV - demo" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/OZxNbO/">總體台灣老年人口與長照機構供需比 - AntV - demo</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

## 結論

如同我在前言提及的[報導者專輯](https://www.twreporter.org/topics/nursing-home-truth)中所呈現的，台灣的長照產業正面臨很大的困境，只要家中有人有這種需求，相信都能有很深的體會。先前也曾有學長以長照產業為創業題目，可惜最終黯然收場，可見這問題真的有其嚴重與困難性存在。

身為前端開發者，或許目前能做的最大貢獻就是盡量製作出許多一目了然的資訊圖表、網頁專輯，讓更多的人知道問題的嚴重性，透過 AntV G2 這樣的工具，製作互動式圖表真的是越來越方便與容易，希望大家能多加貢獻，讓台灣社會往更好的方向走去！而小魯我能力不足分身乏術...先下台一鞠躬...

<!-- 資料來源 -->

## 資料來源

1. [AntV](https://antv.alipay.com/zh-cn/index.html)
2. [G2](https://antv.alipay.com/zh-cn/g2/3.x/tutorial/index.html)
3. [內政部戶政司人口資料庫](https://www.ris.gov.tw/346)
4. [行政院性別平等統計資料庫](https://www.gender.ey.gov.tw/gecdb/Stat_Statistics_DetailData.aspx?sn=%2BwEzRrF1jtJ82VtOVizhxw%3D%3D)
5. [衛福部統計處-長期照顧統計](https://dep.mohw.gov.tw/DOS/lp-3550-113-xCat-T02.html)
6. [長照機構裡的大象——10萬老人被照顧的真相](https://www.twreporter.org/topics/nursing-home-truth)
7. Icon made by Freepik from www.flaticon.com

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化