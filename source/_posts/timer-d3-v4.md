---
title: 詳解簡易 Timer 來學習 D3
date: 2016-10-22 16:46:48
tags: d3, timer, pomodoro
author: arvinh
---

**The Pomodoro Technique** 是一個簡易又廣為人知的時間管理方法，其主軸在於將工作時段切割成多個**蕃茄時間**，每個蕃茄時間包含：

1. 25 分鐘的執行時間
2. 5 分鐘的休息時間

[蕃茄工作法 wiki](https://zh.wikipedia.org/wiki/%E7%95%AA%E8%8C%84%E5%B7%A5%E4%BD%9C%E6%B3%95)
詳細內容不管是網路上，或是書籍都有許多描述，有興趣深入了解的可以去看看。

今天重點不在番茄工作法。

重點在，番茄工作法中不可或缺的工具 - 計時器！

我用過幾款 Chrome extension 的計時器，但剩餘時間都是用數字來顯示，而我想要的是用圖像的方式來呈現，這樣我的大腦可以省略掉辨識數字的時間（是會花你多少時間... XD），也因為沒有了數字，不知道確切剩餘時間，不會看剩一分鐘就因為緊張而分心（至少對我而言啦~）。

所謂圖像的呈現方式就像下面的成品，輸入完你想要的倒數時間後，按下enter，就會看到倒數的區塊以及倒數秒數。

<p data-height="322" data-theme-id="dark" data-slug-hash="RGJOov" data-default-tab="js,result" data-user="arvin0731" data-embed-version="2" class="codepen">See the Pen <a href="http://codepen.io/arvin0731/pen/RGJOov/">Timer-d3-v4</a> by Arvin (<a href="http://codepen.io/arvin0731">@arvin0731</a>) on <a href="http://codepen.io">CodePen</a>.</p>
<script async src="//assets.codepen.io/assets/embed/ei.js"></script>
[Inspired by @ericbullington](https://www.ericbullington.com/blog/2012/10/27/d3-oclock/)

一開始本來想說看能不能直接用 html、css 把圖畫出來，但要計算弧度畫出扇型感覺有點麻煩，好在我們有 D3.js 這個方便的工具！除了拿來資料視覺化外，d3.js 提供的許多 lib 都很適合拿來單純作畫。並且，實作計時器的難度並不高，非常適合初學者來學習 D3.js，又比畫出長條圖等來得有成就感！（或是像我ㄧ樣很久沒碰 d3，又想稍稍複習一下的人也可以看看！）

因此，今天就來說明如何利用 D3.js 來繪製 Time timer！並在最後解釋一點 D3 data join 的概念。

## Make a plan & Prepare your data

在打開你的編輯器開始 coding 之前，讓我們先停下來想一下，繪製計時器你大概會需要哪些東西。

1. 你需要一個圓圈代表時鐘。

2. 你需要一個秒針，讓你知道時間不斷在跑。

3. 你需要一個會隨著時間流逝而縮小面積的扇行區塊。

4. 你需要一個輸入筐，讓使用者輸入想要倒數的時間。

先不管圓圈、秒針與扇形該怎麼畫，D3.js 比較不一樣的地方在於它是 data-driven，所以我們需要先把資料給準備好，而繪製計時器所需要的資料就是**時間**。

在我們的 case 裡面，**時間**是使用者輸入的數字，我們要將其 parse 為適合的資料格式：

```js
const timeData = (minutes, sec) => {
  return [
    {
      "unit": "seconds",
      "numeric": sec
    }, {
      "unit": "minutes",
      "numeric": minutes
    }
  ];
};
```

此外，一般繪製 D3.js 都是一次性的，也就是讀入資料後，根據該次讀入的資料繪製圖形。我們需要的則是類似 streaming data 的行為，不間斷的傳入目前時間，讓 D3 幫我們繪製出對應的圖形。作法有很多種，這邊採用最簡單的做法：`setInterval`

```js
let timer;
const startTimer = (e) => {
  // Main program
    if (isNaN(e.target.valueAsNumber)) {
      clearInterval(timer);
      clockGroup.selectAll(".clockhand").remove();
      return;
    };
    clearInterval(timer);
    let data;
    let timeAsSec = e.target.valueAsNumber * 60;
    let sec = 0;
    timer = setInterval(() => {
      data = timeData(timeAsSec / 60, sec);
      sec = sec + 1;
      timeAsSec = timeAsSec - 1;
      // render() 為繪製 d3 的函式
      render(data);
      if (timeAsSec === 0) {
        clearInterval(timer);
      }
    }, 1000);
};

let timeInput = document.querySelector('#time');
timeInput.addEventListener("change", startTimer);
```

`startTimer` 是綁定在 input field 的 EventListener，我們根據使用者輸入的數字乘上 60 轉換為需要執行的總秒數(`timeAsSec`)。目的是為了能夠適時（倒數結束）的跳出 `setInterval()`。

在 `setInterval()` 中，我們每秒執行一次，遞減 `timeAsSec`，同時遞增 `sec` 用以讓秒針轉動。

將 `timeAsSec` 與 `sec` 傳入剛剛的 `timeData()` 產生 D3 需要的資料格式。這邊我們直接將 `timeAsSec` 除以 60，結果會是分數，因此我們的分鐘區塊會隨著秒數的增加而順順的減少，若你想要有明顯跳動，可以轉為整數後再丟入 `timeData`（轉為整數的話，只有每 60 秒數值會變動一次，才看得到差別）。

總結一下整個流程：

每秒執行一次，從 `timeData` 中產生新的資料傳入 `render()` 去繪製圖形，直到 `timeAsSec` 遞減為零。

## Render Setting

資料都設定好後，接著準備繪製圖形。

```js
// value setting
let width = 300;
let height = 200;
let offSetX = 150;
let offSetY = 100;
let pi = Math.PI;

let scaleSecs = d3.scaleLinear().domain([0, 59 + 999 / 1000]).range([0, 2 * pi]);
let scaleMins = d3.scaleLinear().domain([0, 59 + 59 / 60]).range([0, 2 * pi]);

let vis = d3.selectAll(".chart")
  .append("svg:svg")
  .attr("width", width)
  .attr("height", height);

let clockGroup = vis.append("svg:g")
  .attr("transform", "translate(" + offSetX + "," + offSetY + ")");

clockGroup.append("svg:circle")
  .attr("r", 80).attr("fill", "none")
  .attr("class", "clock outercircle")
  .attr("stroke", "black")
  .attr("stroke-width", 2);

clockGroup.append("svg:circle")
  .attr("r", 4)
  .attr("fill", "black")
  .attr("class", "clock innercircle");
```

`line 2 ~ line 6` 先定義一些常數，這邊就隨意設定。

<!-- 講解 d3 scale -->
`line 8 ~ line 9` 設定 `Mins` 與 `Secs` 的 scale。scale 是什麼？基本上你可以把 scale 想像成是能夠幫你把 **資料映射** 到適合你想繪製圖形的維度上 的一種方法。

```js
let scaleSecs = d3.scaleLinear().domain([0, 59 + 999 / 1000]).range([0, 2 * pi]);
let scaleMins = d3.scaleLinear().domain([0, 59 + 59 / 60]).range([0, 2 * pi]);
```

透過 `domain` 與 `range` 來分別給予 **資料的輸入範圍** 以及 **資料的輸出範圍**，以今天的 case 來說，我們要將時間資料轉化為圓形的角度，因此給予 `domain` 的輸入範圍為 0 ~ 59。

`range` 負責輸出的範圍，我們要在圓形的時鐘上顯示，因此是 0 ~ 2 * pi。

到這邊你可能會想，degree 0 跟 degree 360 不是一樣嗎？這樣 0 秒跟 59 秒會映射到同一個點？

好問題！所以先前的 seconds 與 minutes 基本上可以共用，但如果想要區別 0 秒與 59 秒，我們需要把範圍多加上小數點的範圍，以分鐘來說，分成 60 等份，因此就是 [0, 59 + 59/60]，秒則是分為 1000 毫秒，也就是 [0, 59 + 999/1000]。

`line 11 ~ line 14` 由於 d3 的 method 是 Chainable 的，因此可利用 `d3.selectAll()` 選取你想要 mount 上去的 DOM 元素，並接著 append 上 `svg` 元素，以及設定 `width` 與 `height` 的屬性值。

```js
let vis = d3.selectAll(".chart")
  .append("svg:svg")
  .attr("width", width)
  .attr("height", height);
```

`line 16 ~ line 28` 採用同樣方式，在 `vis` 這個 svg 元素下，再 append 上一個 svg group，並加入兩個 circle。

第一個 circle 是時鐘的外圍，第二個 circle 作為時鐘中心點。

## Render Timer

終於可以開始畫圖！

```js
// render clock
const render = (data) => {

  let minuteArc, secondArc;

  clockGroup.selectAll(".clockhand").remove();
  
  secondArc = d3.arc()
    .innerRadius(0)
    .outerRadius(70)
    .startAngle((d) => {
    return scaleSecs(d.numeric);
  })
    .endAngle((d) => {
    return scaleSecs(d.numeric);
  });
  
  minuteArc = d3.arc()
    .innerRadius(0)
    .outerRadius(80)
    .startAngle((d) => {
    return 0;
  })
    .endAngle((d) => {
    return scaleMins(d.numeric);
  });

  clockGroup.selectAll(".clockhand")
    .data(data)
    .enter()
    .append("svg:path")
    .attr("d", (d) => {
      if (d.unit === "seconds") {
        return secondArc(d);
      } else if (d.unit === "minutes") {
        return minuteArc(d);
      }
    })
    .attr("class", "clockhand")
    .attr("stroke", (d) => {
      if (d.unit === "seconds") {
        return "black";
      } else if (d.unit === "minutes") {
        return "blue";
      }
    })
    .attr("stroke-width", (d) => {
      if (d.unit === "seconds") {
        return 2;
      } else if (d.unit === "minutes") {
        return 3;
      }
    })
    .attr("fill", "red")
    .attr("opacity", "0.8");
};
```

`line 8 ~ line 26` 定義了兩個 method: secondArc 和 minuteArc，分別用來將傳入的資料依據其所設定的 `innerRadius` 、 `outerRadius` 、 `startAngle` 與 `endAngle` 來繪製成弧形。 `d3.arc()` 在 d3 v4 中屬於 [d3-shape](https://github.com/d3/d3-shape) 的一環。

![d3-arc](/img/arvinh/d3-arc.png)

在 `endAngle()` 中，我們將接收到的參數（也就是傳入的資料）放入我們先前定義好的 `scaleSecs()` 與 `scaleMins()` 中，讓 `d3.arc()` 能接收到我們映射過後的值，進而繪出正確的 scale。

我們讓秒針的 startAngle 與 endAngle 設定為一樣，以呈現 針 的狀態。而 分鐘區塊 則讓其 startAngle 固定為 0，只在 endAngle 中傳入每次更新的時間，如此一來，每次時間更新時，區塊會隨之改變大小！

此外，這邊的innerRadius 都設為 0，讓其等同於圓中心，才能產生扇形。

`line 28 ~ line 56` ： 還記得最一開始我們有將最外層的 svg 指定給變數 clockGroup 嗎？
```js
let clockGroup = vis.append("svg:g")
  .attr("transform", "translate(" + offSetX + "," + offSetY + ")");
  
  ...
  ..
  .

  clockGroup.selectAll(".clockhand")
    .data(data)
    .enter()
    .append("svg:path")
```

在最後一段裡面，利用 `clockGroup.selectAll(".clockhand")` 我們把資料 import 到擁有 `.clockhand` 這個 classname 的 `svg:path` 底下，透過設置 `svg:path` 的 attribute `d`來繪製 秒針 與 分鍾區塊（利用先前提到的 `secondArc()` 與 `minuteArc()` ）。

`attr()` 函式可以接受 `callback`，所以我們可以根據資料的不同來設定不同的 style，讓秒針與分鍾區塊做個分別。

不知道大家看到這邊有沒有覺得奇怪，從最一開始到現在從來沒有設置過 `.clockhand` 這個 class 的 DOM 元素，為什麼我們這邊可以直接 `selectAll` 還塞入資料並 append svg 呢？

原因是在於，d3 的 `selectAll()`, `select()` 如果找不到所指定的元素時，會回傳一個空的 NodeList，所以我們可以拿這個空 NodeList 去進行操作。但要記得在你 append 上去的 DOM 元素中加上你想指定的 classname，否則每次執行這個函數的時候，他就會重新 create 一個空的 NodeList。

這會造成什麼問題呢？以我們這邊實作的例子來說，在我們的 `render()` 一開始的地方，我們有個

```js
clockGroup.selectAll(".clockhand").remove();
```

這個 `remove()` 的動作會在我們每次執行 `render()` 的時候（也就是 setInterval 每次執行時）幫我們把原先的 `.cockhand` DOM 拿掉，重新繪製正確的 秒針 與 分鐘區塊。

若我們在 append `svg:path` 的時候設定他的 `attr` 為 `clockhand`，則會造成這邊抓不到 `.clockhand` 而無法移除原有的 DOM（秒針與分鐘區塊）。

像是這樣：

![Wrong classname timer](/img/arvinh/timer-wrong.png "Wrong classname timer")

因此記得要加上對應的 class name。

另外一個要注意的部分是，`clockGroup.selectAll(".clockhand").remove();` 他雖然會移除掉 DOM 元素，但是 `clockGroup.selectAll(".clockhand")` 還是會留存有對原本該 DOM 元素的 reference，有可能會造成 memory leak，若想避免記得額外處理這部分。

## 小總結 與 附錄

到這邊為止我們就已經實作出一個簡易的 Timer，希望能讓大家對 d3 繪圖有一點概念，至於再進階一點的，像是互動與動畫的部分，時間實在不多，只能下次再努力找個範例來寫寫。

### Data join

不過我想稍微介紹一點關於 D3 裡面很重要的 Data join，對之後實作更複雜的圖形時有絕對幫助。

先給點資源：

D3 作者關於 data join 的說明（必看）：
[Thinking with joins](https://bost.ocks.org/mike/join/)

UNC 的 data join 視覺化教學（也是必看, 上面那篇太長看不下去直接看這邊可以秒懂）：
[Dat joins in D3.js](https://ils.unc.edu/~gotz/D3joins/)

在我們先前的範例裡面，有用到 `clockGroup.selectAll(".clockhand").data().enter()`，意思是將資料跟 DOM 連接，並將進入 `enter` 選取狀態 的資料傳遞到後面的 method （像是 `attr()`）中。

嗯？！ `enter()` 不是指把資料 enter (輸入) 進去的意思喔！

是，也不是。

![Data joins](/img/arvinh/d3-join.png "d3 data join")

在 D3 中，資料在繫結到 DOM 上時，會分別進入三種選取狀態：enter, update 與 exit。

<span style="font-size: 14px;color: gray;">[Note] selection 代表的是 `selectAll()` 或 `select()` 所選取到的 d3 node object。</span>

**update selection**: 當你 selection 中的資料與新的 `.data()`傳遞進來的資料有重疊時，新的資料會進入 update selection 狀態，可以進行資料更新。

**exit selection**: 當你原本 selection 中的資料與新的資料沒有匹配時，那些舊有的資料會進入 exit selection 狀態，可以透過呼叫 `.exit().remove()` 將之從 selection 中移除。

**enter selection**: 與 update 相反，在原本 selection 中找不到對應的新資料就會進入 enter selection。

所以，當你想要在原本的圖形內更新資料時，可以直接呼叫 `selectAll('.oldData').data(newData)`，該進入 update 狀態的資料就會取代掉舊的，接著再呼叫 `selectAll().exit().remove()` 移除掉其他不再 newData 內的資料，最後呼叫 `selectAll().enter().append()` 來把不在舊資料中，但在 newData 中的資料新增上去。

強力推薦搭配參考 [Dat joins in D3.js](https://ils.unc.edu/~gotz/D3joins/)，裡面的範例非常清楚，有機會我會翻成中文版的。

了解資料在 d3 中的狀態後，對於後續設計互動效果與資料更新等等的操作會更得心應手！


## 參考資料
1. [Thinking with joins](https://bost.ocks.org/mike/join/)
2. [Dat joins in D3.js](https://ils.unc.edu/~gotz/D3joins/)
3. [d3-oclock](https://www.ericbullington.com/blog/2012/10/27/d3-oclock/)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化
