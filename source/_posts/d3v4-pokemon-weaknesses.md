---
title: 用 D3.js v4 看 Pokemon 屬性表
date: 2016-08-20 22:11:52
tags:
    - d3
    - pokemon
---

自從 Pokemon Go 在台灣可以玩後，勾起了我許多兒時回憶，因此除了跟著大家一起抓神奇寶貝以外（對我就是不想講寶可夢啊啊啊），我也稍微去追了一下最新版的神奇寶貝動畫，似乎是在打什麼卡洛斯聯盟，也出現了好奇怪的 Mega 進化，會讓神奇寶貝在戰鬥中轉屬性...

咳咳，等等，再講下去這整篇都是神奇寶貝了...

總之，看到會轉屬性這件事情就讓我想到，我小時候從來都沒有認真研究過哪種屬性剋哪種屬性，只知道基本的水剋火之類的，於是乎決定來好好研究一下，順便練習已經出來一陣子的 D3 v4，看看差異性在哪。

幸運的是，當我在搜尋 Pokemon 的 API 時，發現 http://filipekiss.github.io/pokemon-type-chart/ 這個人已經把我想做的做完了 XD 也做得不錯。不過是兩三年前的專案，用的是 D3 v3。雖然點子已經被做完了，但臨摹也是一種學習，所以我們就來把它 Migrate 到 D3 v4，順便看看有哪些值得注意的地方吧！

想直接看 code 的在這邊... [<span style="color: #c40522;">成果 與 程式碼</span>](http://bl.ocks.org/arvinh/b30ed888914a2794830ceb023c911a5b)

## 介紹

<img src="/img/arvinh/pokemontype.png" alt="pokemon type" style="width: 500px;"/>

這張圖乍看之下我原本以為是修改自 Chord Diagram，但其實是來自於 Cluster。想想也對，屬性間的關係的確類似於階層樹狀，也不需要有比例分佈對應。
( 從程式碼看來，原作者應該是修改自 https://bl.ocks.org/mbostock/7607999 )

使用方法很簡單，只要點擊某個屬性，就會列出該屬性對哪些屬性較為強勢 (Strong)、弱勢 (Weak) 或是 免疫 (Immune)，同時點擊兩種屬性的話，就會秀出擁有雙重屬性的結果為何。

## 解析

由於 D3 v4 的變動幅度很大，為了模組化，將很多 packages 都拆出來，替代以往使用 namespace 的方式，因此最單純的 Migration 方式就是直接重刻並設法 re-use 原來的 code。

首先，基本的 `index.html` 內定義好圖要畫在哪裡，並加上一個 reset button 來還原圖表狀態：

```html
  <div id="typeChart">
    <div id="graph"></div>
  </div>
  <button class="button reset"onclick="reset()">reset</button>
```

接著，我們需要先定義 `layout`，這邊使用 Cluster Layout，在原本的 D3 v3 版本中，使用的方式為：

```js v3.js
  var diameter = 750,
      radius = diameter / 2,
      innerRadius = radius - 120;
  var cluster = d3.layout.cluster()
      .size([360, innerRadius])
      .sort(null)
      .value(function(d) { return d.size; });
```
然而，v4 模組化後，原本的 namespace 都不需要了，因為實際上是個別存放在一個 lib 底下，以 Layout 來說 會放在 `d3-hierarchy`，而使用方式則變成直接呼叫 `d3.cluster()` 即可，原有的 `sort()`, `value()`等 method 也都移到 `node` 這個層級底下了（後面會在講到 node）：

```js v4.js
  // 定義圖形的基本設定值
  var diameter = 750,
      radius = diameter / 2,
      innerRadius = radius - 120;

  var cluster = d3.cluster()
      .size([360, innerRadius]);
```
寫好圖形的基本設定值後，先把我們已知的 svg 放上去吧！
我們先把剛剛定義好的 `diameter`, `radius` 的值 append 到最外層的 graph div 上，
接著先把待會會用到的四種 svg group 先記錄起來，分別有 immune (免疫)、weak (弱勢)、strong (強勢)、node (屬性)

```js weaknesses-graph.js
var svg = d3.select("#typeChart > #graph").append("svg")
    .attr("width", diameter)
    .attr("height", diameter)
    .append("g")
    .attr("transform", "translate(" + radius + "," + radius + ")");

var immune = svg.append("g").selectAll(".immune"),
    weak = svg.append("g").selectAll(".weak"),
    strong = svg.append("g").selectAll(".strong"),
    node = svg.append("g").selectAll(".node");
```

我蠻喜歡這樣的寫法，將與資料繪製較無關（相對較固定）的程式碼先寫好，接著再利用 `d3.json` 將資料讀入後去繪製。

```js Draw-graph
d3.json("types.json", function(error, classes) {
    var nodes = cluster(d3.hierarchy(packageHierarchy(classes))).children;
    var immunes = typeImmune(nodes);
    var strengths = typeStrong(nodes);
    var weaknesses = typeWeak(nodes);

    // draw path

    ......
    ....
    ..

    //Make the immune links
    function typeImmune(nodes) {
    var map = {},
        immunes = [];

    nodes.forEach(function(d) {
        map[d.data.name] = d;
    });

    nodes.forEach(function(d) {
        if (d.data.immunes) d.data.immunes.forEach(function(i) {
            immunes.push({
                source: map[d.data.name],
                target: map[i]
            });
        });
    });

        return immunes;
    }
    // 以下先省略
});
```
解釋一下上面這段程式碼，我們的資料存放在 types.json 中，利用 d3.json 將資料讀出後，
會先做兩件事情：

1. 將資料轉化成 hierarchy 格式，並初始化 cluster
2. 將產生的 nodes 轉化並分類成 immunes, strengths, weaknesses。

這邊的 `packageHierarchy` 主要是將 raw data 整理成有父子關係的 structure，並且給予每筆資料自己的 `key` 與 `name`。（詳細程式碼可以到最後的連結看，這部分比較跟資料格式相關，就不放在這裡佔版面了）

在原本 v3 的做法裡，如果我們要把資料轉化成 hierarchy 的格式，可以直接利用 `cluster.nodes()` ，即可一次初始化 cluster 並且得到擁有 x, y 值 的 nodes，但在 v4 中，我們必須先利用 `d3.hierarchy()` 將資料轉化成 hierarchy 格式，建立好父子關係與每個 node 的深度，接著才能丟入 `cluster` 中初始，其回傳值才會是擁有對應 cluster 內 x, y 值的 nodes。

`window.nodes = cluster(d3.hierarchy(packageHierarchy(classes))).children;`
(這邊取 children 也只是資料格式的關係)

由於繪製 `svg path` 需要給訂 data 的 source 與 target，因此這邊利用 `typeImmune`, `typeStrong`, `typeWeak` 來作轉換，也將資料分為這三種關係的 path 來繪製。

接著，轉換好後就能根據 node 的 `source` 與 `target` 繪製 path。

```js draw-path
// 這邊只列出一種
window.immune = immune
        .data(immunes.map(function(node){
            return node.source.path(node.target);
        }))
        .enter().append("path")
        .each(function(d) {
            d.source = d[0], d.target = d[d.length - 1];
        })
        .attr("class", "immune")
        .attr("d", line);
```

以往在 v3，我們可以事先定義 `var bundle = d3.layout.bundle();`，然後在上面這段程式碼中的 `data()` 中呼叫 `bundle(immunes)`，他就會幫我們把 source 跟 target 做連接。

但是在 v4 裡，`bundle` 被 `node.path()` 給取代了。

注意喔！是 `node.path()`，層級是在 node，因此我們要從剛才分類好的 immunes 中將 node 一個一個抓出來呼叫。

另外在這邊我們有用一個 `each()` 來將每筆 node 資料都加上 `d.source = d[0], d.target = d[d.length - 1];`
原因是為了之後我們點擊每個類別的時候，要利用這個來找出對應的點來上色。

```js colorPath.js

window.colorPath = function(d, l, type) {
      var type = type || 'strong';
      if (type == 'strong') {
        if (l.target === d) return l.source.target = true;
      }
      if (type == 'weak') {
        for (type in d) {
          if(type !== "size") {
            if (l.target === d[type]) return l.source.target = true;
          }
        }
      }
    }
```
到目前為止，已經把原本 v3 的 cluster layout 轉移到 v4 了，其餘繪製部分就與版本沒有什麼關聯性，需要注意的是資料格式的變動，像是在原本作者的程式碼內，點擊類別的 `activate()` 函數中，根據 `d.name` 來判斷位置的部分，由於資料格式的變動，要改為 `d.data.name`：

```js activate(d).js

    window.node
        .classed("node--active", function(target) {
            return (target === d) || this.classList.contains("node--active");
        })
        .classed("node--target", function(n) {
            return n.target;
        })
        .classed("immune-node", function(target, l) {
            return (this.classList.contains('immune-node') || target.data.immunes.indexOf(d.data.name) != -1);
        })
        ....
        ...
        ..

```

其餘繪製部分，包含點擊後的上色邏輯（ activate() 函數中），有興趣的讀者就直接看 code 吧，相信會更清楚！

[<span style="font-size: 20px; color: #c40522;">成果 與 程式碼</span>](http://bl.ocks.org/arvinh/b30ed888914a2794830ceb023c911a5b)

不過實際上我並沒有完整 Migration 完成，在原本 v3 的 `d3.svg.line.radial()` 這裡，v4 的寫法應該是 `d3.radialLine()`，並搭配上 `curve()` 函數，只是我並沒有嘗試成功，還請高手指教！


最後，送給大家一隻純 CSS 卡比獸，祝大家早日成為神奇寶貝大師！
（好啦其實不像XD...畢竟我對 css 的掌控度大概跟我對腰間肥肉的掌控度一樣低落...）

<p data-height="448" data-theme-id="dark" data-slug-hash="dXLvVm" data-default-tab="css,result" data-user="arvin0731" data-embed-version="2" class="codepen">See the Pen <a href="http://codepen.io/arvin0731/pen/dXLvVm/">Snorlax-pokemon</a> by Arvin (<a href="http://codepen.io/arvin0731">@arvin0731</a>) on <a href="http://codepen.io">CodePen</a>.</p>
<script async src="//assets.codepen.io/assets/embed/ei.js"></script>

<!-- 資料來源 -->
## 資料來源
1. [d3/d3-hierarchy](https://github.com/d3/d3-hierarchy#hierarchy)
2. [d3/d3-shape](https://github.com/d3/d3-shape)
3. [origin version](http://filipekiss.github.io/pokemon-type-chart/)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化