---
title: D3v4 工作坊 - 製作 D3 plugin 來繪製草圖風格長條圖
date: 2017-10-21 18:16:38
tags:
  - d3
  - plugin
  - rollup
---

## 前言

前陣子 Mozilla 如火如荼地推出他們的量子專案，而其中在介紹 Quantum CSS 架構的時候，用了許多 [可愛的草圖]((https://hacks.mozilla.org/2017/08/inside-a-super-fast-css-engine-quantum-css-aka-stylo/) 來解說，讓我突然想到如果能夠將這種手繪風格套用到資料圖表上面，應該會蠻有趣的，也才因此有了這篇文章。

一般來說，我們不管是用 D3.js 或是 Highcharts 等工具來製作圖表，都是朝向專業、有質感、表達清晰的方向去實作，利用各種顏色搭配與互動操作來讓圖表更漂亮更吸引人。

![source: highchart offical site](/img/arvinh/highchart_demo.png)
（來源：highchart offical site）

但看久了以後總會有點疲乏，加上現代人專注力越來越短暫，利用一些漫畫、草圖的方式來呈現要說明的東西，反而夠容易抓住大家的注意力，像是上述文章的作者 Lin Clark 在 medium 上就有一系列的 [Cartoon Intro](https://code-cartoons.com/) 的文章。

但身為前端工程師，手上的工具只有 Javascript、CSS 與 HTML，要怎麼樣 "手繪" 圖表呢？

其實很簡單，利用 D3.js 我們就能做得到！

## d3 sketchy

>“開玩笑的吧？我知道 D3 可以在 Canvas 上繪圖，但 Canvas 用起來還是沒有操作 svg 來得方便，而 svg 畫的長條圖就是四四方方的啊！"

沒錯我一開始也是這麼想的，但就在某次偶然機會下，拜讀到 [D3 in Action](https://www.manning.com/books/d3js-in-action-second-edition) 這本書的作者在 [Github 上的 code](https://github.com/emeeks/d3-sketchy) 後讓我改觀了。

這篇文章的範例就是修改自他三年前的 code，當時還是 v3 的版本，似乎也沒有將它推上 npm。

所以順便利用這篇文章記錄並分享一下如何製作出 Sketchy 效果的長條圖，並且包裝成 d3 plugin 來使用！

先給大家看一下比較圖：

一般的長條圖：
![一般的長條圖](/img/arvinh/normal_barchart.png)

套用 sketchy 效果的長條圖：
![套用 sketchy 效果的長條圖](/img/arvinh/sketchy_barchart.png)

是不是瞬間活潑很多呢？

雖然你可能會說這樣喪失了圖表的精準度，但很多時候我們只需要用來強調比較性的結果，這種情況就很適合了。

## 那到底是如何做到的呢？

稍微仔細觀察上面的兩張圖，你可能會發現，擁有 Sketchy 效果的長條圖，好像少了間距，多了黑色的手繪編框。

發現到這點以後，答案就呼之欲出了！

其實所謂 Sketchy 的效果，就只是在原本的長條圖上加入了不規則的黑色邊框線條！

而這些黑色的邊框其實是利用 `svg` 的 `path` 來繪製的，我們把它拆開來看就很清楚：

<p data-height="419" data-theme-id="29194" data-slug-hash="XeQbPP" data-default-tab="html,result" data-user="arvin0731" data-embed-version="2" data-pen-title="sketchy-split" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/XeQbPP/">sketchy-split</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://production-assets.codepen.io/assets/embed/ei.js"></script>

黑色的手繪風格邊框其實是利用至少九條 `L` 線段繪製出來的，等於是將原本的長方圖用線段包起來，實作上我們只要在原本長條圖的 x, y 位置與寬高上加上一些隨機變動的 offset，就可以做出這樣的效果！原理其實就這麼簡單！

![pathd](/img/arvinh/path-d-order.png)

## 知道原理以後來看點 code 吧！

既然他的實作原理很簡單，我們理當可以將它變成一個 d3 的 plugin，像是其他 d3 v4 的模組一樣，讓其他使用者都能夠透過 npm 或是 script 的方式載入使用。

而要做成 d3-plugin 的話，在 code 的架構上我們就要稍微注意一下。

基本上我們希望能沿用一般使用 d3 lib 的語法習慣：

```js demo

d3.select('#svgParent').selectAll("g").data(data)
    .enter()
    .append("g")
    .attr("class", "bar");

d3.select("#svgParent")
    .selectAll("g.bar")
    .each(function (d, i) {
        var x = i * 100 + 20;
        var y = 500 - hscale(d) / 2;
        var rw = 100;
        var rh = hscale(d) / 2;
        // 1. 我們想將此 plugin 放到 d3 的 global object 內
        // d3.sketchy 就是我們的 plugin
        var sketchyBar = d3.sketchy.rect();

        // 2. 我們要能 chaining 函數
        sketchyBar
            .height(rh)
            .width(rw)
            .x(x)
            .y(y)
            .fill(color)
            .stroke("black")
            .strokeWidth(10)
            .jostle(5)
        // 3. 最後我們利用 d3-selection 來幫我們選取 d3 DOM 傳入
        d3.select(this).call(sketchyBar);
    })
```

要達到這種效果，對 Javascript 熟悉的你應該都不困難，我們只要 `export` 一個物件，在其內設定一個會回傳函式的 function，製作出 closure，再實作相對應的 `setter`，並且每一個 `setter` 都回傳 `this` 即可。


```javascript d3-sketchy
import { select as d3Select, selectAll as d3SelectAll } from "d3-selection";
import { scaleLinear as d3ScaleLinear } from "d3-scale";
import { line as d3Line, curveLinear as d3CurveLinear } from "d3-shape";
let d3sketchy = {};

d3sketchy.rect = function(selection) {

  let rh = 50, rw = 10, w = 2, c = [0, 0],
    fillColor = "red", strokeColor = "black", jostle = 0;

  function d3_sketchyRect(selection) {
      // 繪製 sketchy 長條圖的主要函式
  }

  // 實作各個屬性的 setter
  d3_sketchyRect.height = function(data) {
    if (!arguments.length) return rh;
    rh = data;
    return this;
  }

  d3_sketchyRect.width = function(data) {
    // ... 作法與 height 相同
  }

  d3_sketchyRect.x = function(data) { /* ... */ }

  d3_sketchyRect.y = function(data) { /* ... */ }

  d3_sketchyRect.fill = function(data) { /* ... */ }

  d3_sketchyRect.jostle = function(data) { /* ... */ }

  // ... 可以再繼續設置你要的函數

  return d3_sketchyRect;
}
export default d3sketchy;
```

而 export 出去的物件函式執行後所回傳的 function 就是用來接收 d3-selection 傳遞給我們的 d3 DOM，並且繪製 sketchy bar 的函式，相關的 code 大家可以從下面的 codepen 看到，或是到我的 [github](https://github.com/ArvinH/d3-sketchy/blob/master/src/sketchy.js) 上去看也可以：

<p data-height="360" data-theme-id="29194" data-slug-hash="QqPbEK" data-default-tab="result" data-user="arvin0731" data-embed-version="2" data-pen-title="d3-sketchy demo I" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/QqPbEK/">d3-sketchy demo I</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://production-assets.codepen.io/assets/embed/ei.js"></script>

繪製 sketchy bar 的函式主要做三件事：

1. 繪製一般的長條圖

2. Random 產生一些動態 offset （在 `d3_sketchyRect` 這個函式中可以看到有一段看似複雜的計算，其實就只是對稱的畫出內外框，像是在原理介紹那邊的圖一樣）

3. 用 Svg path 來繪製手繪邊框

## 最後來打包 Plugin 吧！

其實要打包一個 lib 有很多方法，但是 D3 的作者在 D3 v4 尚未推出前就有寫了一篇文章介紹要[怎麼樣製作一個 d3 plugin](https://bost.ocks.org/mike/d3-plugin/)，為了要能盡量共用所有的 d3 plugin，最好大家都是遵照同樣的準則來處理會比較好。

不過其實也沒什麼準則 XD...主要比較特別的就是 D3 的 plugin 大多是用 rollup 來打包，我猜想是因為當時 webpack 1.x 還沒有加入 tree-shaking 的功能，而針對 plugin 這種小型模組來說，也用不到 webpack 的許多功能，rollup 反而比較適合用來打包 plugin 模組。

完整的 code 與結構可以直接到 github 上參考：[d3-sketchy](https://github.com/ArvinH/d3-sketchy/)

檔案結構大概就是這樣：

![檔案結構](/img/arvinh/file_structure.png)

在你的 `index.js` 中加入：

`export {default as sketchy} from "./src/sketchy";`

當作 module 的入口，也方便 require。

接著在你的 package.json 中加入 script：

```json package.json
"scripts": {
    "pretest": "rm -rf build && mkdir build && rollup -c rollup.config.js",
    "test": "tape 'test/**/*-test.js'",
    "prepublish": "npm run test && uglifyjs build/d3-sketchy.js -c -m -o build/d3-sketchy.min.js",
    "postpublish": "zip -j build/d3-sketchy.zip -- LICENSE README.md build/d3-sketchy.js build/d3-sketchy.min.js"
}
```

可以看到這邊在 publish 前都會用 uglify.js 來產生 min.js 檔案，如此一來你的 user 就可以選擇要不要下載 minify 過的套件。

最後就是撰寫你的 `rollup.config.js`：

```js rollup.config.js
import resolve from 'rollup-plugin-node-resolve';
import babel from 'rollup-plugin-babel';

export default {
    entry: 'index.js',
    dest: 'build/d3-sketchy.js',
    format: 'umd', // umd 的格式能透過 CommonJS 或是 AMD 載入
    moduleName: 'd3', // 設定為 d3 可以將你的 plugin 放入 global.d3 底下
    sourceMap: true,
    plugins: [
        resolve(),
        babel({
            exclude: 'node_modules/**'
        })
    ]
};
```

這邊要注意兩件事情。

1. rollup 不會幫你打包你從 node_modules 裡面 import 進來的檔案，所以如果你有用到其他的 d3 plugin，變成在你的 plugin 說明內要告知使用者，記得載入相依的 plugin。或是你也可以向我這邊一樣，透過 `rollup-plugin-node-resolve` 這個 rollup plugin 來幫我們把 `node_modules` 底下的 lib 也打包

2. rollup 不會幫你編譯 babel，如果你真的想用 babel，就像我一樣加入 `rollup-plugin-babel` 即可。
3. format 記得設為 umd，這樣才能夠在 CommonJS 或 AMD 環境下都能使用。
4. 最雷的一點在這邊！目前用 rollup 打包 d3 plugin 的話，你需要將 roullup 的版本鎖在 0.41，否則你會發現你編譯出來的 `build/d3-xxx.js` 內的最前面幾行長得像這樣：

```js
(function (global, factory) {
	typeof exports === 'object' && typeof module !== 'undefined' ? factory(exports) :
	typeof define === 'function' && define.amd ? define(['exports'], factory) :
	(factory((global.d3 = {})));
}(this, (function (exports) { 'use strict';
```

有看出什麼問題嗎？

`factory((global.d3 = {}))`

他會將 global.d3 塞入一個空物件，而非先使用原先的 global.d3：`factory((global.d3 = global.d3 || {}))`

如此一來，你如果在載入 plugin 之前先載入其他 d3 plugin，就會發生問題了...

```html
  <script src="https://d3js.org/d3.v4.js"></script>
  <script src="../build/d3-sketchy.js"></script> <!-- 裡面的 d3 object 會蓋掉上面 d3.v4.js 產生的--->
```

可能有其他解法，但看了 d3 的一些 plugin 也是先將版本卡在 0.41。

當你做完上述工作後，在你的 repo 底下執行 `npm install`，就會產生 `build` folder，裡面含有：

1. d3-sketchy.js
2. d3-sketchy.min.js
3. d3-sketchy.js.map (如果你 rollup.config.js 有設定 sourceMap: true 就會有此檔案)

如此一來別人就能使用你的 plugin 了：

```html
<script src="../build/d3-sketchy.js"></script>

<script>
    var sketchyBar = d3.sketchy.rect();
    //...
    //..
</script>
```

## 結論

有些時候看似很複雜的東西，其實原理卻非常簡單，只是需要發揮點創意。剛好透過這篇也學習了一下 rollup 的相關設定，雖然我私人猜測 rollup 應該還是會慢慢被 webpack 壓過去，即便 rollup 作者寫了[這篇文章](https://medium.com/webpack/webpack-and-rollup-the-same-but-different-a41ad427058c)。
這篇算是一個小小的筆記，不是太完整，若有任何疑問或建議歡迎留言討論！

## 資料來源
1. [d3-sketchy](https://github.com/emeeks/d3-sketchy)
2. [d3-plugin](https://bost.ocks.org/mike/d3-plugin/)
2. [rollup](https://github.com/rollup/rollup)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化
