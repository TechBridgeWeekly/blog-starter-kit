---
title: 從製作 visfest 2019 badge 認識 ObservableHQ

date: 2019-09-09 02:01:00
tags:
  - d3.js
  - javascript
  - visualization
---

## 前言

軟體工程師應該很少沒參加過 Conference 吧，不管是社群或是學術性的，只要是與會者都會拿到一面主辦方製作的名牌，上面除了印著你的大名外，大多就剩下 Conference 名稱與 Logo 了，然而，由灣區的資料視覺化社群所舉辦的年度聚會 - visfest unconf 很是特別，他們提供與會者一個製作自己 badge 的機會，讓大家自行發揮創意，特別之餘也很符合整個會議的調性。今年八月是他們舉辦的第五屆 visfest unconf，這次他們在 [ObservableHQ](https://observablehq.com/) 這個平台上釋出了一個 Template，讓大家更方便的製作名牌，效果如同下方（因為用到 ECMAScript modules，建議使用桌面版 Chrome），若無法觀看，可以前往此 [link](https://observablehq.com/@arvinh/visfest-unconf-badge-builder-template)，或是拉到最下面看 gif：

<div id="animation"></div>

<script type="module">
  import notebook from "https://cors-anywhere.herokuapp.com/https://api.observablehq.com/@arvinh/visfest-unconf-badge-builder-template.js";

  const renders = {
    "result": "#animation",
  };

  import {Inspector, Runtime} from "https://unpkg.com/@observablehq/notebook-runtime@2?module";
  for (let i in renders)
    renders[i] = document.querySelector(renders[i]);

  Runtime.load(notebook, (variable) => {
    console.log(variable)
    if (renders[variable.name])
      return new Inspector(renders[variable.name]);
  });
</script>

除了 Observable 與 visfest 的 Logo 外，你可以繪製任何你想呈現的東西，改變背景顏色等等，而主辦方會用 [gitpop](https://gifpop.io/) 這項服務，將你的動畫製作成 10 frames 的 gif，因此在製作時，可以根據 template 提供的繪圖函式傳入的 `frameNumbers` 來控制動畫的呈現。

我自己是覺得這樣的想法很酷，所以即便無緣參加 visfest unconf，也製作了一個自己的 Badge 玩玩，順便來試用已經想玩很久的平台 [ObservableHQ](https://observablehq.com/)，這篇文章就記錄一下使用的過程，並說明其特性，希望能引起大家興趣！

## ObservableHQ 起源

D3.js 的作者 [@mbostock](https://twitter.com/mbostock)，在 2017 年的時候發了一篇名為 [A Better Way to Code](https://medium.com/@mbostock/a-better-way-to-code-2b1d2876a3a0) 的文章，介紹了他當時正在製作的專案 - `d3.express`，也就是現在的 [ObservableHQ](https://observablehq.com/)。

<!-- 介紹目的 -->
Mike Bostock 在過去十年左右的時間都致力於資料視覺化，為了能夠更方便的將資料以各種方式呈現，D3.js 因應而生，然而，致力於一個工具太久，反而忘了當初為何要製作工具，目的變成了工具本身，而不是透過該工具所能傳達的價值。

> The purpose of visualization is insight, not pictures - Ben Shneiderman

資料視覺化的重點應在於將資料的 insight 更加輕易的傳遞給世界，儘管現在已經有很多圖形化介面的拖拉工具輔助你將資料視覺化，但若是想要最大幅度的自由創作並表達 insight，撰寫程式碼是無可避免的。

然而寫程式一直都不是一件簡單的事，為了降低寫程式的難度，他創造了另一個工具：[ObservableHQ](https://observablehq.com/)。你可以把它想像成 JavaScript 版本的 jupyter notebook，但使用上某些語法跟純 JavaScript 有些差異，因此剛開始可能會有些不太習慣的地方，等到上手後，你就會發現它的好用之處，尤其適合應用在製作資料視覺化專案上。

## ObservableHQ 基礎操作

ObservableHQ 其實不能算是 JavaScript，因為有許多操作行為與 vanilla JavaScript 不相同，作者也有寫了一篇[說明](https://observablehq.com/@observablehq/observables-not-javascript?collection=@observablehq/introduction)。

基本上，你可以把 ObservableHQ 想像成一個試算表，由一格一格的 `cell` 所組成，而每一格 `cell` 都是一個 JavaScript snippet，可以是一則運算式、一段簡短的函式，當然也可以將值指定給一變數名稱。

![ObservableHQ statements](/img/arvinh/observablehq-statement.png)

在上面的圖片中，比較不同的地方在於運算函式的寫法，除了一般的 Function Declarations 與 Function Expressions 外，你也可以用一個大括號 `{}` 包裹著一段程式碼，最後 `return` 出去的值，就會被印出來。

而就像試算表的公式一樣，表格內的值是互相 reference 的，只要你在某處修改了某個 `cell` 的值，所有引用到該 `cell` 的程式都會相應改變（re-run），跟一般 vanilla JavaScript 從上到下執行程式碼的邏輯有所不同：

![demo](/img/arvinh/observablehq-var-reference.gif)

從這點來看，你也可以把所有在 ObservableHQ 上的變數 assignments 都想成是 hoisted declarations，因為宣告順序不影響 reference。

此外，支援使用 `html` 與 `markdown` 的語法，利用 `Template literals` 來完成：

![demo-html-md](/img/arvinh/observablehq-html-md.png)

還有一個特別的 built-in 運算子 - `viewof`：

![demo-viewof](/img/arvinh/observablehq-viewof.gif)

如上面的 gif 所示，viewof 可以將使用者針對一個 html input element 操作所產生的值，exposes 出來，或是 assign 給一個變數。可以想見，這用在互動式視覺化專案中會是很棒的功能。

這幾個大概是最基本的操作，還有其他諸如：如何匯入第三方套件、使用非同步 API 呼叫等等，我們就直接透過實際演練來學習吧！

## ObservableHQ 實際操作

接下來用我這次製作的 visfest badge 來做範例，進一步帶大家認識這個工具！

這次的 badge，主要概念是在 2D 地球圖上，顯示各城市的人口數量級別，用地球的轉動與 marker 的 radius 大小變化來呈現動畫。

要完成這個目標，起手第一步就是得先把 2D 地球畫出來。而 Mike Bostock 有提供一份 `topojson` 格式的世界地圖資料：`https://unpkg.com/world-atlas@1/world/110m.json`。

因此，我們需要載入 `topojson` 套件，並 `fetch` 世界圖資，而在 ObservableHQ 上我們可以這樣做：

![Import libs](/img/arvinh/observablehq-import-lib.png)

這邊我們用 `require` 來載入 `topojson` 套件，但其實它背後並非 CommonJS，而是用 [Asynchronous Module Definition (AMD)](https://requirejs.org/docs/whyamd.html) 實作。另外它也支援 ES modules 與 imports，所以可以依照你所使用的套件支援度來選擇要用哪種方式載入模組。

至於取得世界圖資，我們需要兩個步驟：

1. 取得圖資 JSON 檔案：`let world = await (await fetch("https://unpkg.com/world-atlas@1/world/110m.json")).json();`
2. 取得拓樸後的資料：`topojson.feature(world, world.objects.countries);`

在 ObservableHQ 上，我們可以利用先前說到的大括號 `{}` 來執行多行程式碼，並把最後 return 的值賦予給某個變數：

![Get world map data](/img/arvinh/observablehq-get-worldmap.png)

接著還需要有各城市的人口資料，取得方式與上面相同：

![Get cities population](/img/arvinh/observablehq-get-cities-pop.png)

資料到手後，剩下的就就跟一般撰寫 D3.js 的專案ㄧ樣。

在 visfest 提供的模板中，提供了許多以定義好的變數宣告，像是姓名、字體亮度、大小等等，而我們只需要在 `badgeCode` 這個函數中實作我們的視覺化作品即可。

```js
// Your D3 code goes here.
// The function badgeCode gets called below to generate the previews.
function badgeCode(g, c, frameNumber) {
  // g: a <g>, i.e. d3.select(... the node)
  // context: a canvas context, useful for doing canvas drawing
  // frameNumber: a number that represents the frame (1, 2, … 10) for psuedo-animation.
  // width & height are available via the environment, as is d3
  // Note: the <g> is drawn on top of the <canvas>
}
```

`badgeCode` 接收三個參數，一個是在 badge 上留給你繪圖的空間 `<g>` 元素，以及 `canvas context` 和 `frameNumber`，該模板會呼叫此函式十次，依序傳入遞增的 `frameNumber`，讓你來掌控動畫過程。

到這邊可能你會好奇，在 ObservableHQ 中的 **模板** 到底是怎麼運作的？為什麼能定義出一個函式，讓我去填寫內容，又幫我執行呢？

答案要追究到 ObservableHQ 的一個特殊 `import-with` 方法：

![import-with](/img/arvinh/observablehq-import-with.png)

`b93171820ba3f268` 是 ObservableHQ 上的另一個 notebook，也就是實際的 template 程式碼所在位置，我們可以從該 notebook 中 import 進 `{preview, animation, download, width, height, d3}` 這幾個函式來呼叫使用，而這幾個函式中所用到的變數，我們能使用 `with` 來取出，並且賦予其新的值！

這就是為什麼我們可以修改 `firstName`、`lastName` 以及自行填入 `badgeCode` 函數的原因了。

`badgeCode` 內的程式碼基本上就是普通的 D3.js 程式，這邊就不再附上程式碼，有興趣想知道怎麼實作在地球呈現人口分布的，可以到我的 [notebook](https://observablehq.com/@arvinh/visfest-unconf-badge-builder-template) 去看，其中我也有用到前面提及的 `viewof` 運算子，讓我能手動調整地球的 scale 大小。

都完成後就會看到由模板提供的 `preview` 函式所繪製出的十張圖：

![Preview ten pics](/img/arvinh/observablehq-preview-ten-pics.png)

以及用 `animation` 函式（一樣模板提供）繪製的動畫：

![Animation gif](/img/arvinh/observablehq-animation.gif)

## 分享你的 ObservableHQ 作品

完成作品後，最重要的就是分享。

這篇文章的最上方，我放入的成品並不是 gif，而是貨真價實從 ObservableHQ 所匯入的，利用 `ECMAScript modules`，載入想要嵌入的 notebook，接著再載入 ObservableHQ 提供的 `{Inspector, Runtime}`，當 notebook 載入後，取得其中 export 的變數，透過 `Inspector` 將其繪製到指定的 DOM id 上：

```html
<div id="animation"></div>

<script type="module">
  import notebook from "https://cors-anywhere.herokuapp.com/https://api.observablehq.com/@arvinh/visfest-unconf-badge-builder-template.js";

  const renders = {
    "result": "#animation",
  };

  import {Inspector, Runtime} from "https://unpkg.com/@observablehq/notebook-runtime@2?module";
  for (let i in renders) {
    renders[i] = document.querySelector(renders[i]);
  }

  Runtime.load(notebook, (variable) => {
    if (renders[variable.name]) {
      return new Inspector(renders[variable.name]);
    }
  });
</script>
```

透過這種方式，除了能夠在 ObservableHQ 上載入引用他人的 notebook 外，也能在一般網站上嵌入任何作品，非常方便！

## 結論

這篇文章只是非常簡略的說明了 ObservableHQ 的起源、用途與使用方式，希望能引起大家的興趣，如果有想要使用這套工具玩玩，或是製作視覺化專案的話，官方網站其實有出了一系列的[教學與說明文件](https://observablehq.com/collection/@observablehq/introduction)，直接就是用 ObservableHQ 的 notebook 撰寫的，互動式的閱讀體驗非常好，可以非常清楚的知道各個環節該怎麼使用，以及其背後的設計原理。

<!-- 資料來源 -->
## 資料來源

1. [Observablehq five minute intro](https://observablehq.com/@observablehq/five-minute-introduction)
2. [Observable’s not JavaScript](https://observablehq.com/@observablehq/observables-not-javascript?collection=@observablehq/introduction)
3. [Observablehq introduction](https://observablehq.com/collection/@observablehq/introduction)

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化
