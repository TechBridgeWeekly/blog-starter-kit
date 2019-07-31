---
title: CSS 魔術師 Houdini API 介紹
date: 2017-05-23 19:44:02
tags: 
    - CSS
    - Houdini
    - Custom-Properties
    - Worklets
author: arvinh
---

# 前言

今天想繼續 CSS 的議題，常常會覺得學 CSS 的新發展不太划算，因為每次看到新的 Feature 出現，都只能當下興奮個幾分鐘，然後就會認命接受可能還要再等個五年才能真正使用的可能性...如果你有跟我一樣的感受，那今天這篇文章或許可以帶給你一絲絲希望。

在現今的 Web 開發中，JavaScript 幾乎佔據所有版面，除了控制頁面邏輯與操作 DOM 物件以外，連 CSS 都直接寫在 JavaScript 裡面了，就算瀏覽器都還沒有實作的規格，總會有人做出對應的 Polyfills，讓你快速的將新 Feature 應用到 Production 環境中，更別提我們還有 Babel 等工具幫忙轉譯。

而 CSS 就不同了，除了制定 CSS 標準規格所需的時間外，各家瀏覽器的版本、實作進度差異更是曠日費時，再加上 CSS 並非 Javascript 這樣的動態語言，我們無法簡單的提供 Polyfills，頂多利用 PostCSS、SASS 等工具來幫我們轉譯出瀏覽器能接受的 CSS，而剩下的就是瀏覽器的事了。

這邊讓我們回想一下，瀏覽器在網頁的渲染過程中，做了哪些事情？

![Render Pipeline (source from google)](/img/arvinh/render_pipeline.png)
*source from google*

瀏覽器的 Render Pipeline 中，**JavaScript** 與 **Style** 兩個階段會解析 HTML 並為載入的 JS 與 CSS 建立 Render Tree，也就是所謂的 DOM 與 CSSOM：（對於 Render Pipeline 與 Render Tree 若不了解，可以先看看我先前的文章 [Front-end kata 60fps的快感](https://blog.arvinh.info/2016/03/26/Front-end%20kata%2060fps%E7%9A%84%E5%BF%AB%E6%84%9F/)）

![Render Tree (source from google)](/img/arvinh/RenderTree.png)
*source from google*

而就現階段的 Web 技術來看，開發者們能操作的就是透過 JS 去控制 DOM 與 CSSOM，來影響畫面的變化，但是對於接下來的 **Layout**、**Paint** 與 **Composite** 就幾乎沒有控制權了。

既無法讓各家瀏覽器快速並統一實作規格，亦不能輕易產生 Polyfills，所以到現在我們依然無法大膽使用 Flexbox，即便它早在 2009 年就被提出了...

但 CSS 並非就此駐足不前。

為了解決上述問題，為了讓 CSS 的魔力不再被瀏覽器把持，Houdini 就誕生了！（ Houdini 是美國的偉大魔術師，擅長逃脫術，很適合想將 CSS 從瀏覽器中解放的概念）

# CSS Houdini

CSS Houdini 是由一群來自 Mozilla, Apple, Opera, Microsoft, HP, Intel, IBM, Adobe 與 Google 的工程師所組成的工作小組，志在建立一系列的 API，讓開發者能夠介入瀏覽器的 CSS engine 運作，帶給開發者更多的解決方案，用來解決 CSS 長久以來的問題：

* Cross-Browser isse
* CSS Polyfill 的製作困難

Houdini task force 目前起草了一些 API 規範，並逐步努力讓其通過 W3C，成為真正的 Web standards。
由於都是草稿階段，有些甚至只有規劃，還未被真正寫入規範，所以變動很大，有些我也不是很了解，所以就大致介紹一下，若有錯誤拜託務必告知！
另外，有興趣的讀者可以直接從這裡 [CSS Houdini Drafts](http://dev.w3.org/houdini/) 看詳細內容（ Drafts 的更新時間都非常近期，活躍中的草稿！）。

下面這張圖我將 Google 提供的 Render pipeline 與 [Houdini: Maybe The Most Exciting Development In CSS You’ve Never Heard Of](https://www.smashingmagazine.com/2016/03/houdini-maybe-the-most-exciting-development-in-css-youve-never-heard-of/) 中提到的 pipeline 做個結合對比，顯示出 Houdini 試圖在瀏覽器的 Render pipeline 中提供哪些 API 給開發者使用：

![Houdini API on render pipeline](/img/arvinh/huodini-apis.png)

其中灰色部分就是只在規劃階段，而黃色部份就是已經寫入規範正在推行中。

# Houdini API 介紹

## CSS Properties and Values API

先介紹一個最能夠使用的 API，除了 IE family 以外，Chrome、Firefox 與 Safari 都已經能夠直接使用了！ [caniuse](http://caniuse.com/#search=custom%20properties)

相信很多人都使用過 CSS Preprocessors，他給予開發者在 CSS 中使用變數的能力：

```scss
$font-size: 10px;
$brightBlue: blue;

.mark{
  font-size: 1.5 * $font-size;
  color: $brightBlue
}
```

但其實使用 Preprocessors 還是有其缺點，像是不同的 Preprocessors 就有不同的 Syntax，需要額外 setup 與 compile，
而現在 CSS 已經有原生的變數可以使用了！就是 CSS Properties and Values API！

SCSS 與 Native CSS Custom Properties 的一個主要差別可以看[下圖](http://slides.com/malyw/houdini-codemotion#/18)：

![source from http://slides.com/malyw/houdini-codemotion#/18](/img/arvinh/scss-vs-css.png)

原生的 CSS variable syntax：

```css
/* declaration */
--VAR_NAME: <declaration-value>;
/* usage */
var(--VAR_NAME)
```

變數可以定義在 root element selector 內，也能在一般 selector 內，甚至是給別的變數 reuse：

```scss
/* root element selector (全域) */
:root {
  --main-color: #ff00ff;
  --main-bg: rgb(200, 255, 255);
  --block-font-size: 1rem;
}

.btn__active::after{
  --btn-text: 'This is btn';

  /* 相等於 --box-highlight-text:'This is btn been actived'; */
  --btn-highlight-text: var(--btn-text)' been actived';

  content: var(--btn-highlight-text);

  /* 也能使用 calc 來做運算 */
  font-size: calc(var(--block-font-size)*1.5);
}

body {
  /* variable usage */
  color: var(--main-color);
}
```

而有了變數以後，會為 CSS 帶來什麼好處應該很明顯，他的 Use case 可以多寫一篇文章來介紹了，[或是可以直接看這篇的詳細介紹](https://blog.hospodarets.com/css_properties_in_depth)，我這邊介紹幾個我覺得比較有趣的：

1. 模擬一個特殊的 CSS rule:

    單純透過更改變數來達到改變 box-shadow 顏色

    ```css
    .textBox {
    --box-shadow-color: yellow;
    box-shadow: 0 0 30px var(--box-shadow-color);
    }

    .textBox:hover {
    /* box-shadow: 0 0 30px green; */
    --box-shadow-color: green;
    }
    ```

2. 動態調整某個 CSS rule 內的各別屬性：

    <p data-height="300" data-theme-id="29194" data-slug-hash="ZKKyer" data-default-tab="html,result" data-user="malyw" data-embed-version="2" data-pen-title="Generated colors from CSS custom properties" class="codepen">See the Pen <a href="https://codepen.io/malyw/pen/ZKKyer/">Generated colors from CSS custom properties</a> by Serg Hospodarets (<a href="https://codepen.io/malyw">@malyw</a>) on <a href="https://codepen.io">CodePen</a>.</p>
    <script async src="https://production-assets.codepen.io/assets/embed/ei.js"></script>

此外，我們也可以用 JavaScript 來控制：

```js
const textBox = 
  document.querySelector('.textBox');
// GET
const Bxshc = getComputedStyle(textBox)
    .getPropertyValue('--box-shadow-color');

// SET
textBox.style
    .setProperty('--box-shadow-color', 'new color');
```

非常好用的特性，幾乎所有主流瀏覽器都已經支援了，大家快來使用吧！

## Box Tree API

Box tree API 並沒有出現在上圖中，但在 Paintin API 中會用到其概念。
大家都知道在 DOM tree 中的每個元素都有一個 Box Modal，而在瀏覽器解析過程中，還會將其拆分成 fragments，至於什麼是 fragments？以 drafts 中的例子來解釋：
<p data-height="300" data-theme-id="29194" data-slug-hash="wdZvOm" data-default-tab="css,result" data-user="arvin0731" data-embed-version="2" data-pen-title="fragments-sample" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/wdZvOm/">fragments-sample</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://production-assets.codepen.io/assets/embed/ei.js"></script>

上面的 HTML 總共就會拆出七個 fragments：
* 最外層的 div
* 第一行的 box (包含 foo bar)
* 第二行的 box (包含 baz)
* 吃到 `::first-line` 與 `::first-letter` 的 f 也會被拆出來成獨立的 fragments
* 只吃到 `::first-line` 的 oo 只好也獨立出來
* 吃到 `::first-line` 與 包在 `<i>` 內的 bar 當然也是
* 在第二行底下且為 italic 的 `baz`

而 Box tree API 目的就是希望讓開發者能夠取得這些 fragments 的資訊，至於取得後要如何使用，基本上應該會跟後面會介紹的 Parser API、Layout API 與 Paint API 有關聯，當我們能取得詳細的 Box Modal 資訊時，要客製化 Layout Module 才更為方便。

## CSS Layout API

Layout API 顧名思義就是提供開發者撰寫自己的 Layout module，Layout module 也就是用來 assign 給 `display` 屬性的值，像是 `display: grid` 或 `display: flex`。
你只要透過 `registerLayout` 的 function，傳入 Layout 名稱與 JS class 來定義 Layout 的邏輯即可，例如我們實作一個 `block-like` 的 Layout：

```js blockLike.js
registerLayout('block-like', class extends Layout {
    static blockifyChildren = true;
    static inputProperties = super.inputProperties;

    *layout(space, children, styleMap) {
        const inlineSize = resolveInlineSize(space, styleMap);

        const bordersAndPadding = resolveBordersAndPadding(constraintSpace, styleMap);
        const scrollbarSize = resolveScrollbarSize(constraintSpace, styleMap);
        const availableInlineSize = inlineSize -
                                    bordersAndPadding.inlineStart -
                                    bordersAndPadding.inlineEnd -
                                    scrollbarSize.inline;

        const availableBlockSize = resolveBlockSize(constraintSpace, styleMap) -
                                   bordersAndPadding.blockStart -
                                   bordersAndPadding.blockEnd -
                                   scrollbarSize.block;

        const childFragments = [];
        const childConstraintSpace = new ConstraintSpace({
            inlineSize: availableInlineSize,
            blockSize: availableBlockSize,
        });

        let maxChildInlineSize = 0;
        let blockOffset = bordersAndPadding.blockStart;

        for (let child of children) {
            const fragment = yield child.layoutNextFragment(childConstraintSpace);

            // 這段控制 Layout 下的 children 要 inline 排列
            // fragment 應該就是前述的 Box Tree API 內提到的 fragment
            fragment.blockOffset = blockOffset;
            fragment.inlineOffset = Math.max(
                bordersAndPadding.inlineStart,
                (availableInlineSize - fragment.inlineSize) / 2);

            maxChildInlineSize =
                Math.max(maxChildInlineSize, childFragments.inlineSize);
            blockOffset += fragment.blockSize;
        }

        const inlineOverflowSize = maxChildInlineSize + bordersAndPadding.inlineEnd;
        const blockOverflowSize = blockOffset + bordersAndPadding.blockEnd;
        const blockSize = resolveBlockSize(
            constraintSpace, styleMap, blockOverflowSize);

        return {
            inlineSize: inlineSize,
            blockSize: blockSize,
            inlineOverflowSize: inlineOverflowSize,
            blockOverflowSize: blockOverflowSize,
            childFragments: childFragments,
        };
    }
});
```

上面這段 code 是來自 [houdini draft 的範例](https://drafts.css-houdini.org/css-layout-api/)，完整放上來是想給大家看一下實作一個 Layout 需要注意的細節有多少，其實並不是如想像中的輕鬆，
相信未來會出現更多方便的 API 輔助開發。（放心接下來不會再有這麼多 code 了 XD）

有了 Layout API，不管是自己實作或是拿別人寫好的 Layout，你都可以直接如下方式使用：

```css
.wrapper {
    display: layout('block-like');
}
```

## CSS Painting API

Painting API 與 Layout 類似，提供一個叫做 `registerPaint` 的方法：

定義 Paint Method，這邊偷偷用到了待會要介紹的 CSS Properties：
```js simpleRect.js
registerPaint('simpleRect', class {
    static get inputProperties() { return ['--rect-color']; }
    paint(ctx, size, properties) {
        // 依據 properties 改變顏色
        const color = properties.get('--rect-color');
        ctx.fillStyle = color.cssText;
        ctx.fillRect(0, 0, size.width, size.height);
    }
});
```

宣告使用：
```css
.div-1 {
    --rect-color: red;
    width: 50px;
    height: 50px;
    background-image: paint(simpleRect);
}
.div-2 {
    --rect-color: yellow;
    width: 100px;
    height: 100px;
    background-size: 50% 50%;
    background-image: paint(simpleRect);
}
```

`.div-1` 與 `.div-2` 就可以擁有各自定義寬高顏色的方形 background-image

## Worklets

在上述的 Layout API 與 Paint API 中，我們都有撰寫一支 js 檔案，用來定義新的屬性，然後在 css 檔案中呼叫取用，你可能會覺得那支 js 檔案就直接像一般 web 嵌入 js 的方式一樣即可，
但實際上並非如此，我們需要透過 **Worklets** 來幫我們載入。以上面的 Paint API 為例：

```js
// add a Worklet
paintWorklet.addModule('simpleRect.js');

// WORKLET "simpleRect.js"
registerPaint('simpleRect', class {
    static get inputProperties() { return ['--rect-color']; }
    paint(ctx, size, properties) {
        // 依據 properties 改變顏色
        const color = properties.get('--rect-color');
        ctx.fillStyle = color.cssText;
        ctx.fillRect(0, 0, size.width, size.height);
    }
});
```

同理，Layout API 則是 `layoutWorklet.addModule('blockLike.js')`。

**Worklets** 光名字就有點像 Web Worker 了，都是獨立於主要執行緒之外，並且不直接與 DOM 互動。你可能會想那為何還需要有一個 Worklets？

因為 Houdini 是希望將開發者的程式碼 hook 到 CSS engine 中運作，而根據規範內的敘述，web worker 相對笨重，不適合用來處理 CSS engine 這種可能會牽扯到數百萬畫素圖片的工作。
所以可以推斷，Worklets 的特點就是輕量以及生命週期較短。

其實除了 Layout Worklets 與 Paint Worklets 外，還有所謂的 Animation Worklet，雖然還沒有放入規範，但已經有在著手進行中，也有 [Polyfills](https://github.com/GoogleChrome/houdini-samples/tree/master/animation-worklet) 了，Chrome 的 Sticky Header 就是採用 Houdini 的 Animation Worklet。Twitter 的 [Header Effect](https://blog.hospodarets.com/demos/houdini-anim-twitter/) 也是採用 Animation Worklet
Animation Worklet 是想介入 Render Pipeline 中的 Composite 步驟，也就是原本利用 js 與 css 控制動畫時，瀏覽器會重新執行的部分。
關於 Animation Worklet 的詳細實作介紹可以看這份投影片 [houdini-codemotion](http://slides.com/malyw/houdini-codemotion#/48)

## CSS Parser API

Parser API 目前還是處在 Unofficial draft，但我相信如果這個 API 確認的話，對前端開發有絕對的幫助，她的概念是想讓開發者能擴充瀏覽器解析 HTML、CSS 的功能，
也就是說，你可以想辦法讓他看得懂最新定義的 pseudo-classes 或甚至是 element-queries 等等，這樣就能正確解析出 CSSOM，從此不用再等瀏覽器更新。

## CSS Typed OM

CSS Typed OM 就是 CSSOM 的強化版，最主要的功能在於將 CSSOM 所使用的字串值轉換成具有型別意義的 JavaScript 表示形態，像是所有的 CSS Values 都有一個 base class interface：

```js
interface CSSStyleValue {
    stringifier;
    static CSSStyleValue? parse(DOMString property, DOMString cssText);
    static sequence<CSSStyleValue>? parseAll(DOMString property, DOMString cssText);
};
```

你可以如下操作 CSS style: (source from [CSS Houdini- the bridge between CSS, JavaScript and the browser](http://slides.com/malyw/houdini-codemotion#/27))
*source from [CSS Houdini- the bridge between CSS, JavaScript and the browser*

```js
// CSS -> JS
const map = document.querySelector('.example').styleMap;
console.log( map.get('font-size') );
// CSSSimpleLength {value: 12, type: "px", cssText: "12px"}

// JS -> JS
console.log( new CSSUnitValue(5, "px") );
// CSSUnitValue{value:5,unit:"px",type:"length",cssText:"5px"}

// JS -> CSS
// set style "transform: translate3d(0px, -72.0588%, 0px);"
elem.outputStyleMap.set('transform',
    new CSSTransformValue([
        new CSSTranslation(
          0, new CSSSimpleLength(100 - currentPercent, '%'), 0
        )]));
```

根據 [Drafts 的內容](https://drafts.css-houdini.org/css-typed-om/)，有了型別定義，在 JavaScript 的操作上據說會有效能上的顯著提升。此外，CSS Typed OM 也應用在 Parser API 與 CSS Properties API 上。

## Font Metrics API

Font Metrics 也沒有出現在上方的 **Houdini API on render pipeline** 中，但它其實已經被寫入 [Draft 規範](https://drafts.css-houdini.org/font-metrics-api/) 中了。
老實說看不是很懂他的 spec 寫的內容，但我猜測這東西的用途應該跟這篇文章 [Deep dive CSS: font metrics, line-height and vertical-align](http://iamvdo.me/en/blog/css-font-metrics-line-height-and-vertical-align) 其中提到一個問題有關，（裡面非常詳細的介紹了 font metrics、line-height 與 vertical-align 在網頁上如何互相影響，推薦大家有空的話耐心閱讀一番。）：

不同 font-family 在相同 font-size 下，所產生的 span 高度會不同。

![source from http://iamvdo.me/en/blog/css-font-metrics-line-height-and-vertical-align](/img/arvinh/font-size.png)
*source from http://iamvdo.me/en/blog/css-font-metrics-line-height-and-vertical-align*

要想控制 Font metrics，也就是控制字所佔的寬高的話，目前可以先用 CSS Properties 來處理，根據已知字體的 font-metrics 動態算出我們要 apply 多少的 font-size：

```css
p {
    /* 定義好我們已知字型的 Font metrics */
    /* font metrics */
    --font: Catamaran;
    --fm-capitalHeight: 0.68;
    --fm-descender: 0.54;
    --fm-ascender: 1.1;
    --fm-linegap: 0;

    /* 定義想要的高度 */
    --capital-height: 100;

    /* 設定 font-family */
    font-family: var(--font);

    /* 利用 Font metrics 的資訊與想定義的高度來計算出 font-size */
    --computedFontSize: (var(--capital-height) / var(--fm-capitalHeight));
    font-size: calc(var(--computedFontSize) * 1px);
}
```

而想必 Font Metrics API 就是希望能 expose 出更方便的 API 來達成上述的事情。

## 結論
<!-- JS 與 CSS 的發展速度剛好相反 -->
Web 開發基本上就是由 HTML、JS、CSS 三大要素構成，然而 JS 與 CSS 的發展差異卻極其龐大，一個速度快到讓人跟不上，一個則是等半天還是無法放心使用新規格，實在非常有趣...
但透過這次了解 Houdini API 的過程，理解到了 CSS 算是朝向好的方向前進，雖然很多離實際採用還有段距離，但至少我們已經能夠在最新的瀏覽器上使用 Custom Properties 了！CSS 的未來還是充滿希望的！

## 資料來源
1. [CSS Houdini- the bridge between CSS, JavaScript and the browser](http://slides.com/malyw/houdini-codemotion))
2. [Houdini Draft](https://drafts.css-houdini.org)
3. [Deep dive CSS: font metrics, line-height and vertical-align](http://iamvdo.me/en/blog/css-font-metrics-line-height-and-vertical-align)
4. [Houdini: Maybe The Most Exciting Development In CSS You’ve Never Heard Of](https://www.smashingmagazine.com/2016/03/houdini-maybe-the-most-exciting-development-in-css-youve-never-heard-of/)
5. [houdini-samples](https://github.com/GoogleChrome/houdini-samples)
6. [Houdini API Draft](https://github.com/w3c/css-houdini-drafts)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化
