---
title: 2017，讓我們再來看看 Web Components 吧！
date: 2017-01-06 13:00:24
tags: web, web components, shadow dom, custom element
author: arvinh
---

## 前言

先前研究 Progressive Web App 的時候就有注意到 Web Components 這個東西，也看過公司前輩使用過，甚至在前陣子 [GDG DevFest Taipei 2016](https://devfest-taipei-3cbee.firebaseapp.com/) 的官方網站內，不僅使用 PWA，也採用了 Web Components 的技術（當然應該是用 Google 的 Polymer）來搭建。但還是很少看人使用與談論，我自己也是從來沒有仔細研究過（汗..

我稍微搜尋一下，大部分的介紹文章都是2013, 2014 年的，過了三年，Web Components 的 API 都從 v0 來到了 v1。除了瀏覽器的支援程度外，React 跟 Vue 等等的崛起相信也是主因之一，畢竟，React 寫起來這麼爽快，實在沒必要去專研一個支援度低，複雜度也不低的技術啊！

等等，那我寫這篇要幹麻...

欸不是，都已經 2017 年了，新的一年總要有些新的開始，剛剛也提到，API 版本從 v0 進化到 v1 了，加上 ES6 的支援越來越好，許多寫法都更漂亮了，是個好時機來認識一下這個強大的 web 標準！

## 為何需要 Web Components ?

不論是什麼樣的程式，模組化在開發上都是很重要的一個概念，前端工程也不例外，我們都會希望能夠將 Web 開發過程中的 JS, CSS, HTML 封裝起來，以便後續重複使用，而過程中就會出現一些缺點：

1. js, css, html 三者的資源在最一般的網頁開發上基本上是分開管理，你如果想用別人的 library，可能還需要先載入對方需要的 JQuery，include 該套件需要的 css，最後才能在你的 HTML 內加入。

2. Scope 問題：即便是使用 React，都有可能遭遇到自己實作的 component 因為別人設定的 CSS class name 衝突而導致 Style 變動的情況。

3. 相容性：各個 Framework 間的 components 基本上無法共用，當你的舊專案想移植到 React 上頭時，即便我們先不管 React 在狀態管理的機制，很多元件要能夠相容的話，也可能需要透過 webpack 等各種 hack 的方式才能成功。

關於前兩點，透過 React, Webpack 與 [CSS Modules](https://github.com/css-modules/css-modules) 都可以找到相對應的解法，而第三點相信透過許多 hack 也是可以解決。

而 Web Components 則是希望透過規範，用更好更方便的方式，解決這些問題。

Web Components 是 [W3C 擬定的標準](https://www.w3.org/standards/techs/components#w3c_all)
由下列四大規範組成：

* HTML Templates
* Custom Elements
* Shadow DOM
* HTML imports

其中除了 `HTML Templates` 外，皆尚處在 Drafts 階段。
接下來會一一介紹。

不過這邊要先提醒一下，雖然剛剛用 React 來舉例比較，但實際上 React 與 Web Comoponent 真正要解決的問題是不同的，React 主旨在於 state 與 view 的狀態管理方式，而 Web Components 主力於整體元件的封裝，包含 Scoped CSS 等等。React component 內甚至可以載入 Web Components，畢竟 Web Components 可以只是 Plain JS, HTML, CSS。


### 先來個 Demo

在進入規範的介紹前，怕大家覺得我騙台錢，先給大家看一下一個最簡單的 Web Component 會長什麼樣子。有興趣的話再往下看！

<a class="jsbin-embed" href="http://jsbin.com/sijerap/embed">JS Bin on jsbin.com</a><script src="http://static.jsbin.com/js/embed.min.js?3.40.3"></script>

可以看到在 `<body>` 的最下方，有個 `<gakki-div-template>` 的 tag，這就是我 create 出來的一個簡單的 Web Components，tag 內的文字就會像 Meme 一樣顯示在圖片上方。而圖片以及標題文字與 CSS 都是封裝在裡頭，就算我在外面對一樣的 class name 設定不同 style，(即使是 !important）都不會影響到。

整段 code 我只有載入一個 polyfill 的外部 js 檔案 `webcomponents.min.js`，但如果你是用 Chrome 53 以上版本，基本上不需要也能正常運作。

另外，會寫 Vue.js 的讀者可能會覺得有點點眼熟，基本上 Vue.js 的寫法跟概念都跟 Web components 雷同，Vue.js 等於是自己寫了一套 Web Components，並且提供更好的瀏覽器支援，所以如果喜歡 Web Components 的朋友們，在標準真的被各家瀏覽器實作前，可以先用用 Vue.js！

## HTML Template

Template 的概念在許多 Web Framework 當中都有，像是 Django(python), EJS/Jade (Express/Node)，雖然用法皆有所不同，但概念都是共通的，而已經成為 HTML5 標準的 `<template>` 也不例外。

```html
<template id="mytemplate">
  <img src="" alt="great image">
  <div class="comment"></div>
</template>
```

以上述的 template 來說，我們在 html 裡面宣告一個 template，裡面包含一個 `<img>` 與 `<div>` 元件，在瀏覽器解析 HTML 文件時，template 的 DOM 物件並不會生效，裡頭包含的 Javascript 也不會執行，但是會產生一個 `cloneable` 的 DOM 物件，讓你在整個 web app 的生命週期都能使用。

來看一個簡單的使用情境：

假設我們要做一個簡單的 counter，計算 button 的點擊次數，然後每次都將次數新增到 button 下方的話，可以怎麼做？

可能會用字串串接的方式：

<a class="jsbin-embed" href="http://jsbin.com/mumacu/embed">JS Bin on jsbin.com</a><script src="http://static.jsbin.com/js/embed.min.js?3.40.3"></script>

但如果有了 template 的幫助，可以將 HTML 字串的部分與邏輯分開：

<a class="jsbin-embed" href="http://jsbin.com/yaxiyir/embed">JS Bin on jsbin.com</a><script src="http://static.jsbin.com/js/embed.min.js?3.40.3"></script>

source: [html5rocks-html-template](https://www.html5rocks.com/en/tutorials/webcomponents/template/)
```html
<button onclick="useIt()">Use me</button>
<div id="container"></div>
<script>
  function useIt() {
    const content = document.querySelector('#counter-template').content;
    // Update something in the template DOM.
    let span = content.querySelector('span');
    span.textContent = parseInt(span.textContent) + 1;
    document.querySelector('#container').appendChild(
        document.importNode(content, true));
  }
</script>

<template id="counter-template">
  <div>Template used: <span>0</span></div>
</template>
```

我們準備了一個 `template`，`id` 為 `counter-template`，在每次 button 被點選時，從 template 中取得目前的 `content`，將內容加一之後再 append 到 document 內。

雖然這個例子看起來，使用 template 反倒還要寫比較多 code，但是如果你今天需要產生的 template 是很複雜的 HTML 結構時，`template` 絕對可以為你帶來不少幫助的。

我們也可以直接複製 template 內容：

```js
const content = document.querySelector('#counter-template').content;  
const copyedContent = content.cloneNode(true);

document.body.appendChild(copyedContent);  
```

## Custom Elements

Custom Elements API 是構成 Web Components 的基礎之一，有了它的幫助，我們可以自己用最單純的 JS/HTML/CSS 來創造新的 HTML tags，或是 extend 別人的 components，甚至是擴展原有現存的 HTML tags；而也因為可以自行定義 tag 的名稱，讓標籤能夠更加語意化。

Chrome 在 version 33 時就有推出 v0 的 Custom Elements，而到了現在 version 55，已經採用 v1 的版本，雖然觀念是一樣的，但語法有許多差異，以下介紹皆為 v1 版本，並使用 ES6 的寫法。

要新增一個自定義的 HTML tag 很簡單：

```js
class GakkiTemplate extends HTMLElement {}
window.customElements.define('gakki-div-template', GakkiTemplate);
```

或是可以用暱名函式的方式

```js
window.customElements.define('gakki-div-template', class extends HTMLElement {});
```

兩種方式都可以讓你產生一個 `<gakki-div-template></gakki-div-template>` 的 tag。

其中我們用到 ES6 的 class 語法，來繼承 `HTMLElement`，這樣做可以讓我們的 `GakkitTemplate` 擁有所有 DOM API，並且讓你自己新增 Method 到你創建的這個 element 的 DOM interface 中。

拿最前面的 Demo 來舉例，宣告元素的地方我們可以修改成這樣：

<a class="jsbin-embed" href="http://jsbin.com/vuvoceq/embed">JS Bin on jsbin.com</a><script src="http://static.jsbin.com/js/embed.min.js?3.40.3"></script>

變動有點多，沒關係我們慢慢解釋：

```js
class GakkiTemplate extends HTMLElement {
    ...
    
  get light() {
    return this.hasAttribute('light');
  }

  set light(val) {
    // Reflect the value of the light property as an HTML attribute.
    if (val) {
      this.setAttribute('light', '');
    } else {
      this.removeAttribute('light');
    }
  }

  // Can define constructor arguments if you wish.
  constructor() {
   ...
  }
}
```

首先是 get/set，這兩個 method 代表賦予 `light` 這個 property getter/setter 的功能，這個要做什麼呢？

假設當別人使用我們的 custom-element 時，有可能他們會想要能透過給予 `light` 這個 property 來設定以下的 style：

```css

gakki-div-template[light] {
    opacity: 0.5
}

```

當然如果他直接設定 `<gakki-div-template light></gakki-div-template>` 就可以吃到 style，但多數時候我們會希望能用 JS 的方式來設置，像是：

```js
var gakkiDiv = document.getElementsByTagName('gakki-div-template');
gakkiDiv[0].light
// 此時會觸發 getter，return this.hasAttribute('light');

gakkiDiv[0].ligth = true;
// 此時會觸發 setter，this.setAttribute('light', '');

```
此時我們的 custom-elements 就需要給予 getter/setter 的 method，並在其中設定 `this.setAttribute()` 的方式讓我們的 elements 吃得到該 property 的設定。（ Note: property 的名稱跟 get/set 的名稱要相同）


再來是比較特別的兩個 method：

```js
static get observedAttributes() {
    return ['add'];
  }

...

// Only called for the change attributes due to observedAttributes
attributeChangedCallback(name, oldValue, newValue) {
    if (this.add) {
      this.addImg();
    }
}
```

透過定義 `static get observedAttributes()` 可以讓我們的 element 監聽該設定的 attributes 有沒有被 consumer 設置，如果有變動，會觸發 `attributeChangedCallback()` 來執行我們想要的動作。

那當然也可以加入自定義的 method，像是這邊的 `addImg()`；也可以在 constructor 的時候加入 EventListener，

```js
// Setup a click listener on <gakki-div-template> itself.
    this.addEventListener('click', e => {
     
      this.addImg();
    });
```

### Custom Elements - Life Cycle:

Custom Elements 有自己的生命週期，前述的 `attributeChangedCallback()` 也是其中之一：

* constructor：Custom Elements 建構式，通常會在這邊 create Shadow DOM。

* connectedCallback：當你插入元件到 DOM 時會被呼叫。

* disconnectedCallback：當你從 DOM 中移除元素時會被呼叫，可以在這邊 remove Event listener 之類的。

* attributeChangedCallback(attrName, oldVal, newVal)：監聽的屬性有變動時會被呼叫。

* adoptedCallback：整個 custom element 被人用 `document.adoptNode(el)` 呼叫時觸發。

可以玩玩看上述範例，看看我做了什麼無聊的功能 (畢竟只是範例麻 XD)。

除了 `HTMLElement` 外，你也可以繼承別人或是你自己的 Custom-Elements：

```js
class FancyDrawer extends AppDrawer {
  constructor() {
    super(); // always call super() first in the ctor. This also calls the extended class' ctor.
    ...
  }
}

customElements.define('fancy-app-drawer', FancyDrawer);
```

或是繼承 Native Element：

```js
// See https://html.spec.whatwg.org/multipage/indices.html#element-interfaces
// for the list of other DOM interfaces.
class FancyButton extends HTMLButtonElement {
  constructor() {
    super(); // always call super() first in the ctor.
    this.addEventListener('click', e => this.drawRipple(e.offsetX, e.offsetY));
  }

  // Material design ripple animation.
  drawRipple(x, y) {
    let div = document.createElement('div');
    div.classList.add('ripple');
  }
}

customElements.define('fancy-button', FancyButton, {extends: 'button'});
```
這邊要注意的是，定義繼承 Native element 的元件時，需要傳入第三個參數 {extendds: '{native element you extend}'}
表明你要繼承的元素是什麼，因為不同的 HTML tags 可能共享相同的 DOM interface，像是 `<q>` 與 `<blockquote>` 都是 `HTMLQuoteElement`。

而繼承 Native Element 的 Custom-Element 可以有另一種特殊使用方式（不過支援度更低 XD）：

```js
<!-- This <button> is a fancy button. -->
<button is="fancy-button" disabled>Fancy button!</button>
```

## Shadow DOM

Shadow DOM 算是 Web Components 中的靈魂角色，主要就是設計來建構 Component-based 的 web app，它所帶來的好處有下列幾項：

* Isolated DOM：在 Shadow DOM 裡面的任何 nodes 都不會被外面的 `document.querySelector()` 給取得，不會被汙染也不會去污染別人。

* Scoped CSS：定義在 Shadow DOM 內的 CSS 其作用域就只在 Shadow DOM 當中，不會作用於 Shadow DOM 外的 elements，而其他 Page 的 style 也不會影響到 Shadow DOM 內的定義。

* Composition：可以透過 Shodow DOM 賦予你的 component 擁有 Declarative, markup-based 的 API 可操作。

* Productivity：既然有 Isolated DOM 跟 Scoped CSS，就可以將你的 web app 切割成多個 DOM object 組成，完成模組化的使命！

### 名詞介紹

Shadow DOM：跟一般的 DOM 差異在於其產生的方式，以及他與頁面其他物件的互動方式。

shadow tree：一般來說你可以利用 `document.createElement()` 來創建 DOM，並 `appendChild()` 到其他 element 上，而 shadow dom 則是依附在某個 normal DOM 底下，產生一個 scoped subtree，稱作 `shadow tree`。

shadow host：掛載 shadow tree 的元素即為該 Shadow DOM 的 `shadow host`。

### Create Shadow DOM

其實 Shadow DOM 不一定要用在建構 Web components，要創建 Shadow DOM 只要用下列方式即可：

```js
const header = document.createElement('header');
const shadowRoot = header.attachShadow({mode: 'open'});
shadowRoot.innerHTML = `
      <style>#menu { ... }</style> <!-- styles are scoped to fancy-menu! -->
      <div id="menu">...</div>
    `; // Could also use appendChild().

// header.shadowRoot === shadowRoot
// shadowRoot.host === header
```
這樣一來，你就有一個 header 底下的 Shadow DOM 了，裡面定義的 #menu style，不會向內外影響。

但並非所有 DOM 都可以掛載 Shadow DOM，有些事已經有自己的 Shadow DOM （像是 `<textarea>`, `<input>`），有些是沒什麼必要（像是 `<img>`）。

### Create Shadow DOM for Custom Element

在 Web Components 中，Shadow DOM 都會搭配 Custom Element 出現，如果說 Custom Element 提供 Web Component 骨幹 (HTML, DOM interface)，Shadow DOM 就是提供血和肉 (JS, Scoped CSS)。

一樣以我們剛剛的範例來看，在 Custom Element 中創建 Shadow DOM：

```js
  constructor() {
    super();
    let shadowRoot = this.attachShadow({mode: 'open'});
    const t = document.querySelector('#gakki-div-template');
    const instance = t.content.cloneNode(true);
    shadowRoot.appendChild(instance);
    
    this.shadowDOM = shadowRoot;

    // Setup a click listener on <gakki-div-template> itself.
    this.addEventListener('click', e => {
      this.addImg();
    });
  }
```

這邊有幾點要說明：

1. 利用 `this.attachShadow({mode: 'open'})` 來綁定 Shadow DOM 到目前的 Custom Element，其中的 `{mode: 'open'}` 是代表這是個 open mode 的 Shadow DOM，host 這個 Shadow DOM 的 Element (以這邊的例子就是我們的 Custom Element）可以透過 JS 取得 Shadow DOM 內部的 DOM 元件，反之，如果設置為 close，即無法取得，官方不建議我們設定為 close，詳情可以參考 [這裡](https://developers.google.com/web/fundamentals/getting-started/primers/shadowdom)

2. 透過 `const t = document.querySelector('#gakki-div-template');` 我們取出 template，並利用 `t.content.cloneNode()` 複製一份 template，接著將此 template appendChild 到我們的 shadow tree 中。

透過這樣的方式，我們就擁有一個擁有 Scoped template 的 Custom Element，因為整個 Element 的內容都是透過 template 與 Shadow DOM 產生的，別人可以自由拿去使用，也不用擔心他們會污染到這個元件！

### Slot element

在我們範例的 template 中有個奇妙的元素 `slot`

```html
<template id="gakki-div-template">
  <style>
    ...
  </style>
  <p>Gakki Meme</p>
  <div class="wrapper">
    <img width="300px" src="http://static.ettoday.net/images/2083/d2083850.jpg" />
    <div class="slot">
      <slot></slot>
    <div>
  </div>
</template>
```

它的用途很簡單，就是一個 placeholders，讓元件的 consumer 可以安插自己的 DOM 進去，而你透過 slot 的位置來決定這些 user 的 DOM 該放在哪些位置。

在範例中：

```html
<gakki-div-template light>你好，我是森山</gakki-div-template>
```

我們將文字 '你好，我是森山'，放入 custom element，他就會被我們安插到 `<slot></slot>` 的位置

如果有兩個以上的元素要插入的話，就需要用 `name` 來綁定：

```html
<gakki-div-template light>
  <span slot="title">你好，我是森山</span>
  <div>嗨..妳好>//< </div>
</gakki-div-template>
```

```html
<template>
    ...
    <p>Gakki Meme</p>
    <div class="wrapper">
        <img width="300px" src="http://static.ettoday.net/images/2083/d2083850.jpg" />
        <div class="slot">
            <slot name="title"></slot>
        </div>
        <div class="slot2">
             <slot></slot>
        </div>
    </div>
  ...
</template>
```
<a class="jsbin-embed" href="http://jsbin.com/jezepof/embed">JS Bin on jsbin.com</a><script src="http://static.jsbin.com/js/embed.min.js?3.40.3"></script>

你們也可以打開 DevTool 來看一下 Shadow DOM 的元件展開會是長什麼樣子，這邊就不再贅述。

### Shadow DOM Styling

接下來介紹如何在 Shadow DOM 中設定 CSS。

在 Shadow DOM 中的 CSS 其實跟一般使用上沒有太大差異，你所設定的 CSS selector 都是 local 的，也就是只能設定到 Shadow DOM 裡頭的 element，不會受外部影響，也不會影響到外部。

比較需要說明的有三個部分：

1. `:host`：

你可以用 `:host` 這個 selector 來設定你的 component 的 style，但要注意的是，`:host` 所設定的值，是可以被外部 component 的使用者蓋過去的。

像範例中為了讓 slot 的字能絕對定位，我需要把 `<gakki-div-template>` 本身設定為 `relative`

```css
:host {
      position: relative;
      display: block;
      width: 300px;
    }
```

2. `:host-context`：
讓你根據 Component 的 Context 來設定 style，像是：

當處於 darktheme class 下時，顏色要變成白色。
```html
<body class="darktheme">
  <fancy-tabs>
    ...
  </fancy-tabs>
</body>
<style>
    :host-context(.darktheme) {
      color: white;
      background: black;
    }
</style>

```

3. `::slotted`：
最後則是透過 `::slotted`，來控制傳入的 slot 元件的 style。

像範例中：

```html
    <template>
        <style>
            ...
            ::slotted(#me){
              color: blue;
            }
            ...
        </style>
        ...
    </template>
    ...
    <gakki-div-template light>
      <span slot="title">你好，我是森山</span>
      <div id="me">嗨..妳好>//<</div>
    </gakki-div-template>
```

### Event Handle in Shadow DOM

Shadow DOM 還有許多 Event 處理，包含 Slot 的 event 處理等等 issue 可以探討，這邊只簡單講個主要觀念。

在 Shadow DOM 中對於事件處理，是透過在 Event Bubble 的過程中，來重新定位 event target 的位置，讓該事件像是從 Host 的元件觸發，並且會擋掉可能影響到外部頁面的事件處理。

詳細可以看 [這邊](https://developers.google.com/web/fundamentals/getting-started/primers/shadowdom#historysupport) 與 [這邊](http://www.jianshu.com/p/e8994b92bb7a)

## HTML Import

Web Component 的最後一哩路，當我們製作好我們的 Component 後，當然會希望能很方便地給他人使用，HTML Import 就是要處理這樣的問題。

他讓我們可以直接 link 一份 HTML 檔案，不管他是 template 也好，用 shadow dom 創建的 custom-element 也好，都可以直接 inlcude。

使用方法很簡單，假設我們範例中的 HTML 叫做 `GakkiMeme.html`，別人想要 include 的話，只要在他的 index.html 中加入 `<link rel="import" href="GakkiMeme.html">` 即可使用：

```html index.html
<head>
  <link rel="import" href="GakkiMeme.html">
</head>
<body>
  <gakki-div-template>
    <span slot='title'> 嗨嗨嗨 </span>
    <div id="me"> 嘿嘿嘿 </div>
  </gakki-div-template>
</body>
```
當然 Import 的部分還有很多可以玩，有興趣的可以到 [HTML5rocks](https://www.html5rocks.com/en/tutorials/webcomponents/imports/) 研究。


## Conclusion

Web Components 提供我們一種方式來建構 reusable components，能支援 Cross-browser (當然要等標準全通過，瀏覽器全支援，但相信那天會到來的！)；不需要任何 framework 支援；也不需要學特殊語法，就是 DOM/CSS/HTML/JS；可以跟現有 Framework 共存使用。

相信在未來我們會有更美好的 Web 可以使用！

最後附上 Browser Support 的 Information 給各位：

[custom-elementv1](https://developers.google.com/web/fundamentals/getting-started/primers/customelements#historysupport)
[shadow-dom](https://developers.google.com/web/fundamentals/getting-started/primers/shadowdom#historysupport)



## 資料來源
1. [W3C Web Components](https://github.com/w3c/webcomponents)
2. [Custom Elements v1](https://developers.google.com/web/fundamentals/getting-started/primers/customelements)
3. [Shadow DOM v1](https://developers.google.com/web/fundamentals/getting-started/primers/shadowdom)
4. [Web Components整理及分享](http://www.jianshu.com/p/e8994b92bb7a)
5. [Web Components 初探](https://blog.hinablue.me/web-components-first-look/)
6. [basic-web-components](https://github.com/basic-web-components)
7. [HTML Imports](https://www.html5rocks.com/en/tutorials/webcomponents/imports/)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化