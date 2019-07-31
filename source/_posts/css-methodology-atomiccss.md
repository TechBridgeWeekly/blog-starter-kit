---
title: 淺談 CSS 方法論與 Atomic CSS
date: 2017-04-29 19:05:31
tags:
  - css
  - atomic css
  - methodology
---
<p data-height="300" data-theme-id="29194" data-slug-hash="bWqOeK" data-default-tab="result" data-user="arvin0731" data-embed-version="2" data-pen-title="css-is-awesome" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/bWqOeK/">css-is-awesome</a> by Arvin (<a href="http://codepen.io/arvin0731">@arvin0731</a>) on <a href="http://codepen.io">CodePen</a>.</p>
<script async src="https://production-assets.codepen.io/assets/embed/ei.js"></script>

今天來點輕鬆的，看標題就知道我想介紹一下 [Atomic CSS](https://acss.io/)，這是一套由 Yahoo 開源的 CSS 工具，工作上使用了蠻長一段時間，一開始使用起來其實覺得蠻不習慣的，
但是久了以後發現搭配 React 寫起來雖然醜了點但是方便又易懂，非常適合獨立作業的前端工程師（設計師不參與 HTML, CSS 切版等動作），待我稍後慢慢說來。

在介紹 Atomic CSS 之前，我想順便複習一下現今的 CSS methodology，以及 React 出現後對 CSS 的影響，進而帶出 Atomic CSS 想解決的問題與其帶來的好處。後續的一些說明多參考自許多網路資源，附錄在文章最後面。

## CSS 架構心法

剛開始接觸前端時，對於 CSS 也不太會去思考什麼架構，覺得就自己 class name 命名清楚一點，檔案整理好一點就好，但是這幾乎僅適用於專案規模還算小的時候，一但開發的專案龐大起來，並且有多位前端工程師在進行程式碼撰寫時，就很容易遇到命名衝突、stylesheet 過於龐大等問題，主因是在 CSS 的世界中，所有規則集都是全域的。（註：規則集 (ruleset) - 由一個宣告區塊所涵括的一或多個選擇器所組成, ex: `modal-text { color: #000, font-size: 12px }`）

為了更加明確的管理 CSS，開始有人提出一些 CSS 的架構心法，想讓 CSS 也能有良好的**重用性**、**維護性**與**延展性**。

比較有名的 CSS 架構心法大致上分為這三種：

* OOCSS
* SMACSS
* BEM

有份流傳已久的投影片在說明這三種心法：[漫談 CSS 架構方法 - Kuro Hsu](https://www.slideshare.net/kurotanshi/css-oocss-smacss-bem)

這邊我就簡單節錄重點：

<span style="font-size: 2.5rem; color: #ba0707;">OOCSS</span>

身為工程師，看到 OO 兩個字一定就會想到 Object-Oriented 吧，OOCSS 主意就在於將 CSS 物件化、模組化，其主要原則有兩個：

<span style="font-size: 2rem; color: #2626cc">1. Separation of Structure from Skin: </span>

Structure 可以看作是 CSS 中定義元素的 box-modal 大小、margin 與 position 的部分，而 Skin 自然就是表現性的 Style，像是顏色、字型大小、border-color、box-shadow 等等，在 OOCSS 的原則中，這兩部分的 CSS 不能混合在同一個規則集中。

Example:

一般在定義一個 div 的長相時，直覺就會寫出下列這種 CSS，根據該 div selector 定義好其大小、位置與顏色

```css
#modal {
    width: 500px;
    height: 500px;
    box-sizing: border-box;
    padding: 20px;
    border: solid 1px #1ED3A9;
    background: linear-gradient(#09D083, #1ED3A9);
    box-shadow: rgba(0, 0, 0, .5) 2px 2px 5px;
    overflow: hidden;
    position: fixed;
    top: 50%;
    left: 50%;
    transform: translateX(-50%);
}

#button {
    width: 100px;
    height: 30px;
    box-sizing: border-box;
    padding: 20px;
    border: solid 1px #1ED3A9;
    background: linear-gradient(#09D083, #1ED3A9);
    box-shadow: rgba(0, 0, 0, .5) 2px 2px 5px;
}
```

而 apply OOCSS 的第一原則後，可以修改成如下，將共用的表現型 Style 抽取出來，並且以 class 取代 id 作為 selector，讓其可以 reuse：

```css
.modal {
    width: 500px;
    height: 500px;
    box-sizing: border-box;
    padding: 20px;
    position: fixed;
    top: 50%;
    left: 50%;
    transform: translateX(-50%);
}

.button {
    width: 100px;
    height: 30px;
    box-sizing: border-box;
    padding: 20px;
}

.defaultTheme {
    border: solid 1px #1ED3A9;
    background: linear-gradient(#09D083, #1ED3A9);
    box-shadow: rgba(0, 0, 0, .5) 2px 2px 5px;
    overflow: hidden;
}
```

<span style="font-size: 2rem; color: #2626cc">2. Separation of Containers and Content:</span>

關於 OOCSS 的第二原則，白話來說就是要求你將 css 與 html 盡量切割，以可共用的 class selector 來定義 style 並放入該 html 元素中。

Example:

假設我們定義 Header id 底下的 h1 要是如下 style：

```css
#header h1 {
    font-size: 2rem;
    color: #1ED3A9
}
```

假若之後想要在不同地方的 h1 有不同顏色，但同時保有相同 size 呢？你得這樣複寫：

```css
#header h1, #footer h1 {
    font-size: 2rem;
    color: #1ED3A9
}

#footer h1 {
    color: red;
}
```
這樣不僅是有重複的 style，更是難以維護，以 OOCSS 的角度來看，若將這些共用的 style 另外包成 class，最後在 apply 到需要的 html 上，會清楚許多。

```html
<div class="header commonFontSize"></div>
<div class="footer commonFontSize"></div>
```

```css
.commonFontSize {
    font-size: 2rem;
}

.header {
    color: #1ED3A9
}

.footer {
    color: red;
}
```

OOCSS 的指標人物 Nicole Sullivan 有個 [media object](https://github.com/stubbornella/oocss/wiki/Content) 的 reusable module 以 oocss 概念實作，大家可以參考。

總結一下 OOCSS 優點與實作方針：

優點：更小的 css size，能讓網站加速；更方便管理模組化的 css stylesheet。
實作方針：避免 descendent selector 與 id selector，使用 class 並盡量與 html 元素綁定。
**Don' do**: `#button h3`, `span.title`

<span style="font-size: 2.5rem; color: #ba0707;">SMACSS</span>

SMACSS 有線上的官方電子書 [Scalable and Modular Architecture for CSS](https://smacss.com/book/)

看名稱就知道是以整體專案的 Architecture 來考量，與 OOCSS 我覺得是相同概念，只是關注點的起始位置不同，從不同瀏覽器對於基礎元件的 style 就開始考量，除了與 OOCSS 相似但有規範的 **結構分類** 與 **CSS與HTML分離**，還多了命名規則的限制。

### 結構分類：

* Base
* Layout
* Module
* State
* Theme

### Base:
定義頁面中HTML Element的最基本Style，包含CSS Reset(一致化各瀏覽器自定義的 style)，因此只會用到 element tag selector。

### Layout:
所謂 Layout 就是將頁面切割定義成不同的區塊，像是 naviagtion、header、sidebar 等等，由於這些區塊大多獨立出現在頁面，因此用 ID 宣告是 ok 的，但若是有重複區塊類型，但不同 style，則可以採用 class 加上 cascade 來處理。

```css
#sidebar {
    width: 30%;
    float: right;
}
.l-fixed #sidebar {
    width: 10%;
}
```

### Module:
Module 基本上與 Layout 相同，都是頁面上的區塊，只是偏向於 Content，但是嚴禁使用 id 或是 element selector，只准使用 **class selector**，可以透過命名的方式來管理這些 class，即 `Subclassing` 或 `Sub module`:

```css
.modal-body { width: 100% } /* 用 dash 分隔 class (subclassing/submodule)*/
.modal-header { height: 50px; width: 100% }
```

### State:
State 顧名思義就是根據元件的狀態給予不同的 style，因此命名上，針對該狀態的描述越精準越好：

`<div class="modal-button active"></div>`

```css
.active { color: red; }
```

### Theme:
這個很好懂，其實就是針對網站主視覺定義好各種 Module 或是 Layout 需要的 Style，像是 Bootstrap、Material-UI 中也有類似概念。

這邊稍為總結一下 SMACSS 的優缺，更多細節可以參考電子書：

優： 根據結構分類，並定義出 Base style，最小化各瀏覽器的差異，遵守其 Layout、Module、State 規則可以有良好的重用性與維護性，並分離 CSS 與 HTML，進而幫助簡化 selector 深度，增加效能、減少 Size。
缺：與 OOCSS 一樣，可能會造成 Class 定義過多

<span style="font-size: 2.5rem; color: #ba0707;">BEM</span>

最後一個心法是 BEM，核心觀念與現今流行的 React, Vuejs 相像，強調模組化與 css 的重複利用性，因此只使用 Class selector，以其特有的命名規則來規範。不像先前 OOCSS 或 SMACSS 的 class 可能會讓你命名出 MargintTop-10 這樣以 skin 為主的 class name，BEM 以**功能導向**來命名，將網頁組成分為 *Block*, *Element*, *Modifier*。

### Block：
就是一個獨立並可重複使用的頁面元件，如同 SMACSS 的 Layout/Module，命名若有需要則以 dash (-) 串接
`.search-field {...}`。

### Element：
是 Block 中不可分離的小元件，一定存在於 Block 下，但 Block 不一定會有 Element。
因為一定存在於 Block 中，因此命名會一定有 Block Name 作為 prefix，以<span style="font-weight: 600; color: red;">雙底線</span>分隔 :

.search-field<span style="color:red">__button</span> {...}

![BEM example](/img/arvinh/BEM_BE_example.png "BEM example")

### Modifier：
用來定義 Block 或 Element 的狀態或屬性，像是 SMACSS 的 State，可以多個 modifier 同時存在於 Block 與 Element 中。命名則以 Block 或 Element name 作為 prefix，以<span style="font-weight: 600; color: red;">單底線</span>分隔

.search-field__button<span style="color:red">_hover</span> {...}

另外，BEM 甚至提出了依照 BEM 的架構來區分 file structure：
（截圖至[官網](https://en.bem.info/methodology/quick-start/#file-structure)）
![BEM file structure](/img/arvinh/BEM_file.png "BEM file structure")

## 結論與轉折

CSS 心法除了上述三種以外，其實還有 [SUIT CSS](https://suitcss.github.io/) 等等，不過就大同小異，主要都是希望提高 CSS 的重用性、可維護性與延展性，發展至此似乎趨於穩定，搭配 SASS 等工具幾乎已經能很好的管理 CSS 了，但是從 React 出來以後，其推薦的 CSS in JS 根本打翻了上述的哲學，既然要用 JS 來寫 component，那 CSS 直接用 inline-style 的方式寫在 jsx 中，就不用記一堆有的沒的命名規則，又不用擔心全域變數影響，多棒啊！

但從來沒有完美的解法，有很多人討厭這樣的做法，所以出現了 [Radim](http://stack.formidable.com/radium/) 或 [CSS-module](https://github.com/css-modules/css-modules) 這樣的東西，算是蠻完美的利用 Scoped css 
來做到擁有原始 css style 使用彈性的 CSS in JS。

不管是用哪種方式，朝向模組化、Scoped CSS 的方向看來都是不變的。

<span style="font-size: 2.5rem; color: #ba0707;">Atomic CSS</span>


既然我們複習了 CSS 心法，也了解到目前因應 React 的發展而出現的 CSS-module 等方式，我們就可以來介紹一下由 Yahoo 這個曾經的網路巨人所開源的 Atomic CSS 吧！

**這邊要特別說明一下，Atomic CSS 並不是來解決上述心法的缺點，要解決的問題都雷同，都是希望能讓 CSS 在大型專案下能擁有更好的重用性與維護性，只是採用的方法與面向不同罷了。**

透過前述心法我們可以利用 class selector 的方式來處理命名衝突的麻煩，但是還有可能造成 stylesheet 過大，因為你可能會依據不同 component 來設置不同的 namespace，而且一個不小心，若 CSS 階層越多，效能就會越差。加上不同團隊一起開發時，可能還會有不同命名，卻有相同效果：

```css
block1__text_highlight {
  color: yellow;
}

...

block2__text_bright {
  color: yellow;
}
```

因此 Atomic CSS 提出另一種觀點：

### 將 CSS style 最小元件化，重用性最大化

只要確保同一個 style 永遠只會被定義一次，並且可以運用在各個地方，就能解決這些問題！

實際作法就看一下範例吧：

```html
<div class="D(f) W(100px)">
</div>
```

利用 Atomic css 的工具，會幫你將上述 html 中的 class name 解析成：

```css
.D\(f\) {
  display: flex;
}

.W\(300px\) {
  width: 100px;
}
```

應該很淺顯易懂吧！`D(f)` 對應到 `display:flex` 這個 style，也就是說，Atomic 以一種 css style 作為 class name 的最小單位。

再稍微想一下你就會發現，這根本就是在寫 inline-style，只是我們用 class name 的方式來表示而已啊！

沒錯，但這樣做的好處就是可以 <span style="color:red">Define once, use everywhere.</span>

今天你就算有另一個 div 也想要有 display:flex 的屬性，只要加上 `D(f)` 這個 class name 就可以了，同樣 style，不用重複定義 class name！

**在大型專案內，你的元件越多、重複的屬性用得越多，相對於其他心法，你就可能省下更多 Size！**

而且這樣的寫法，搭配 React 真的很方便，也符合原先 CSS in JS 的概念，透過串接多個 "Atomic Class" 的方式在元素上來達到原先 css style 的效果。

來看個實際的 Example：

```jsx
import React from 'react';

class Modal extends React.Component {
    render() {
        return (
            <div className="P(10px) M(20%) Pos(f) Start(50%) Bgc(#fff)" />
        );
    }
};

export default Modal;
```

Atomic css tool parsed 過的 css

```css
.Bgc\(#fff\) {
  background-color: #ffffff;
}
.M\(20\%\) {
  margin: 20%;
}
.Pos\(f\) {
  position: fixed;
}
.Start\(50%\) {
  left: 50%;
}
.P\(10px\) {
  padding: 10px;
}
```

有發現嗎？從此以後你要看一個 component 的時候，可以直接從 jsx 檔案中看完整個元件的狀態與樣式，不用再切換 jsx 與 css 檔案了，而且透過這樣的 class name 命名，只要稍微熟悉以後，就超級好懂這個 div 用了哪些 style。（就算不懂，官網也有很方便的[查閱工具](https://acss.io/reference)）。

最後列舉一下優點：

1. 將 class name 定義最小化，讓全站都能重複使用。
2. 透過 Atomic CSS 的 parser（or webpack loader），只會產生你有使用到的 classname 的 stylesheet。
3. 比起 inline style 的寫法更簡潔，又不會有命名衝突。
4. 加上此種 class name 很好壓縮，整體 size 可以很小。
5. 搭配 React，從此 component 的狀態與樣式合為一體。

我知道會有很多人覺得這樣違反直覺、寫起來很醜、沒有語意化等等，我一開始也閃過這種念頭，
但是身為工程師，我們擅長打破常規、利用創意來解決問題。
這些 class name 的確不語意化，但是身為工程師，我想我們擅長理解這些代號。
寫程式都知道，語言只是工具，邏輯才是重點，如果可以避免，可以不必花這麼多時間在思考命名。

當然，以上只是我的觀點啦～推薦大家試用看看！

同場加映另一個類似概念，但處理機制不太相同的 [Tachyons css](http://tachyons.io/)

## 資料來源
1. [Intro-to-OOCSS](https://www.smashingmagazine.com/2011/12/an-introduction-to-object-oriented-css-oocss/)
2. [漫談 CSS 架構方法 - Kuro Hsu](https://www.slideshare.net/kurotanshi/css-oocss-smacss-bem)
3. [Scalable CSS - 介紹OOCSS/SMACSS/BEM](http://sj82516-blog.logdown.com/posts/1077348/finish-css-intro-oocss-smacss-bem)
4. [Scalable and Modular Architecture for CSS](https://smacss.com/book/)
5. [BEM](https://en.bem.info/methodology/quick-start/#file-structure)
6. [Atomic css 介紹](http://tom76kimo-blog.logdown.com/posts/737611-atomic-css-introduction)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化