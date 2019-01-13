---
title: 一起探討 Micro Frontends 的世界
date: 2019-01-12 18:00:17
tags:
  - frontends
  - micro
---

# 前言

大概三個月前，忘了在哪看到 Micro Frontends 這個詞，當時沒有仔細去了解（畢竟聽起來就像是另一個 Buzzword XD），只是從名字大概猜測是想讓前端頁面更加模組化，讓不同團隊能更有效率的開發共同的站。稍微 Google 了一下這個 term，發現文章多數在 2018 的五六月出現，之後就鮮少人提起，我也就淡忘了他。

直到最近工作上接觸到的框架，似乎也想讓頁面上的元件更加低耦合，甚至是希望能支援運行不同版本或是 tech stack 的元件，才讓我又再想起 Micro Frontends 這個東西，所以查了點資料，這邊做個紀錄分享。

# 什麼是 Micro Frontends

大部分查到的文章都有提到，Micro Frontends 最早出現在 [2016 年 ThoughtWorks 這間公司公佈的技術雷達](https://www.thoughtworks.com/radar/techniques/micro-frontends)中，並在 2018 年五月進入試驗階段，代表他們認為這個架構是有發展淺力的。（查了一下 [wiki](https://en.wikipedia.org/wiki/ThoughtWorks)，ThoughtWorks 是一間蠻大的全球軟體顧問公司，有自己的產品，也會為客戶制定各種前瞻軟體設計）

要解釋 Micro Frontends，可以先從 Microservices 講起。

Microservices 在後端的世界裡面已經運行多年，採用 Microservices 的團隊架構通常都是如下圖所繪：

![monolithic-fe-micro-be](/img/arvinh/monolithic-fe-micro-be.png)

後端中每個 Microservices 彼此之間獨立作業，各個團隊可以擁有自己的部署與開發技術，溝通可以透過各樣 API 介面來達成，與前端之間也只需要 HTTP request 即可進行各種服務串接。

在這樣的架構下，Frontend team 的程式碼卻還是都共存在一個 Monolithic 的結構內，當網站功能越趨複雜、團隊成長茁壯後，整個前端架構就會越來越難以維護，更別提前端技術的日新月異，很容易會產生 legacy code，這時若想要更新就麻煩了。

由於慢慢有人體會到這種問題，Micro Frontends 的想法才被提出：

![micro frontend](/img/arvinh/micro-fe-micro-be.png)

從圖中不難發現，Frontend 的模組被拆分至各個 team 中，每個團隊獨立掌管自己的前後端服務，擁有自己的部署環境與 tech stack，團隊間低耦合，團隊中高內聚。
此外，每個團隊產出的前端模組，要能夠有效的『拼貼』在同一個 SPA 頁面當中，保持使用者在產品的體驗上與原先的 SPA 一致。

在 [micro-frontends.org](https://micro-frontends.org/) 這個網站中說到，這樣的結構就算是 Micro Frontends，而這樣的概念其實更早之前就有，只是有別的名稱，分別叫做 [Frontend Integration for Verticalised Systems](https://dev.otto.de/2014/07/29/scaling-with-microservices-and-vertical-decomposition/) 與 [Self-contained Systems](http://scs-architecture.org/)。

但我自己認為，Micro frontend 應該主要著重在：如何將前端頁面中以功能模組來拆分成不同獨立應用，並黏貼共存在同個 SPA 中。(後面會再提到目前常見的實作方式。)

而由 Micro Frontends 與 Micro Services 所組成的一個從 UI、商業邏輯、資料處理和系統部署，這樣完整的獨立服務，才算是一個 [SCS - Self contain system](https://scs-architecture.org)。

## Micro Frontends 的核心思想

* **Be Technology Agnostic**：每個團隊可以使用自己的技術結構來開發前端模組，與其他團隊彼此之間互不干擾，也無須耗費成本相互協調。

* **Isolate Team Code**：就算各個團隊採用同樣的技術框架，彼此之間也不該共享變數或狀態。彼此間應該透過 public API 來溝通。

* **Establish Team Prefixes**：利用 Prefix 的方式來避免 CSS、Browser API、Web Event、Cookies 或 Local Storage 的衝突。

* **Favor Native Browser Features over Custom APIs**：要整合 Micro Frontends 的複雜度其實很高，當每個模組間需要溝通時，盡量採用 Browser Native API 來完成較好；若真的需要額外的溝通方式（pub/sub system），要盡量保持簡單。

* **Build a Resilient Site**：透過 SSI 或 PWA 的方式增強網站的穩定性，在 JS 無法執行的狀況下也有堪用的呈現。

# Micro Frontends 的實作方法

![PoC demo](/img/arvinh/three-teams.png)
[source](https://micro-frontends.org/)

接著我們先來看看要如何實現 Micro Frontends，達成如上圖所呈現的狀態，在同個頁面中，運行三個不同 Team 採用不同 Tech stack 的模組。

Team checkout 與 Team inspire 各自開發了前後端整合的模組，而 Team product 要負責整合它們。

三個 Team 採用不同 tech stack，並且獨立部署在各自的環境中運行，基本上可能會有三個 Host 分別 serve 它們：

1. https://microfrontends-checkout.com/

2. https://microfrontends-inspire.com/

3. https://microfrontends-product.com/

## 方法一：使用 iframe

在 Team product 的模組中，可以利用嵌入 iframe 的方式來載入另外兩個模組，由於 iframe 天生就有隔離運行環境的特性，各團隊的模組相互不會干擾，若要通訊，在同網域下，我們更能直接透過 `window.postMessage` 來達成。

```html
<body>
  <!-- in Team Product -->
  <iframe width="100%" height="200" src="https://microfrontends-checkout.com/"></iframe>
  <iframe width="100%" height="200" src="https://microfrontends-inspire.com/"></iframe>
</body>
```

**缺點**:

但使用 iframe 的缺點很多，基本上不會有人想採用這種方法...
像是：由於應用模組是分開的，無法將共用的依賴模組取出來，導致頁面可能會同時載入重複的 code；再者，UI 的呈現上也會變得很難控制，若是其中還含有表單之類的功能，就更麻煩了。

## 方法二：在 Client side 用 JS 載入模組

```js
function loadPage (element) {
  [].forEach.call(element.querySelectorAll('script'), function () {
    const script = document.createElement("script");
    script.setAttribute("src", `https://microfrontends-${element.dataset.url}.com/`);
    script.setAttribute("type", "text/javascript");
    element.appendChild(script);
  });
}
document.querySelectorAll('.pagelet').forEach(loadPage);
```

```html
<!-- in Team Product -->
<div class="pagelet" data-url="checkout"></div>
<div class="pagelet" data-url="inspire"></div>
```

也就是在 client side ajax 抓取模組，塞入對應的 target div 內。這種做法需要注意 js、css 等的載入順序，Facebook 在多年前是用 [BigPipe](https://www.facebook.com/notes/facebook-engineering/bigpipe-pipelining-web-pages-for-high-performance/389414033919/) 來處理（可能 FB 已經沒在使用，但還是很值得了解的一個專案！）：

```js
<script type="text/javascript">
big_pipe.onPageletArrive({id: “pagelet_composer”, content=<HTML>, css=[..], js=[..], …})
</script>
```

類似這樣，用 array 來依序載入資源。

但缺點明顯就是無法 Server-side render。

## 方法三：Web component

這邊是指 Web standard 下包含 `Custom Element`、`HTML template`、`Shadow DOM` 與 `HTML imports` 的 Web component。

各個團隊可以用自己想使用的 tech stack 來製作元件，但最終包裹成 Web component 的形式，以 React 為例：

```js
import React from 'react';
import ReactDOM from 'react-dom';
const SearchComponent = ({ children }) => {
  return (
    <div>
      <p className="search-component">{children}</p>
    </div>
  );
};
class CustomSearch extends HTMLElement {
  connectedCallback() {
    const mountPoint = document.createElement('span');
    this.attachShadow({ mode: 'open' }).appendChild(mountPoint);
    const name = this.getAttribute('name');
    const keyword = this.getAttribute('keyword');
    const url = 'https://www.google.com/search?q=' + encodeURIComponent(name);
    ReactDOM.render(
      <>
        <a href={url}>{name}</a>
        <SearchComponent>{keyword}</SearchComponent>
      </>,
      mountPoint,
    );
  }
}
customElements.define('custom-search', CustomSearch);
```

```html
<!-- in Team Product -->
<!-- search component from Team Inspire -->
<custom-search name="TeamInspire" keyword="Micro Frontends"></custom-search>
```

採用 Web component 可以讓程式碼清晰可讀，元件之間各自獨立，所有資源都由自身控制該如何加載，團隊在使用這些元件上，就如同一般 HTML DOM，並且可以直接利用 Web Event API 來進行元間間的溝通。例如：

```js
// in Team checkout
const event = new CustomEvent('buy', { item: 'car' });
window.dispatchEvent(event);
```

In React:

```js
componentDidMount() {
  window.addEventListener('buy', (event) => {
    this.setState({ buyItem: [...this.state.buyItem, event.item] });
  }, false);
}
```

此外，如果 Web component 還能搭配 SSI (Server Side Includes) 來達成 SSR：

```html
<custom-search keyword="Micro Frontends">
  <!--#include virtual="/custom-search?keyword="Micro Frontends" -->
</custom-search>
```

```js
server {
  listen 3000;
  ssi on;

  location /TeamInspire {
    proxy_pass  https://microfrontends-TeamInspire.com;
  }
```

每個 Micro Frontend team 自己要建立起 `component server`，用來 serve component 的 markup。並且實作的 Web component 本身也要修改成支援 Server side render。
不過重點是，這樣在技術上是可行的，至少比上一種方式好。

但缺點也很明顯：

1. 需要 polyfills 來補足瀏覽器支援度問題。
2. 雖說大家能用不同 Tech stack，但本質上每個團隊最終還是得想辦法用成統一的 Web component
3. Web component 的發展與普及不夠快速。

關於 Web component 實作 Micro Frontends 的範例與細節，大家可以到 [micro-frontends.org](https://micro-frontends.org/) 看，範例 code 甚至都包成 docker，很快就能裝起來玩！

## 方法四：[Single-SPA - javascript metaframework](https://github.com/CanopyTax/single-spa)

CanopyTax 這間公司推出的一個開源專案，提供了一些方便的 API 讓你達到：

1. 用各種技術構建 micro frontends。

2. 在同個頁面下使用多種前端 framework 組成的模組，且無需 refresh page。

3. 在現有的應用中嘗試新的 framework，而不必全部重寫。

4. 支援 Lazy load

5. 支援 Hot reload

有興趣的讀者可以前往他們的 [repo](https://github.com/CanopyTax/single-spa) 看各種範例與程式碼。類似這樣的使用方式：

```js
import * as singleSpa from 'single-spa';
const loadingFunction = () => import('./react/react.app.js');
// 可根據 route 來切分要換成哪個模組並動態載入相應資源
const activityFunction = location => location.hash.startsWith('#/react-app');
singleSpa.declareChildApplication(appName, loadingFunction, activityFunction);
singleSpa.start();
```

## 其他方法：

在查資料的過程中，其實大家在討論的 Micro Frontends 都會著重在各 Team 間採用不同 tech stack 的前提，可能是這樣才比較好凸顯 Micro Frontends 切割模組的感覺。

但是如果只是想要讓 Microservices 都有各自對應的前端模組，我們其實也可以直接用 React 來建造各類元件庫，讓主 App 下載使用。

這樣會是一個折衷的方案，不用煩惱 common dependences 的問題，但相對的在升級版本時，可能就得一次將所有元件庫內的元件都升級才可以使用。

有點偏離 Micro Frontends 的核心就是了。

## Micro Frontends 帶來的好處與壞處

Micro Frontends 的好處就在於，各個模組間是非常獨立的，彼此間的部署不需要互相等待，tech stack 也不一定要用相同的，也就是說，在汰換 legacy code 的過程可以順暢一些，也更容易嘗試新技術，可以漸進式的把 SPA 上的模組替換成更好或更新的框架，讓新舊 code 能夠並存無衝突。團隊在開發上可以更敏捷。

但實務上我想應該不太會有人想在同個頁面上同時存在多種框架吧？試想，一個頁面上如果同時存在 React 與 Vue，甚至是 Angular，這樣 Page load time 大概難以想像。
再加上缺乏統一打包的步驟，共用資源的相依性就無法被知曉，要如何有效處理 Common resources 是很麻煩的難題。

還有就是 CSS 解決方案，會不會需要處理命名衝突，也是一個隱憂。若是公司資源不夠，沒有辦法利用像 Zeplin 之類的工具了統一定義 Style guideline，就還得想辦法協調各 Team 開發模組的 style 問題。

整體而言看起來 Micro Frontends 的複雜度並不小，應該比較適合大型應用，或真的有許多 Legacy code 要處理的公司團隊。

# 那有誰採用 Micro Frontends？

很多公司可能都有自己的類似解決方案，就像 FB 的 bigpie，或是 Yahoo 也有類似的作法。

在我查到的資料當中，比較有提到 Micro Frontends 的大公司有 Spotify 與 IKEA，細節就給大家自行去欣賞他們的分享了：

[Spotify - How Spotify Builds Products (Organization. Architecture, Autonomy, Accountability)](https://www.slideshare.net/kevingoldsmith/how-spotify-builds-products-organization-architecture-autonomy-accountability):

![Spotify sharing](/img/arvinh/spotify-microfe.jpg)

Spotify 是從 UI 來區分團隊負責的 function，原先採用 iframe 與 postMessage 的方式來處理 Micro Frontends，團隊間 technology independence，但他們已經捨棄這個架構很久了，後來改採一律 react/redux 的架構。可以看這份 [twitter](https://twitter.com/derberq/status/910056617881817089) 討論。

另外，它們是有一個團隊在負責檢驗各 team 產出的模組 UI style 是否維持產品的一致性。

[MICROSERVICE WEBSITES - Gustaf N. Kotte](https://www.youtube.com/watch?v=4KVOuQDIfmw)

![MICROSERVICE WEBSITES - Gustaf N. Kotte](/img/arvinh/ikea-microfe.png)

# 小結論

其實會需要用到 Micro Frontends 結構的應該不多，尤其在 React/Vue/Angular 主宰的這幾年，需求相對更少。越大型的 web app 與團隊才比較值得去嘗試。

現行的實作方式其實都很複雜，上面描述的方法與網路看到的 prototype 都過於簡化了。

但這種高階層架構的思考與設計其實多多益善，從中發現的困難更能推動技術發展，而且也很有趣！

## 資料來源

1. [micro-frontends.org](https://micro-frontends.org/)
2. [フロントエンドエンジニアは Micro Frontends の夢を見るか](https://tech.mercari.com/entry/2018/12/06/162827)
3. [技术雷达之「微前端」- 将微服务理念扩展到前端开发（上：理论篇）](https://blog.jimmylv.info/2017-12-24-tech-radar-microfrontends-extending-microservice-to-fed/)
4. [技术雷达之「微前端」- 将微服务理念扩展到前端开发（下：实战篇）](https://blog.jimmylv.info/2017-12-24-tech-radar-microfrontends-extending-microservice-to-fed-next/)
5. [Micro Frontends Proof of Concept](https://github.com/Pragmatists/microfrontends)
6. [Independent micro frontends with Single SPA library](https://blog.pragmatists.com/independent-micro-frontends-with-single-spa-library-a829012dc5be)
7. [Single-SPA](https://single-spa.surge.sh/)
8. [Spotify - How Spotify Builds Products (Organization. Architecture, Autonomy, Accountability)](https://www.slideshare.net/kevingoldsmith/how-spotify-builds-products-organization-architecture-autonomy-accountability)
9. [MICROSERVICE WEBSITES - Gustaf N. Kotte](https://www.youtube.com/watch?v=4KVOuQDIfmw)

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化