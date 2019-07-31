---
title: Chrome devtools extension 實作介紹
date: 2018-02-10 11:04:27
tags:
  - Chrome
  - devtools
  - extension
---
# 前言

身為工程師，想辦法提高自己的工作效率是一件很重要的事情，~~畢竟這樣才有多餘的時間打電動~~，除了平時加強各種知識與累積經驗外，撰寫工具的能力也值得培養。剛好在前不久，公司舉辦了一次內部的 hackday，我就利用這個時間針對公司內部的 framework 寫了一個簡單的 chrome devtools 的 extension，增加開發上的便利性。而在開發的過程中，發現關於 Chrome devtools extension 的文章並不是很多，中文的更少。總之，雖然[官方文件](https://developer.chrome.com/extensions/devtools)該有的都有，但跟 vscode extension 的文件比起來就是差了很多，光是排版就讓人不太想閱讀...
因此希望藉此篇文章介紹開發 chrome devtools extension 的方法與一些注意事項。（註：可能需要先有點 Chrome extension 的相關知識會比較好懂）

先給大家看其中一個範例（共有兩個），主要是能自動將 DOM 物件的 element inline style 轉化為 [Atomic CSS](https://acss.io/) 的 class 名稱（[不知道 Atomic CSS 的可以看這篇](https://blog.techbridge.cc/2017/04/29/css-methodology-atomiccss/)），如此一來，在 Inspector 中調整完 style 後，就能直接將轉換好的 Atomic CSS 複製貼上到 code 當中，省去一次自己轉換的時間（有時還會忘記 class name...）

![Sidebar Demo](/img/arvinh/devtools-acss-demo.gif)

# Chrome devtools extension 基本介紹

有開發過 Chrome extension 的人應該都知道，我們會有所謂的 `Content Script` 與 `Background page` 兩種不同的 context 存在於我們的 extension 中，而 `Devtools page` 也是一個獨立的 context，從下面這張官方圖可以很清楚的看到其之間的差異：

![官方圖片](/img/arvinh/devtools-extension-overview.png)

* Content Script: 可以存取實際頁面的 DOM 物件與事件。
* Background page: 可以調用多數 extension API，像是 `chrome.runtime.*` 與 `chrome.tabs.onUpdated`，並負責 extension 與 Content script、Devtools page 之間的溝通。
* Devtools page: 可以調用 `chrome.extensions.*` 與 `chrome.devtools.*` Devtools API，其他的就都無法存取。可以透過 `chrome.devtools.inspectedWindow.eval` 能與目前開啟 inspector 的頁面互動。

與一般 Extension 不同的地方就在於多了 Devtools API 需要了解，而主要的 Devtools API 其實也只有三種：

### 1. chrome.devtools.inspectedWindow

透過 `inspectedWindow.eval` 可以在當前開啟 inspector 的頁面 context 執行 javascript：
```js
chrome.devtools.inspectedWindow.eval(
  "window.$0.style.cssText",
  function callback(result, Error) {
    // result 為 window.$0.style.cssText
    // 在當前頁面的 context 下支執行結果 
});
```
### 2. chrome.devtools.network

network api 可以取得你在 `Network panel` 看到的資訊。
![network panel](/img/arvinh/devtools-networkpanel.png)
```js
// 取得當前開啟 inspector 的頁面所發出的 request 中，bodySize > 40*1024 的 url
chrome.devtools.network.onRequestFinished.addListener(
      function(request) {
        if (request.response.bodySize > 40*1024) {
          chrome.devtools.inspectedWindow.eval(
              'console.log("Large request: " + unescape("' +
              escape(request.request.url) + '"))');
        }
  });
```
### 3. chrome.devtools.panels
panels api 應該是最重要的一塊了，因為我們必須透過它來創建 Panel 或 Sidebar。

![panel-sidebar](/img/arvinh/devtools-panel&sidebar.png)

Chrome devtools 的 extension UI 基本上就是分為上面這兩種類型，與上方 `Elements`、`Network` 和 `Sources` 同 Level 的稱為 Panel，而在每個 Panel 底下還可以另外創建 Sidebar，像是 Elements panel 右邊的 `style sidebar`。

```js
chrome.devtools.panels.create("Simple Panel",
  "logo.png",
  "Panel.html",
  function (panel) {
    // code invoked on panel creation
  }
);
```

看完這些 API 應該也是一頭霧水，也不清楚到底要在哪裡呼叫，別急，接下來會針對這兩種 UI 個別實作一個範例來說明。

# Chrome devtools extension - Sidebar 實作

開頭的範例中，就是採取 Sidebar 的 UI，屬於 `ElementPanel` 底下的 sidebar。

接著先看一下我們的檔案結構：

<img src="/img/arvinh/devtools-foldr-structure.png" style="width:50%;height:50%">

很簡單，重點只有三個檔案，ruleMap.js 是跟 Atomic CSS 相關的 mapping 檔案，這邊不需要理會：

### 1. manifest.json
  ```json
    {
      "name": "Atomic CSS Devtool",
      "version": "0.1",
      "description": "devtool extension for making Acss users happier",
      "devtools_page": "devtools.html",
      "manifest_version": 2
    }
  ```
  跟一般 extension 一樣，要在 manifest.json 中做相對應設定，既然是開發 Devtools extension，自然就要註冊 `devtools_page`，指定為 devtools.html，這份 html 就是用來載入相關 js 的入口頁面。
### 2. devtools.html
  ```html
    <html>
      <body>
        <script src="ruleMap.js"></script>
        <script src="devtools.js"></script>
      </body>
    </html>
  ```
  內容非常簡單，載入整個 devtools extension 需要的 javascript 檔案。如果你在 devtools.js 中有其他需要使用的 lib，也請記得在這個地方進行載入，像是 lodash 等等。
  但若你要載入非本地端的 javascript（透過 cdn 之類），會遇到 CSP(content security policy) 的錯誤，在 <a href="#notice">注意事項</a> 中我會再說明解法。
### 3. devtools.js
  ```js
    (function() {
      chrome.devtools.panels.elements.createSidebarPane(
        "acss class",
        function (sidebar) {
          // The function below is executed in the context of the inspected page.
          var page_generateAtomicClass = function (selectedElementCssText) {
            // generate Atomic CSS
            // 略...
            return styleMap;
          }
          function updateElementProperties(acssClass) {
            sidebar.setObject(acssClass);
          };

          function getAcssClass() {
            chrome.devtools.inspectedWindow.eval(
              "window.$0.style.cssText",
                function (result, isException) {
                  const selectedElementCssText = result;
                  const acssClass = page_generateAtomicClass(selectedElementCssText);
                  updateElementProperties(acssClass);
                }
            );
          };
          chrome.devtools.panels.
            elements.onSelectionChanged.addListener(getAcssClass);
          getAcssClass();
      });
    })();
  ```

devtool.js 就比較複雜了，所有主要功能都發生在這裡。

首先，我們利用 `chrome.devtools.panels.elements.createSidebarPane(sidebarTitle, callbackFunc)` 來創建 Sidebar，在 callback 中我們會拿到一個 `sidebar` 物件，此物件是我們與右邊這個 sidebar 區塊互動的媒介，有四種 method 可以使用：

* sidebar.setObject()
    我們範例中就是使用 `setObject()` 來將運算完的資料（轉換後的 atomic css classname）傳到 sidebar 顯示，他會將傳入的 Object 展開：

    ![setObject](/img/arvinh/devtools-setobj.png)

* sidebar.setPage() 與 sidebar.setHeight()
    若是覺得光是顯示 JS 的 Object 太單調，你也可以利用 `setPage()` 搭配 `setHeight()` 來在 sidebar 中塞入一個 html。
    ```js
      chrome.devtools.panels.elements.createSidebarPane("Atomic Css",
          function(sidebar) {
            sidebar.setPage("Sidebar.html");
            sidebar.setHeight("8ex");
          });
    ```
    Sidebar.html 中，可以自由繪製畫面，但要注意的是，你是在 sidebar 的 context 中，不能取得當前頁面的 DOM 物件資料，需要的話得透過 background.js 以 `postMessage` 來傳遞，我們最後還會提到。
* sidebar.setExpression()
    除了 `setObject` 以外，我們也能夠過 `setExpression` 直接 `eval()` js code 到當前的 inspected page。
    
    ```js
      sidebar.setExpression("(" + page_generateAtomicClass.toString() + ")()");
    ```
    使用方法有點特殊，因為是 eval 的方式，你需要把函式 toString() 後再傳入，此外，在傳入的 `page_generateAtomicClass()` 中，你可以取得當前 inspected 頁面的資訊！
    例如：$0 (Chrome devtools 中特殊的變數，等同於 Element panel 中你目前選取的元素 DOM)。
    官方文件中 setExpression 還能傳入一個 callback，但是我怎麼傳都會有 Error 就是了...

接著，我用 `chrome.devtools.inspectedWindow.eval("window.$0.style.cssText", callback);` 的方式去取得 selected element 的 css 資訊，接著在 callback 中將其傳給 `page_generateAtomicClass()` 做運算，最後用 `sidebar.setObject()` 將結果輸出。

等等，你不是說 `sidebar.setExpression` 就能直接取得 `$0` 了嗎？何必多此一舉？

原因很簡單，因為在 `setExpression` 傳入的 function 中，你取不到 devtools.js 中的 context，所有你在 devtools.html 中 include 的 js 都無法取得，像是我需要用來轉換 Atomic css 的 `ruleMap.js` 就無法拿到，只好採此作法。在實作時需要特別注意 context 的問題！

最後，我們註冊一個 `onSelectionChanged` 監聽事件 `chrome.devtools.panels.elements.onSelectionChanged.addListener(getAcssClass);`，只要選擇別的 elements 時就重新執行。

到這邊為止，你就能夠做出如同一開始範例所展現功能的 Devtools extension 了！

# Chrome devtools extension - Panel 實作

這個範例是實作 Panel UI 的 extension，這邊我將功能降之最低，單純抓出目前頁面的 Page title，目的在展示如何將不同 context 的資訊呈現在 Devtools 的 Panel 中。

![Panel Demo](/img/arvinh/devtools-panel-demo.gif)

一樣先來看檔案結構：

<img src="/img/arvinh/devtools-panel-structure.png" style="width:50%;height:50%">

可以看到基本上跟前一個範例差不多，只是多了 `Panel.html` 與 `background.js` 兩個檔案。

另外的差別在於 `devtools.js`：

```js
(function() {
  chrome.devtools.inspectedWindow.eval(
    "document.title",
    function (result, isException) {
      if (!isException && result) {
        chrome.devtools.panels.create("Panel Demo",
          "logo.png",
          "Panel.html",
          function (panel) {
            // code invoked on panel creation
          }
        );
      }
    }
  );
})();
```

有別於前一個範例我們都將程式邏輯寫在 `devtools.js` 中，這次我們只在這邊進行創建 panel 的程式，可以從上面程式碼中看到，我們創建了一個 Title 叫做 "Panel Demo" 的 panel，並告訴 chrome devtool 是要用 `Panel.html` 這份檔案。

在 `Panel.html` 中，我們載入主要程式邏輯 `getPageTitle.js`，你也可以看到，這邊就是繪製 Panel 的地方，因此可以載入 bootstrap 等 css style 來輔助。

```html
<link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0/css/bootstrap.min.css" integrity="sha384-Gn5384xqQ1aoWXA+058RXPxPg6fy4IWvTNh0E263XmFcJlSAwiGgFAW/dAiS6JXm"
    crossorigin="anonymous">
<div id="app" class="container mt-3">
</div>
<script src="getPageTitle.js"></script>
```

接著來看主要程式邏輯，`getPageTitle.js`：

```js
(function() {
  // 與 background.js 建立 channel 連結
  const port = chrome.extension.connect({
      name: "Devtools.js Communication"
  });
  const inspectedWindowId = chrome.devtools.inspectedWindow.tabId;
  // Listen to messages from the background page
  port.onMessage.addListener(function (message) {
      if (message.action === "reloadExtension" && message.updatedTabId === inspectedWindowId) {
          const appNode = document.querySelector('#app');
          removeHTMLChilds(appNode);
          getPageTitle();
      }
  });
  function getPageTitle() {
      chrome.devtools.inspectedWindow.eval(
          "document.title",
          function (result, isException) {
              const appNode = document.querySelector('#app');
              const titleWrapper = document.createElement("div");
              const title = document.createTextNode(result); 
              titleWrapper.appendChild(title);
              appNode.appendChild(title);
          }
      );
  }
  function removeHTMLChilds(HTMLNode) {
      while (HTMLNode.firstChild) {
          HTMLNode.removeChild(HTMLNode.firstChild);
      }
  }
  // init
  getPageTitle();
})();
```

我們利用 `chrome.devtools.inspectedWindow.eval()` 來執行 `document.title`，取得 page title 資訊，並利用 `document.createElement` 等原生 Web API 來將資訊呈現在頁面上。

接著這邊我們用到了 `port.onMessage.addListener()`，原因是我們想要 monitor 頁面的變化，像是 page reload 或是 page update。而這些資訊都只能透過 `content script` 或是 `Background.js` 才能取得，因此我們必須建立一個 messaging 的 channel，讓 `background.js` 告訴我們頁面是否更新了，若更新就重新繪製 `Panel.html` 的內容。

```js background.js
chrome.runtime.onConnect.addListener(function (port) {
    chrome.tabs.onUpdated.addListener(function (tabId, changeInfo, tab) {
        if (changeInfo.status === 'complete') {
            reloadExtension(port, tabId);
        }
    });
    function reloadExtension(port, tabId) {
        const message = { action: "reloadExtension", updatedTabId: tabId };
        port.postMessage(message);
    }
});
// `background.js` 透過 `chrome.runtime.onConnect.addListener`
// 在與 devtools page 的 script 連接到後，監聽 `chrome.tabs.onUpdated` 事件，
// 當 update status 為 complete 後，`postMessage()` 給 `Panel.html` 中的 `getPageTitle.js`。
```

此外，由於 `Background.js` 存在於整個 Browser 中，因此在 `getPageTitle.js` 中，需要透過 `const inspectedWindowId = chrome.devtools.inspectedWindow.tabId;` 取得當前 inspected page 的 tab id 來過濾其他 tab 的 event change。

就這樣我們就完成了一個可以取得頁面 Title 的 Devtools extension！雖然功能超廢但要是希望讓大家有個概念，知道要怎麼開始。
基本上所有程式碼都在這邊了，但如果還是想直接載範例 code 來看的話可以移駕至 github，但只是 demo 用就是了...[Demo 1](https://github.com/ArvinH/acss_devtool) [Demo 2](https://github.com/ArvinH/chrome-devtools-extension-panelDemo)
<span id="notice"></span>

# 注意事項 

在 hackday 開發內部工具時其實踩到不少雷，而在上述的範例中比較難去說明，因此在文章的最後額外與大家分享：

1. 載入外部檔案的方式
    雖然在 `devtools.html` 或 `Panel.html` 中可以載入除了 `devtools.js` 外的檔案，但若是載入的檔案中存有 `eval()` 或著是透過 web 下載的 js，都會出現 CSP(content ecurity policy) 錯誤，而解法是在 `manifest.json` 中加上一行：
    "content_security_policy": "script-src 'self' 'unsafe-eval'; object-src 'self'"
    這樣就能解決 js 中存有 `eval()` 或是 `setTimeout()` 所造成的 CSP error 

    而若要載外部資源，則還要另外將其 domain 也加進去，當作 white list
    "content_security_policy": "script-src 'self' 'unsafe-eval' https://maps.googleapis.com/; object-src 'self'"
    
2. chrome.devtools.inspectedWindow.eval 長度限制
    在製作內部工具時，其實是需要從 inspected page 的 context 中取出大量資料到 Panel.html 中進行處理，而透過 `inspectedWindow.eval` 的方式並沒有辦法傳送太大量的 JSON object，因此我是先將其 `JSON.stringify()` 後才往後傳的。 eg. `chrome.devtools.inspectedWindow.eval( "JSON.stringify(context.getStore())"`。

# 結論

今天透過兩種範例介紹了如何用最主要的 Devtools API 來搭建 Devtools extension，但當然還有許多 API 或是 Event 沒有介紹到，像 panels 就有 `onShown`, `onHide` 等監聽 devtools 是否有開啟的事件可以用，不過很難從一次的範例中全部介紹到，有需要的話還是得去查看官方文件。至少希望能讓大家對於製作增加自己工作效率的工具有一個初步的開始方向，有任何問題也歡迎提出指教！

<!-- 資料來源 -->
## 資料來源
1. [Google offical doc](https://developer.chrome.com/extensions/devtools)
2. [Chrome Extension CSP 開發小記](https://div.io/topic/1669)
3. [Chrome 插件(擴展)全攻略](http://www.360doc.com/content/17/1212/14/26662048_712402137.shtml)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化