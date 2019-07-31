---
title: 當 React web app 遇上 Progressive web app
date: 2016-09-17 12:37:22
tags:
  - react
  - pwa
  - Progressive web app
---

先前介紹過 Progressive Web App，我們利用 vanilla js 與 css 刻出一個支援 PWA 的天氣 web app，
但現在應該很少人這麼有風格的用純 JS 寫 web app，世俗如我還是會想用 React 來支援前端框架，但 React 的 configuration 就已經頗複雜，要再加上 service-worker 的設定，想想就覺得累。

好在 facebook 先前推出 [create-react-app](https://github.com/facebookincubator/create-react-app) 這個工具，幫你預先設定好一些 config，並且隱藏起來（還是想要自己設定的話，可以透過 `npm run eject` 這個指令），如果是中小型專案應該很適合。

因此，問題就剩下加上 service worker 這段了。

幸運的是，Google chrome 的開發成員有一個 open source 的 repository - [create-react-pwa](https://github.com/jeffposnick/create-react-pwa)，是基於 create-react-app 的架構去支援 PWA。

不過實際上我使用起來還是有些地方需要微調與注意的，所以今天就是來帶著大家一步一步把你從 `create-react-app` 中所創建的 React Web App 加入 PWA 的支援。

## Add PWA support in your create-react-app
<!-- 講解如何加入 pwa support -->

以防大家沒用過 `create-react-app`，這邊稍稍介紹一下。首先你要先透過 `create-react-app` 來產生你的 web app:

`npm install -g create-react-app`

`create-react-app react-pwa-boilerplate`

接著就會產生以下結構的資料夾：

<img src="/img/arvinh/create-react-app-structure.png" alt="create-react-app-structure" style="width: 200px;">

當然 `src` 內的結構你可以依照你自己需求調整。

接著 `npm start` 即可在 `localhost:3000` 看到頁面了。

而在有了一個 react web app 的雛形後，我們就可以來加上 PWA 的支援了！

### Step I - sw-precache

要能夠擁有 `Progressive Web App` 的能力，需要 `Service Worker` 的幫助，除了自己撰寫 service-worker 以外，我們可以利用 Google 出的 [`sw-precache` ](https://github.com/GoogleChrome/sw-precache)來幫我們產生需要的 `service-worker.js`。

在 `package.json` 中的 `devDependencies` 加入 `sw-precache`：
```js package.json
  "devDependencies": {
    "react-scripts": "0.4.1",
    "sw-precache": "^4.0.0"
  },
```
or `npm install sw-precache --save`

接著，在 `package.json` 中的 `script` 內，我們要修改一下 `build` 的指令：

```js package.json
"scripts": {
  "build": "react-scripts build && cp manifest.json favicon.ico build/ && sw-precache --navigate-fallback='index.html' --root='build/' --static-file-globs='build/**/!(*map*)'",
}
```

這個指令做了幾件事情：

1. `react-scripts build` 是原本 `create-react-app` 的指令，將相關的 react component 等等透過 webpack 幫你編譯轉譯後放到 `build` folder 底下。

2. `cp manifest.json favicon.ico build/` ： manifest.json 與 favicon.ico 都是 PWA 需要的東西，待會會在說明內容。因為原本的 react-scripts build 中並不會產生這樣的東西，自然也不會幫你 build 進去，但我們需要讓 `service worker` 能儲存它們，所以這邊就手動把他放入 `build` 底下。

3. `sw-precache --navigate-fallback='index.html' --root='build/' --static-file-globs='build/**/!(*map*)'` ：

這個指令透過 sw-precache，產生一個 service-worker.js 的檔案，他會自動幫你把 build folder 底下的 static 檔案都暫存起來！

`--navigate-fallback='index.html'` 這個 flag 是為了讓你使用 React Router 而放入的。如果你是根據 facebooke 的 [doc](https://github.com/facebookincubator/create-react-app/blob/master/template/README.md) 加入 react-router，並且 deploy 到 github page，那你必須加入這個 flag，這樣一來，當 user 在切換動作而改變 url 時，任何的 random url 都會 fallback 到 index.html，其實也就是你 single page 的那個 entry 點，`service-worker` 才能夠找得到 cache。

`--static-file-globs`：這個 flag 讓 sw-precache 存 source map 外的所有 static files。

### Step II - manifest.json

再來是 PWA 中很重要的 manifest.json，能不能順利在 mobile 上 Add to homescreen 就要靠這份檔案：

```js manifest.json
{
    "short_name": "react-pwa-boilerplate",
    "name": "react-pwa-boilerplate",
    "icons": [
        {
            "src": "favicon.ico",
            "sizes": "144x144",
            "type": "image/png"
        }
    ],
    "start_url": "./",
    "display": "standalone"
}
```

要注意的是，這邊的 icons，size 一定要給正確的值，否則會造成 manifest 的解析錯誤，另外，如果要能讓 iOS 內的 safari 也能夠跑，需要有額外的設定，待會還會提及，也可以先參考之前的 [文章](blog.arvinh.info/2016/07/18/progressive-web-app/)

### Step III - index.html

上述設定都做完後，要將 `manifest.json` 放入 `index.html` 內，並註冊 `service-worker.js`。

```html index.html
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="shortcut icon" href="./src/favicon.ico">
    <!-- manifest 加入這邊～ -->
    <link rel="manifest" href="manifest.json">
    <title>React App</title>
  </head>
  <body>
    <div id="root"></div>
    <!-- service worker 在這邊註冊~ -->
    <script>
        if ('serviceWorker' in navigator) {
            navigator.serviceWorker.register('service-worker.js').catch(function(ex) {
                console.warn(ex);
                console.warn('(This warning can be safely ignored outside of the production build.)');
            });
        }
    </script>
  </body>
</html>

```

### Step IV - Deploy to github page

搭配 github page 可以快速將你的 web app 部署上去，`create-react-app` 在你執行完 build 以後，提供了 deploy to github 的指令，我們可以把它先加入 package.json 當中。另外也一並加入 `homepage` 的欄位。

```js package.json
"scripts": {
    "deploy": "git commit -am \"Save local changes\"; git checkout -B gh-pages; git add -f build; git commit -am \"Rebuild website\"; git filter-branch -f --prune-empty --subdirectory-filter build; git push -f origin gh-pages; git checkout -",
    "start": "react-scripts start",
    "build": "react-scripts build && cp manifest.json favicon.ico build/ && sw-precache --navigate-fallback='index.html' --root='build/' --static-file-globs='build/**/!(*map*)'",
    "test": "react-scripts test --env=jsdom",
    "eject": "react-scripts eject"
  },
  "homepage": "https://blog.arvinh.info/react-pwa-boilerplate"
```

### Step V - build & deploy & run and ... debug?!

如果你一步一步跟著做到這裡，理論上就可以 `npm run build` 來 build 看看，接著再 `npm run deploy` 來部署到 github page 上。

不過當你執行完，console 也沒有噴任何 error，將你的 web app 用手機開啟並加入主畫面後，重新開啟還是一樣進入瀏覽器的畫面？！

#### manifest path error

仔細檢查一下瀏覽器 error 會發現在我們 build 出來的 index.html 內，manifest.json 的路徑被 parse 錯誤了！

![manifest error](/img/arvinh/manifestError.png)

這其實是因為目前 `create-react-app` 內部的 webpack loader 的 issue，在這個 [issue](https://github.com/jeffposnick/create-react-pwa/issues/3) 裡面有提到，目前我的解法是：

1. 將一開始的 index.html 修改為 `<link rel="manifest" href="/manifest.json">`
2. build 完以後，到 build/index.html 內將其修正為 `<link rel="manifest" href="./manifest.json">`

除了上面這個錯誤以外，以上步驟還少了一些東西。

#### iOS support

如果你想要你的 web app 能夠在 iOS 上的 safari 運作，在 index.html 中還得要加入以下幾行：
```html index.html
<!-- Add to home screen for Safari on iOS -->
    <meta name="apple-mobile-web-app-capable" content="yes">
    <meta name="apple-mobile-web-app-status-bar-style" content="black">
    <meta name="apple-mobile-web-app-title" content="React-PWA">
    <link rel="apple-touch-icon" href="./src/images/favicon-144x144.png">
```
其中的 icon 路徑記得要填寫正確，也一樣要確認好 image 的 size。


#### Cache polyfill

現在 service worker 的支援度還很低，有些 polyfill 可能需要加一下，但我還不確定到底需要哪些...

這個 Cache polyfill 用來增加 Cache api 的支援：

https://github.com/dominiccooney/cache-polyfill

修改你 pacakage.json 中的 build script，加上：

```js pacakage.json
"build": "react-scripts build && cp manifest.json cache-polyfill.js favicon.ico build/ && sw-precache --navigate-fallback='index.html' --root='build/' --static-file-globs='build/**/!(*map*)'",
```

並在你 build folder 內的 service-worker.js 內加上：

`importScripts('cache-polyfill.js');`


#### Run

修正完上述錯誤後，執行 `npm run deploy`，應該就會自動幫你把 build folder 底下的內容部署到 gh-pages 這個 branch 上，你就能在 https://[your_github_acount].github.io/[project_name]/ 看到你的 web app。

試著加入主畫面看看，如果沒問題的話，當你再次從主畫面點選 icon 開啟時，應該就可以看到你的網站像 app 一般的呈現在手機上了！

你可以先從 chrome devtool 確認 service worker 的運作情況，打開在 devtool 中的 **Application tab** (以前叫做 resources tab)

online:
![pwa-network](/img/arvinh/pwa-network.png)

offline:
![pwa-nonetwork](/img/arvinh/pwa-nonetwork.png)

可以看到 resource 在 offline 時是從 service worker 來。

手機上的狀況像是:

<img src="/img/arvinh/pwa-sample.gif" alt="pwa-sample" style="width: 200px;">

<img src="/img/arvinh/pwa-sample2.gif" alt="pwa-sample2" style="width: 200px;">

### 補充說明

這篇文章講述到的 PWA support 實際上只能讓你的 web app 能跳脫瀏覽器，運作起來像個 Native App，但是並沒有考量其他實作 PWA 時需要注意的細節，像是 PWA 內的 App shell 等等。

另外，你的 web app 一定不可能只有 static files，勢必會需要有跟 API 溝通的部分，這邊就需要額外使用 [sw-toolbox](https://github.com/GoogleChrome/sw-toolbox) 來負責 runtime caching strategies，你也可以透過 sw-precache 的設定檔來處理：
```js precache-config.json
{
dynamicUrlToDependencies: {
  dynamic/page1: [
    "app/views/layout.jade",
    "app/views/page1.jade"
  ],
  dynamic/page2: [
    "app/views/layout.jade",
    "app/views/page2.jade"
  ]
  },
  staticFileGlobs: [
    "app/css/**.css",
    "app/**.html",
    "app/images/**.*",
    "app/js/**.js"
  ],
  stripPrefix: "app/",
  verbose: true,
  runtimeCaching: [
    {
      urlPattern: "/this\.is\.a\.regex/",
      handler: "networkFirst"
    }
  ]
}
```
只要在 sw-precache 的指令後加上 `--config=sw-precache-config.json` 這個 flag 來指定 config 檔即可。

### 程式碼

上述完整程式碼可以在這裡取得：
https://github.com/ArvinH/react-pwa-boilerplate

測試頁面：https://blog.arvinh.info/react-pwa-boilerplate/

目前測試似乎 offline 會有問題，會持續修正更新！並加入 react-router、redux 等常用 lib。

### 工商服務時間

前陣子強者我朋友寫了一個神奇寶貝屬性對戰的遊戲，可以幫助你瞭解神奇寶貝之間各種屬性的相剋狀況，剛好是使用 `create-react-app` 與 `Redux` 實作，我也加入了 pwa 的版本，只是目前 offline 似乎有點問題...

歡迎大家幫忙修~

大家可以先到原本網站玩玩!

原版：
https://kaddopur.github.io/type_instructor/

PWA：
https://blog.arvinh.info/type_instructor/#/?_k=usu9f3

![type_instructor](/img/arvinh/pwa-react.gif)

## 參考資料
* [Create-react-pwa](https://github.com/jeffposnick/create-react-pwa)
* [Google sw-precache](https://github.com/GoogleChrome/sw-precache)
* [Cache-polyfill](https://github.com/dominiccooney/cache-polyfill)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化