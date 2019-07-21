---
title: 淺嚐 Web AR
date: 2019-07-16 17:40:44
tags:
  - web
  - ar
  - javascript
---

## 前言

近年 VR/AR 一直不斷出現在大家的視線內，雖然一直沒有什麼殺手級的應用出現，但這阻止不了開發者們的雄心壯志，尤其是 JavaScript 社群，畢竟 Jeff Atwood 說過：

> "Any application that can be written in JavaScript, will eventually be written in JavaScript." — Jeff Atwood, Author, Entrepreneur, Cofounder of StackOverflow

隨著 a-frame 的出現，WebVR 成為現實，並且很容易開發；Web AR 部分進展則相對緩慢ㄧ些，瀏覽器原生支援的 API 還一直處於不穩定的開發階段，但即便如此，我們還是可以在特定版本的瀏覽器上使用，此外，也有像是 AR.js 這樣融合 artoolkit、three.js、ＷebGL 和 WebRTC 等技術的工具可以使用。

今天就來稍稍研究一下，看看目前的技術能如何開發 Web AR！

照慣例，開始前先看點 demo，把 Pokemon 帶到你家客應：

![arjs demo](/img/arvinh/arjs-demo.jpg)

<!--
1. 簡介
2. 介紹目前有的 WebAR 實作 lib 或 api
2-1. 說明 WebXR Viewer on iOS 與 Google chrome 可以跑的 api
3. 分別以 AR.js 與 Web 純 API 介紹
-->

## 目前的工具有哪些

[CreateＷebVR](https://createwebvr.com/webAR.html) 這網站上列出了一些目前有的 library，以及目前支援 WebAR 的瀏覽器：

![常見 lib](/img/arvinh/webar-libs.png)

其中 [AR.js](https://github.com/jeromeetienne/AR.js) 使用了 A-Frame (基於 Three.js) 以及 JSARTookit5（JavaScript 移植版的 [ARToolKit](https://github.com/artoolkit/artoolkit5)），而這兩個技術主要皆是利用 WebGL 為主，因此大多現行的瀏覽器都能直接支援，不需要特殊的 API。

這大概也是為何 AR.js 能在 GitHub 上擁有一萬多顆星星，遠勝過上列其他套件的原因。再加上作者的實驗證明 AR.js 即便在[兩年的老舊手機上也能運行順暢，擁有良好的 Performance](https://github.com/jeromeetienne/AR.js#performance)。

但 AR.js 也並非沒有缺點，由於是基於 ARToolKit，因此只能夠支援 Marker-based 的 AR 效果，也就是像最前面的 Demo 圖片一樣，是需要在鏡頭內放置一個設定好的 Marker，讓其辨識，取得環境的一些 Sensor 資訊，包含鏡頭的深淺遠近等等，才能讓 AR 物件渲染在視窗中。

即便如此，AR.js 其簡潔、便利的使用方式（[有 a-frame 與 threejs 的 extension](https://github.com/jeromeetienne/AR.js#folders)），能讓你用[短短 10 行程式碼就產生出一個 WebAR 效果的網頁](https://medium.com/arjs/augmented-reality-in-10-lines-of-html-4e193ea9fdbf)，還是非常好用的。

![支援的 browsers](/img/arvinh/webar-browsers.png)

如果你是使用 AR.js 的話，基本上現行有支援 WebGL 的手機瀏覽器就都能夠運行，不需要額外的 Polyfill 等等。

但如果你想要使用非 Marker-based 效果的 AR 應用，你就得研究 [`WebXR Device API`](https://immersive-web.github.io/webxr/)，其前身為 [WebAR API](https://developer.mozilla.org/en-US/docs/Web/API/WebVR_API)。

## WebXR Device API

WebXR Device API 現在還在不斷更新中，不是個穩定的 API。

運用到 WebXR Device API 的有 Google 使用的 [three.ar.js](https://github.com/google-ar/three.ar.js) 與整合 A-Frame 的 [aframe-ar.js](https://github.com/chenzlabs/aframe-ar)，以及 Mozilla 主導的 [three.xr.js](https://github.com/mozilla/aframe-xr) 和 [aframe-xr.js](https://github.com/mozilla/three.xr.js/)。

Google 提供了 [WebARonARCore](https://github.com/google-ar/WebARonARCore) 和 [WebARonARKit](https://github.com/google-ar/WebARonARKit) 兩種分別在 Android 與 iOS 平台上運行的特殊 browsers，讓你能在上面跑 WebXR Device 相關的 API：而 Mozilla 在 iOS 上也推出 [Mozilla WebXR Viewer](https://labs.mozilla.org/projects/webxr-viewer/) 來對應，三者皆是用到手機平台原生的 ARKit。

如同剛剛所說，WebXR Device API 還很不穩定，而且 Google 與 Mozilla 各自在 Android 與 iOS 上有不同的實作，[Google code lab 上的範例](https://codelabs.developers.google.com/codelabs/ar-with-webxr/#0)只能運行在 version 為 70 - 72 的 Chrome canary 上，而 Mozilla 雖然有推出 [Mozilla WebXR Viewer](https://labs.mozilla.org/projects/webxr-viewer/)，但上面的[範例實作](https://github.com/MozillaReality/webxr-ios-js/tree/master/examples)也**只能**運行在 [Mozilla WebXR Viewer](https://labs.mozilla.org/projects/webxr-viewer/) 上頭...

不過去看一下雙方的範例程式碼後，會發現其實用法蠻雷同的，大多都有下面這些流程（以 Google 的程式碼來當範例）：

判斷是否支援 WebXR Device API，並初始設定：

```js
/**
   * Fetches the XRDevice, if available.
   */
  async init() {
    // `navigator.xr` 是 WebXR Device API 的入口，有必要確認其存在
    // 而 `XRSession` 中的 `requestHitTest` 則是要 enable #webxr-hit-test flag
    // 確認這兩個 API 存在，確保能夠製造出點擊畫面
    if (navigator.xr && XRSession.prototype.requestHitTest) {
      try {
        this.device = await navigator.xr.requestDevice();
      } catch (e) {
        // Error handling，通知使用者的瀏覽器並不支援
        this.onNoXRDevice();
        return;
      }
    } else {
      // Error handling，通知使用者的瀏覽器並不支援
      this.onNoXRDevice();
      return;
    }
    // 成功取得 XRDevice 物件後，需要 bind 一個 user gesture 的 event，然後呼叫
    // `device.requestSession()`，這是規範在 spec  中的
    document.querySelector('#enter-ar').addEventListener('click', this.onEnterAR);
  }
```

取得 `XRDevice` 後，利用 `device.requestSession()` 製造出運行 XR 的環境：

```js
  async onEnterAR() {
    const outputCanvas = document.createElement('canvas');
    const ctx = outputCanvas.getContext('xrpresent');

    try {
      // `device.requestSession()` 一定要是由 user 觸發，像是 click handler 內
      const session = await this.device.requestSession({
        outputContext: ctx,
        environmentIntegration: true,
      });
      document.body.appendChild(outputCanvas);
      // 成功創建 Session 後就能開始運算 AR 了
      this.onSessionStarted(session)
    } catch (e) {
      // Error handling，通知使用者的瀏覽器並不支援
      this.onNoXRDevice();
    }
  }
```

當 XRSession 成功創建後，接著就是 set up three.js，撰寫 renderer，設定 scene、camera，並 attach 上 XRWebGLLayer，然後啟動 render loop：

```js
async onSessionStarted(session) {
  this.session = session;
  // ...省略
  // 利用 Three.js 繪製 3D 物件，因此要借用 THREE.WebGLRenderer 來當作 XRSession 的 render layer
  this.renderer = new THREE.WebGLRenderer({
    alpha: true,
    preserveDrawingBuffer: true,
  });
  // ...省略
  //  設定 render layer
  this.session.baseLayer = new XRWebGLLayer(this.session, this.gl);
  const framebuffer = this.session.baseLayer.framebuffer;
  this.renderer.setFramebuffer(framebuffer);
  // ...省略, 設定 scene
  this.scene = DemoUtils.createLitScene();
  // ...省略, 設定 camera
  this.camera = new THREE.PerspectiveCamera();
  // ...省略, 更多的設定
  // 在 `requestAnimationFrame` 中啟動 render loop
  this.session.requestAnimationFrame(this.onXRFrame);
  // ...省略
}
```

省略了很多細節，但大致的步驟就是這樣，[詳細程式碼在此下載](https://github.com/googlecodelabs/ar-with-webxr/archive/master.zip)

對 WebXR Device API 比較有興趣的讀者除了 [Google code lab 上的範例程式](https://codelabs.developers.google.com/codelabs/ar-with-webxr/#0)與 Mozilla 的 [webxr-ios-js 範例](https://github.com/MozillaReality/webxr-ios-js/tree/master/examples) 外，也可以到 [immersive-web/webxr](https://github.com/immersive-web/webxr/blob/master/explainer.md) 看看該 WebXR Device API 的 Specs detail 與解釋，他們也有提供 [Sample Page](https://immersive-web.github.io/webxr-samples/) 可作參考。

## AR.js

由於手邊沒有適合的 Android 手機，不能嘗試利用 WebXR Device API，所以今天就先來看看 AR.js 有多簡單。

你需要的就只有：

1. 準備好 Marker
2. 準備好 3D models
3. 撰寫十行簡單的程式碼
4. host 你的 WebAR webapp

AR.js 有提供一個 Marker generator - [AR.js Marker Training](https://jeromeetienne.github.io/AR.js/three.js/examples/marker-training/examples/generator.html)

![ARjs marker training](/img/arvinh/arjs-maker-traning.png)

你可以上傳想要的圖片放到 Marker 中，例如一個記載你的 WebVR webapp 網址的 QR code 就很適合。像[這篇文章](https://medium.com/%E9%AB%92%E6%A1%B6%E5%AD%90/%E7%94%A8ar-js%E5%81%9A%E4%B8%80%E5%80%8B%E8%AE%93%E5%8F%A6%E5%B0%8D%E6%96%B9-%E5%96%94%E5%96%94%E5%96%94%E5%96%94-%E7%9A%84%E5%B0%8F%E5%8D%A1%E7%89%87%E5%90%A7-4071ceea41dd)作者利用 AR.js 結合實體卡片送給他女友一個小驚喜。

我建議把你做好的 Marker 下載下來，否則當你手機對著電腦中的 marker 時，3D Model 方位視角會跟你是垂直的，不是很好看。

接著可以到 [Poly](https://poly.google.com/) 或是 [sketchfab](https://sketchfab.com) 下載 `gltf` 的 3D 模型。(搜尋想要的 Model 時記得勾選 `downloadable`，比較不會選到需要付費才能下載的，當然你要付費也很棒！)

以我最前面的例子來說，我在 [sketchfab 中載了一個傑尼龜的模型](https://sketchfab.com/3d-models/squirtle-18caed58804943d7a839dcbd44d21b80)

![sketchfab](/img/arvinh/sketchfb-usage.png)

接著用 AR.js 對應 AFrame 的 extension 撰寫簡單的 WebVR app:

```html
<script src="https://aframe.io/releases/0.9.2/aframe.min.js"></script>
<script src="https://cdn.rawgit.com/jeromeetienne/AR.js/1.7.5/aframe/build/aframe-ar.js"></script>

<body style='margin : 0px; overflow: hidden;'>
  <a-scene embedded arjs='sourceType: webcam; debugUIEnabled: false;'>
    <a-marker type='pattern' url='assets/pattern-marker.patt'>
      <a-entity position='0 -6 -12' rotation="-20 0 0" gltf-model="url(assets/scene.gltf)"></a-entity>
    </a-marker>
  </a-scene>
</body>
```

在 `<a-scene />` 中，我們指定一個 attribute `arjs`，並且設定 `sourceType` 為 `webcam`，相關 attribute 設定其實來自 `artoolkit system`，有需要可以到 [GitHub 的列表](https://github.com/jeromeetienne/AR.js/tree/master/aframe#artoolkit-system)查看。

接著我們透過 `<a-marker />` 放入我們製作的 marker，副檔名為 `.patt`，這邊的 type attribute，如果你是單純用 barcode 的話，可以設為 `type=barcode`，但若是客製化的 marker，就要設為 `type=pattern`。

最後在 `<a-entity />` 上頭設定我們想要呈現的 AR 3D Model，`gltf-model` attribute 設定模型的路徑，再透過 `position` 與 `rotation` 來調整你的模型出現在鏡頭的位置。

這邊特別要注意一下，因為你載下來的 3D Model，都有自己的位置屬性，所以你可能會需要自己多加調整出適合的 `position` 與 `rotation` 值，否則你的 3D Model 很可能一直成像在你手機的鏡頭外而看不到，然後你還以為是程式出問題...

當手機鏡頭偵測到 `<a-marker />` 內對應的 marker 時，就會在鏡頭內渲染出 `<a-entiy />`。

如果只是像我一樣想要嘗試一下的話，可以簡單利用 Chrome 的 webapp - [Web Server for Chrome](https://chrome.google.com/webstore/detail/web-server-for-chrome/ofhbbkphhbklhfoeikjpcbhemlocgigb) 來 host 你的 htlm file，然後利用 [ngork](https://ngrok.com/) 或是 [serveo](https://serveo.net/) 來當作你 localhost 的 proxy，讓你的手機可以方便相連。

一切順利的話，你就會看到一隻傑尼龜出現在你家裡啦～

![another arjs demo](/img/arvinh/another-arjs-demo.jpg)

如果你沒有手機可以玩，想用電腦 Browser 跑的話，會需要到 `chrome://flags` 中把 `WebVR` 的選項開啟，然後用 inpsecter 將 Browser 調整成手機模式。

![chrome flags](/img/arvinh/chrome-flags.png)

並且需要加上 polyfill，因為普通的 chrome 是沒有支援 `navigator.xr` api 的：

```html
<script src='https://cdn.jsdelivr.net/npm/webxr-polyfill@latest/build/webxr-polyfill.js'></script>
<script>
  var polyfill = new WebXRPolyfill();
</script>
```

![run on chrome](/img/arvinh/arjs-run-mac.png)

## 結論

雖然 WebXR Device API 還不穩定，但就是在這時候開始試用才更能給出回饋，高手們來試試吧！決定下次拿公司測試機來跟著 Code lab 上的範例改改看！
另外，AR.js 雖然目前是 Marker-based，但從 [GitHub 上的一些討論](https://github.com/jeromeetienne/AR.js/issues/190) 來看，之後應該是有機會支援 Markerless 的。

AR 這樣牽扯到電腦視覺、硬體、演算法、Sensor 等複雜運算的技術，要實作到 Web 上更是困難，進度慢是可以理解的，但還是衷心期盼著那一天，能夠開啟網頁就能丈量傢俱尺寸或是試穿衣物！

## 資料來源

1. [Repository for the WebXR Device API Specification.](https://github.com/immersive-web/webxr)
2. [CreateＷebVR](https://createwebvr.com/webAR.html)
3. [AR.js](https://github.com/jeromeetienne/AR.js)
4. [AR.js — The Simplest Way to get Cross-Browser Augmented Reality on the Web](https://medium.com/chialab-open-source/ar-js-the-simpliest-way-to-get-cross-browser-ar-on-the-web-8f670dd45462)
5. [Building AR/VR with Javascript and HTML](https://blog.halolabs.io/building-ar-vr-with-javascript-and-html-97af4434bcf6)
6. [Web vs App (AR edition)](https://medium.com/agora-io/web-vs-app-ar-edition-d9aafe988ba2)
7. [Poly](https://poly.google.com/)
8. [sketchfab](https://sketchfab.com)
9. [用ar-js做一個讓另對方-喔喔喔喔-的小卡片吧](https://medium.com/%E9%AB%92%E6%A1%B6%E5%AD%90/%E7%94%A8ar-js%E5%81%9A%E4%B8%80%E5%80%8B%E8%AE%93%E5%8F%A6%E5%B0%8D%E6%96%B9-%E5%96%94%E5%96%94%E5%96%94%E5%96%94-%E7%9A%84%E5%B0%8F%E5%8D%A1%E7%89%87%E5%90%A7-4071ceea41dd)
10. [google code lab - WebXR](https://codelabs.developers.google.com/codelabs/ar-with-webxr/#0)
11. [WebXR Sample page](https://immersive-web.github.io/webxr-samples/)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化
