---
title: PWA 實戰經驗分享
date: 2018-10-13 09:14:47
tags:
  - pwa
author: huli
---

# 前言

前些日子在忙公司的產品改版，從原本的 PHP 換成後端 Go + 前端 React SPA，分成桌面版跟手機版兩個不同的 Project，而既然都改版了，自然要把最新最潮的 PWA 也放在目標裡面，之前耳聞 PWA 很久但卻沒有實作過的我也有了機會來嘗試這個東西。

如今產品已經改版完畢且上線了兩三個月，慢慢穩定下來，在優化 PWA 的過程中也讓我有了一些心得可以讓大家分享。

在舉一些實際案例之前，先讓我們來談談到底怎樣才算是 PWA。

# PWA 到底是什麼？

從 Google 官方的文件：[你的首個 Progressive Web App](https://developers.google.com/web/fundamentals/codelabs/your-first-pwapp/?hl=zh-tw) 中可以看出 PWA 的一些詳細定義，但我不太喜歡這種制式的規則，對我來說，PWA 就是一個很像 Native App 的 Web App，而其中瀏覽器的支援也佔了很大的一部分。

在以往儘管你的網站做得再怎麼像 Native App，你還是有兩個難關沒辦法克服：離線的時候就 GG 以及沒辦法安裝在手機上，所以不管怎麼看，人家都知道你就是個 Web App，永遠都不會長得像 Native。

可是自從瀏覽器開始支援 Service Worker 以及 manifest 之後，上面這兩點就被克服了！得益於 Service Worker，讓網頁離線的時候也能夠運作，可以自己寫 code 來決定要渲染什麼畫面；而瀏覽器的「新增到主畫面」的功能更是讓安裝 Web App 成為可能，開發者也可以用 manifest.json 來自訂一些內容，像是啟動畫面以及安裝在主畫面上的名稱等等。

對我來說，如果你能夠利用上面這兩項技術，讓你的 Web App 成功安裝在手機上並且看起來跟 Native App 沒兩樣，我覺得就能稱作是 PWA。

我在之前的文章（[原來 CORS 沒有我想像中的簡單](https://blog.techbridge.cc/2018/08/18/cors-issue/)）已經有分享過 PWA 在手機上面的樣子了，這邊就不再贅述。還記得我第一次體驗安裝 PWA 的時候也被嚇到了，因為看起來就跟 Native App 沒兩樣，如果真的做得好，應該是很難區分出來的。明明是個網頁可是看起來卻跟 Native App 一樣，這就是 PWA。

接著來介紹幾個 PWA 的重要因素，你要做 PWA 就一定要有下面幾個東西。

# manifest.json

首先先來談`manifest.json`，有寫過 Android 的都知道有個東西叫做`AndroidManifest.xml`，其實兩個本質上是一樣的東西，就是去描述這個 App 的一些特性。

我們先來看看 Google 官方文件：[The Web App Manifest](https://developers.google.com/web/fundamentals/web-app-manifest/)裡面給的範例：

``` json
{
  "short_name": "Maps",
  "name": "Google Maps",
  "icons": [
    {
      "src": "/images/icons-192.png",
      "type": "image/png",
      "sizes": "192x192"
    },
    {
      "src": "/images/icons-512.png",
      "type": "image/png",
      "sizes": "512x512"
    }
  ],
  "start_url": "/maps/?source=pwa",
  "background_color": "#3367D6",
  "display": "standalone",
  "scope": "/maps/",
  "theme_color": "#3367D6"
}
```

裡面給的資訊很簡單，然後會跟你把 PWA 新增到主畫面時出現的東西息息相關。`name`的話就是你 App 的名稱，他在主畫面上面就會顯示這個`name`，但如果你也有提供`short_name`的話會優先使用`short_name`。

再來`icons`就是主畫面上面會出現的 logo 囉，這沒什麼好多談的。`start_url`則是你從主畫面上開啟時會連線到的地方，很多人會加個`?source=pwa`之類的，這樣就可以知道這個使用者是使用 PWA，方便做一些統計。

這邊有個小地方要注意，那就是在某一版的 iOS Safari（抱歉我忘記是哪一版，但總之最新的已經沒有這個問題了），它是不會遵守`start_url`的！他會根據的是你在安裝 PWA 時的網址，例如說你在`https://example.com/test/123`的時候按下「新增到主畫面」，你在主畫面開啟 PWA 時就會連線到這個畫面。

這部分其實滿困擾的，但幸好最新的 iOS Safari 已經沒有這個問題了，大家可以不用擔心。

還有一個要特別提的就是`name`、`background_color`跟`icon`會自動組成`Splash screens`，就是你在打開 PWA 的時候會看到的一個畫面，是由這三個資訊自動被 Chrome 所組成的，意思就是你沒辦法客製化這個啟動畫面。

它就是會顯示你指定的背景顏色、然後中間放一個 icon 下面放你 App 的名稱，沒有其他東西可以調了，至少現在是這樣。

在這點上面 iOS 就不一樣，iOS 是不支援這種啟動畫面的，但好處就是你可以自己透過 html 的 tag 來設定！

``` html
<link
    rel='apple-touch-startup-image'
    href='/assets/splash/splash-1125x2436.png'
    media='(device-width: 375px) and (device-height: 812px) and (-webkit-device-pixel-ratio: 3) and (orientation: portrait)'
/>
```

會有一些尺寸相關的設定你因為要幫每一種不同的 device 都準備一張圖片，詳情可參考：[Progressive Web App Splash Screens](https://medium.com/@applification/progressive-web-app-splash-screens-80340b45d210) 或是 [Few Tips That Will Make Your PWA on iOS Feel Like Native](https://www.netguru.co/codestories/few-tips-that-will-make-your-pwa-on-ios-feel-like-native)。

iOS 跟 Android 的差別在於 iOS 的啟動畫面你可以放一張圖片，因此可以完全客製化，你想放什麼就放什麼，自由度比 Android 來得高。

還有就是 icon 的部分，iOS 也不會看你`mainfest.json`的設定，而是會看自己的 html tag，所以你必須額外設置給 iOS 使用的 icon：

``` html
<link
    rel='apple-touch-icon'
    sizes='192x192'
    href='/assets/favicons/iOS192x192.png'
/>
```

對於`manifest.json`，該注意的點差不多就這些。其實最大的問題還是支援度，所以 Google 出了一個`PWACompat`，可以自動幫你針對舊的瀏覽器調整你的檔案以及 html 的 tag，不過也有人寫了一篇：[You shouldn’t use Chrome’s PWACompat library in your Progressive Web Apps](https://medium.com/@firt/you-shouldnt-use-chrome-s-pwacompat-library-in-your-progressive-web-apps-6b3496faab62) 來告訴大家不要用，論點大概是不能這樣一概論之，你必須針對每種不同的平台跟瀏覽器去了解他的差異再來做適配，才能得到最好的使用者體驗，這種統一調整的做法會在很多地方看起來 ok 但是怪怪的。

既然上面都提到 iOS 了，就來講講 iOS 的一些不同之處。其實 iOS 開始提供 PWA 的支援是今年（2018 年）的事情而已，而且剛推出的時候支援度滿差的，不過有在慢慢改善就是了。

關於那些 iOS 不同的地方，這兩篇文章都講得很清楚了：[PWAs are coming to iOS 11.3: Cupertino, we have a problem](https://medium.com/@firt/pwas-are-coming-to-ios-11-3-cupertino-we-have-a-problem-2ff49fd7d6ea)、[Progressive Web Apps on iOS are here 🚀](https://medium.com/@firt/progressive-web-apps-on-ios-are-here-d00430dee3a7)。

最大的差異之一大概就是很多時候都不看`manifest.json`，你要自己額外設置一些相對應的 html tag 才有用，這點是要特別注意的。

再來就是`<meta name=”apple-mobile-web-app-capable” content=”yes”>`這個 tag 也很重要，主要是告訴瀏覽器説：「我準備好提供全螢幕的體驗了，就算隱藏瀏覽器的 UI 也沒關係」，而這篇：[Don’t use iOS meta tags irresponsibly in your Progressive Web Apps](https://medium.com/@firt/dont-use-ios-web-app-meta-tag-irresponsibly-in-your-progressive-web-apps-85d70f4438cb) 則告訴你千萬不要濫用這個 tag，不然你的 Web App 在 Safari 上的體驗會變得很差，因為很多東西都不支援。

至於 Safari 最大的一點問題我直接引用上面 [PWAs are coming to iOS 11.3: Cupertino, we have a problem](https://medium.com/@firt/pwas-are-coming-to-ios-11-3-cupertino-we-have-a-problem-2ff49fd7d6ea) 的其中一段：

> Also, it’s a massive problem for apps with two-factor authentication, such as Twitter. If you need to go to another app to get a token or to open a text message or an email, you will get out of the PWA. When you go back to paste the code, you are out of context and, you need to start the login process again losing the validity of that code. It happened to me on Twitter! Which means, the Twitter PWA on iOS is completely unusable for me.

這是什麼意思呢？我直接舉一個實際範例，假如你的 PWA 有提供 Facebook 登入的功能而且是用重新導向的方式，你一點下去他就會開一個新的 Safari 視窗連到 Facebook 讓你授權，可是當你授權完之後回到 PWA，你會發現什麼事情都沒發生。

這一點真的超傷，而且現在應該都還沒修好，只能期待 Safari 之後會把問題修掉了。而 Android Chrome 則是會在同一個視窗底下打開 Facebook，因此結束之後能夠順利完成登入流程。

有關於 iOS 的問題跟`manifest.json`的注意事項差不多就到這邊，再來我們談談 PWA 的第二個重點：Service Worker。

# Service Worker

加入 Service Worker 的目的就只有一個，那就是快取。透過 Service Worker（以下簡稱 SW），可以幫助我們在發送 request 之前就先攔截到並且做處理，而離線運行的原理也是這樣的，我們先在第一次開啟時註冊 SW，並且利用 SW 下載靜態檔案並快取住，之後若使用者離線，我們再用已經快取住的檔案來回覆，就不會發送真的 request，自然也不會發生無法連線的情況。

而 Google 有提供了一個方便的工具：[Workbox](https://developers.google.com/web/tools/workbox/) 來幫助我們自動產生出 SW 以及利用更方便的語法來攔截 request。

舉例來說，我自己用的是 Webpack 的 plugin：

``` js
new workboxPlugin.InjectManifest({
    swSrc: path.join(__dirname, '..', SRC_DIR, 'sw.js'),
    swDest: path.join(__dirname, '..', DIST_DIR, 'sw.js'),
    globDirectory: path.join(__dirname, '..', DIST_DIR),
    globPatterns: ['**/*.{js,css}']
}),
  
//sw.js
let precacheList = self.__precacheManifest || []
workbox.precaching.precacheAndRoute(precacheList)
```

只要這樣一寫，就會自動去找符合規則的檔案並且加入快取清單裡面，你只要一註冊 SW 的時候就會把那些檔案給快取起來。

除此之外呢，Workbox 也可以針對 URL 來監聽：

``` js
// sw.js
workbox.routing.registerRoute(/(https?:\/\/)(.*)\/api\/(.*)/, args =>
    workbox.strategies
        .networkFirst({
            cacheName: 'data-cache',
            plugins: [
                new workbox.expiration.Plugin({
                    maxEntries: 100,
                    maxAgeSeconds: 2592000
                })
            ]
        })
        .handle(args)
        .then(response => {
            return response
        })
        .catch(err => {
            console.log('err:', err)
        })
)
```

像上面的程式碼就是針對路徑中含有`api`的 request 做快取，這樣在離線時也可以利用以前快取住的 API response。

Workbox 針對這種動態的快取提供幾種策略，分別是：`staleWhileRevalidate`、`cacheFirst`、`networkFirst`、`networkOnly `與`cacheOnly`，其實看名字就可以大概理解策略是什麼了，想知道詳細的內容可以參考官方文件：[Workbox Strategies](https://developers.google.com/web/tools/workbox/modules/workbox-strategies)。

總之自從有了 Workbox 之後，基本上就不用自己手寫 SW 了，都靠著它提供的 API 以及功能就行了，就可以自動產生出符合需求的 SW。

# Add to home screen banner

最後要來談的是「安裝 PWA」這一塊，在 iOS Safari 上面別無他法，就只能自己叫出選單然後選取「Add to home screen」，可是在 Android Chrome 上面，如果你符合一定的條件（有設置`mainfest.json`以及有註冊 Service Worker），就會自動幫你跳出一個可愛的 Install banner。

![](/img/huli/pwa.png)
（圖片來自：[Changes to Add to Home Screen Behavior](https://developers.google.com/web/updates/2018/06/a2hs-updates)）

根據 Chrome 版本的不同，行為也有所不同。

在 Chrome 67（含）以前的版本，如果你在`beforeinstallprompt`事件裡面沒有特別用`preventDefault()`，或是顯式的呼叫了`prompt()`，就會出現最左邊那個頗大的 A2HS banner。

然後在 Chrome 68（含）之後的版本，無論你做了什麼，系統都會自動出現那個 Mini-infobar，但如果使用者關掉的話，要隔三個月才會再出現一次，實在是有夠久。

接著呢，上面這兩個 A2HS banner 跟 Mini-infobar，使用者點擊之後都會出現最右邊的 A2HS Dialog，提示使用者要不要安裝 PWA。

但是在 Chrome 68 以後，你也可以利用程式去呼叫`beforeinstallprompt`裡面拿到的`event.prompt()`把這個 dialog 顯示出來。

聽起來有點複雜對吧？

先來介紹`beforeinstallprompt`這個 event 好了，這個 event 在一切都準備就緒，確認你滿足條件可以顯示 prompt 的時候會被觸發，會傳來一個 event，你可以阻止顯示 prompt，把這個 event 存起來：

``` js
// 此範例來自上面的官方文件
let installPromptEvent;
  
window.addEventListener('beforeinstallprompt', (event) => {
  // Prevent Chrome <= 67 from automatically showing the prompt
  event.preventDefault();
  // Stash the event so it can be triggered later.
  installPromptEvent = event;
  // Update the install UI to notify the user app can be installed
  document.querySelector('#install-button').disabled = false;
});
```

為什麼要存起來呢？因為使用者可能不想一打開網站就看到這個彈窗，或者他可能正在結帳結果你跳這個東西來干擾他，所以先把它存起來，等適當的時機再呼叫`installPromptEvent.prompt()`來跳出 Dialog。

但要注意的事情是你直接呼叫`installPromptEvent.prompt()`是沒用的，你必須要`within a user gesture`，意思就是你要放在按鈕的 click 事件（或其他由使用者觸發的事件）裡才有效，直接呼叫是沒有用的，而且會看到 console 跳出錯誤訊息。

我之前一度很好奇它是怎麼做判斷的，後來發現原來有`event.isTrusted`可以用，可以判斷一個事件是不是被使用者主動觸發的，參考資料：[MDN - Event.isTrusted](https://developer.mozilla.org/en-US/docs/Web/API/Event/isTrusted)。

總之呢，因為在不同版本上的 Chrome 有不同行為，所以最後我們決定用下面的程式碼針對不同版本有不同的反應：

``` js
// 把 event 存起來
var installPromptEvent
  
// 要顯示 prompt 的延遲
var showTime = 30 * 1000
  
window.addEventListener('beforeinstallprompt', function (e) {
  e.preventDefault()
  installPromptEvent = e
  var data = navigator.userAgent.match(/Chrom(e|ium)\\/([0-9]+)\\./)
  var version = (data && data.length >= 2) ? parseInt(data[2], 10) : null
  if (version && installPromptEvent.prompt) {
  
    // 延遲一段時間才顯示 prompt
    setTimeout(function() {
        // 如果 Chrome 版本是 67（含）以下，可以直接呼叫
        if (version <= 67) {
            installPromptEvent.prompt()
            return
        }
  
        // 否則的話必須透過 user action 主動觸發
        // 這邊幫 #root 加上 event listener，代表點擊螢幕任何一處都會顯示 prompt
        document.querySelector('#root').addEventListener('click', addToHomeScreen)    
    }, showTime)
  }
});
  
function addToHomeScreen(e) {
    if (installPromptEvent) {
        installPromptEvent.prompt()
        installPromptEvent = null
        document.querySelector('#root').removeEventListener('click', addToHomeScreen) 
    }
}
```

如果是 67 以下，直接呼叫就可以顯示 prompt，否則的話還要再一步，要加個 event listener 才行，而我們也選擇延遲 30 秒才顯示。

出乎意料地，這樣一個小改動帶來驚人的成長，原本一天大概才 20、30 個人安裝 PWA，經過這樣調整之後瞬間變成八到十倍，看到 GA 的那個統計圖我也嚇了一跳，沒想到效果這麼好。

與其一直積極地要別人快點安裝 PWA，還不如只要求真的對你產品有興趣（停留超過 30 秒鐘）的人。

# manifest 觀摩

最後我們來看看幾個知名的 PWA 都是怎麼寫他們的`manifest.json`。

第一個是 PWA 界中很有名的 [flipkart](https://www.flipkart.com/)：

``` js
{
    "name": "Flipkart Lite",
    "short_name": "Flipkart Lite",
    "icons": [
        {
            "src": "https://img1a.flixcart.com/www/linchpin/batman-returns/logo_lite-cbb3574d.png",
            "sizes": "192x192",
            "type": "image/png"
        }
    ],
    "gcm_sender_id": "656085505957",
    "gcm_user_visible_only": true,
    "start_url": "/?start_url=homescreenicon",
    "permissions": [
        "gcm"
    ],
    "orientation": "portrait",
    "display": "standalone",
    "theme_color": "#2874f0",
    "background_color": "#2874f0"
}
```

再來是鼎鼎大名的 [twitter](https://mobile.twitter.com/manifest.json)：

``` js
{
  "background_color": "#ffffff",
  "description": "It's what's happening. From breaking news and entertainment, sports and politics, to big events and everyday interests.",
  "display": "standalone",
  "gcm_sender_id": "49625052041",
  "gcm_user_visible_only": true,
  "icons": [
    {
      "src": "https://abs.twimg.com/responsive-web/web/ltr/icon-default.604e2486a34a2f6e.png",
      "sizes": "192x192",
      "type": "image/png"
    },
    {
      "src": "https://abs.twimg.com/responsive-web/web/ltr/icon-default.604e2486a34a2f6e.png",
      "sizes": "512x512",
      "type": "image/png"
    }
  ],
  "name": "Twitter",
  "share_target": {
    "action": "compose/tweet",
    "params": {
      "title": "title",
      "text": "text",
      "url": "url"
    }
  },
  "short_name": "Twitter",
  "start_url": "/",
  "theme_color": "#ffffff",
  "scope": "/"
}
```

最後則是 [Google I/O 2018](https://events.google.com/io)：

``` js
{
  "name": "Google I/O 2018",
  "short_name": "I/O 2018",
  "start_url": "./?utm_source=web_app_manifest",
  "display": "standalone",
  "theme_color": "#6284F3",
  "background_color": "#6284F3",
  "icons": [{
    "src": "static/images/homescreen/homescreen57.png",
    "sizes": "57x57",
    "type": "image/png"
  }, {
    "src": "static/images/homescreen/homescreen114.png",
    "sizes": "114x114",
    "type": "image/png"
  }, {
    "src": "static/images/homescreen/homescreen128.png",
    "sizes": "128x128",
    "type": "image/png"
  }, {
    "src": "static/images/homescreen/homescreen144.png",
    "sizes": "144x144",
    "type": "image/png"
  }, {
    "src": "static/images/homescreen/homescreen192.png",
    "sizes": "192x192",
    "type": "image/png"
  }, {
    "src": "static/images/homescreen/homescreen512.png",
    "sizes": "512x512",
    "type": "image/png"
  }],
  "prefer_related_applications": false,
  "related_applications": [{
    "platform": "play",
    "id": "com.google.samples.apps.iosched"
  }],
  "gcm_sender_id": "103953800507"
}
```

我滿喜歡觀察別人家的這些東西，因為你會發現很多你查資料時遺漏或是根本找不到的資訊，而且這些看久了你也會有個概念，知道哪些屬性特別常用，除了`manifest.json`以外，也可以參考 html 裡面的 tag，一樣能學習到很多。

# 結論

前陣子在與 PWA 奮戰以及被 PM 的夾擊之下，搜集了很多跟 PWA 有關的資料，也參考了許多很有用的文章，真心感謝那些前輩們的分享，才能避免後人踩一大堆坑。

雖然在 iOS 上的體驗差了點，但整體來說我還是很看好 PWA 的發展，第一個是 Google 強力推動，第二個是瀏覽器的支援度愈來愈高，就像我上面說的，iOS Safari 已經有慢慢把 Bug 給修掉了，之後的功能會比較完整一些。

再者，PWA 的使用者體驗是很不錯的，有可以接受的速度以及 Web 的彈性，重點是不用去 Google Play 特地下載就少了一道轉換的門檻（雖然還是有安裝 PWA 的門檻就是了，但我覺得比較容易一些），而 Chrome 也提供了許多機制給 PWA，希望使用者能安裝 PWA 在手機上。

總之呢，這篇主要是跟大家分享我在做 PWA 時候的一些小小心得，如果你也有什麼心得歡迎在底下留言跟我分享，感謝。

延伸閱讀與參考資料：

1. [Changes to Add to Home Screen Behavior](https://developers.google.com/web/updates/2018/06/a2hs-updates)
2. [Progressive Web App Splash Screens](https://medium.com/@applification/progressive-web-app-splash-screens-80340b45d210)
3. [Few Tips That Will Make Your PWA on iOS Feel Like Native](https://www.netguru.co/codestories/few-tips-that-will-make-your-pwa-on-ios-feel-like-native)
4. [PWAs are coming to iOS 11.3: Cupertino, we have a problem](https://medium.com/@firt/pwas-are-coming-to-ios-11-3-cupertino-we-have-a-problem-2ff49fd7d6ea)
5. [Progressive Web App 會是未來趨勢嗎？](https://blog.techbridge.cc/2016/07/23/progressive-web-app/)
6. [PWA case studies](https://developers.google.com/web/showcase/)
7. [A Pinterest Progressive Web App Performance Case Study](https://medium.com/dev-channel/a-pinterest-progressive-web-app-performance-case-study-3bd6ed2e6154)

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好