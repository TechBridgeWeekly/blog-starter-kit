---
title: 原來 CORS 沒有我想像中的簡單
date: 2018-08-18 08:00:49
tags: 
    - cors
    - pwa
author: huli
---

# 前言

CORS（Cross-Origin Resource Sharing，跨來源資源共享）在前端一直是個很經典的問題，簡單來說就是因為瀏覽器的一些安全考量，你在載入其他網域的資源時會受到一些限制，解決方法也很簡單，就是在 Server 那邊加上一些 response header 例如說 `Access-Control-Allow-Origin`，有了這個 header 之後瀏覽器就會認為你是有經過驗證的，就沒什麼問題了。

針對這個問題，我以前也有寫過一篇：[輕鬆理解 Ajax 與跨來源請求](https://blog.techbridge.cc/2017/05/20/api-ajax-cors-and-jsonp/)，裡面把碰到的問題與解法寫得十分詳細。

我以為自從我上次深入研究過這個問題之後，從此以後 CORS 再也難不倒我，再也不會看到 console 跳出禁止跨來源存取的錯誤。

但是我錯了。

這次我在一個特定的使用場景之下摔了一跤，但也因此學到不少。而這次的經驗也讓我想起我以前寫的：[我遇過的最難的 Cookie 問題
](https://blog.techbridge.cc/2017/03/24/difficult-problem-of-cookie/)。

太好了，又有東西可以跟大家分享了！

# 悲劇的開始

事情是這樣的，前陣子公司的產品改版進入收尾階段，嚴重的 bug 都修得差不多了，接下來就是要開始調整一些效能以及測試這次改版最重要的新功能：PWA！

還不知道什麼是 PWA 的我在這邊簡單說明一下，PWA 全名是 Progressive Web App，簡單來說就是把你的 Mobile Web 透過一些瀏覽器的支援變得更像是一個 App，最重要的就是你可以用 Service Worker 來快取任何的 request（連 API request 也行），做得好的話甚至在 offline 的狀況也能夠把這個網頁打開。

除此之外呢，透過瀏覽器可以把你的網站加到主畫面，就像是安裝在手機裡面那樣，變得跟一個 App 沒有兩樣。

下面附上三張截圖，會讓大家對 PWA 更有感覺，首先第一張是你可以把這個網頁加入主畫面：

![](/img/huli/cors/pwa1.jpg)

第二張是這個 PWA 就會跟其他的 Native App 一樣，存在你的手機裡面，你光看這頁絕對分不出來這到底是 Native App 還是 PWA。

![](/img/huli/cors/pwa2.jpg)

最後一張是你開啟這個 PWA 之後，會變成全螢幕，光看這個截圖根本就跟 Native App 沒兩樣。

![](/img/huli/cors/pwa3.jpg)

總之呢，可以把 PWA 想成：現有的網站 + 新技術（Service Worker、manifest.json...），搭配起來就可以變 PWA 了。

對 PWA 簡單的介紹就到這裡，想看更多的可以參考 @arvinh 寫過的：[Progressive Web App 會是未來趨勢嗎？](https://blog.techbridge.cc/2016/07/23/progressive-web-app/)或是[當 React web app 遇上 Progressive web app](https://blog.techbridge.cc/2016/09/17/create-react-pwa/)。

對 PWA 來說，其實最重要的就是這個 Service Worker（以下簡稱 SW），Chrome 內建的 Lighthouse 就可以針對網頁給一個 PWA 的分數，SW 就是其中一個考量的項目，因為你必須實作 SW 才能快取檔案並且實作出離線也能開啟 App 這個功能。

下圖是 Lighthouse 會檢測的項目：

![](/img/huli/cors/lh.png)

好，前情提要結束，可以來進入正題了。

我們的 PWA 該做的都做了，有註冊 SW，也有實作離線可以開啟的功能，可是卻發生了一件事，那就是在 Lighthouse 的檢測裡面，有一項永遠都過不了，那就是註冊 SW。

無論檢驗多少次，Lighthouse 都一直說我們的網站沒有註冊 SW。

超級無敵奇怪，我開無痕用乾淨的 Chrome 手動去測試，無論怎麼測我都確認一定有註冊 SW，可是 Lighthouse 怎麼測都說沒有。

那怎麼辦呢？

幸好 Lighthouse 是[開源](https://github.com/GoogleChrome/lighthouse)的，而且有提供 CLI 的版本，你可以自己在你電腦上面跑。

於是我想說既然 Lighthouse 說沒有，那我們就來看看 Lighthouse 是怎麼檢測的好了，然後我就稍微研究了一下 Lighthouse 的原始碼，覺得檢測的方法看起來也沒什麼問題，於是我決定把 Lighthouse 改一下，讓它跑完測試的時候不會把視窗關起來，這樣我就可以看 console 有沒有什麼有用的資訊，看看註冊成功會印的訊息有沒有印出來。

我稍微改動了幾個地方：

1. 增加設定檔，只跑 SW 的測試
2. 跑完之後不會把 Chrome 關掉
3. 在 SW 的檢查那邊印 log

如果有需要的，我更改的部分在這邊：[改動的地方的 PR](https://github.com/aszx87410/lighthouse/pull/1/files)

改完以後重新跑一遍測試，那一刻，我終於想起了被 CORS 困擾下的恐懼：

![](/img/huli/cors/sw-error.png)

# 撥雲見日

既然有了一些線索就應該好好追查下去，從截圖裡面看起來，SW 是成功註冊了，但是在用 SW 快取檔案的時候碰到一些錯誤，所以好像連帶影響了整個測試。總之呢，只要把這個 CORS 問題解決掉就沒事了吧。

先幫大家再做個背景說明，我們這些靜態檔案都是放在 Amazon S3，然後前面再掛 Cloudfront，我們已經有乖乖按照 [Amazon 的指示](https://docs.aws.amazon.com/AmazonS3/latest/dev/cors.html)加上該加的東西，所以只要 request header 有 origin，response 就一定會有 CORS 的 header，所以一定不會發生這個錯誤。

而 SW 在快取檔案的時候是用 fetch，所以也一定會加上 origin 這個 header，沒理由出問題。

大概卡了一兩個小時還是還無頭緒，於是我決定看一下 network 的 tab，發現了更多線索：

下面這一張是從 SW 裡面發出的 request，header 裡面確實有 origin，可是 response 卻沒有 `Access-Control-Allow-Origin`！
![](/img/huli/cors/sw-r1.png)

除此之外，發現更早以前也有一個相同的 request，因為這個 request 是由 `<script>` 發出的，所以不會帶上 origin，因此 response 也就沒有 CORS 的 header。
![](/img/huli/cors/sw-r2.png)

而這邊值得注意的事情是，第二個 response 是 from disk cache（雖然上面兩個都是，但那是因為我截圖的時候東西沒清空，事實上應該只有第二個是）
![](/img/huli/cors/sw-tab.png)

查線索查到這裡，大概有點頭緒了。

# 深入追查

好，讓我先來解釋一下。

SW 所要快取的那個檔案是頁面會載入的其中一個 JavaScript，而因為頁面會載入，所以在 HTML 裡面我們放了一個 `<script>` 的 tag 來載入這個檔案，從上面的圖片看起來，瀏覽器先載入了這個 JavaScript 檔案，然後因為這個檔案不是用 ajax 發出，所以沒有 origin，根據 S3 的規定，也就自然沒有 `Access-Control-Allow-Origin`。

再來呢，SW 註冊成功，開始執行裡面的程式碼把我們預先準備好的清單給快取住，其中一個就是這個 JavaScript 的檔案，但是當我們用 fetch 來抓這個檔案時，瀏覽器直接用了快取住的前一個的 response（因為 URL、method 都一樣），而這個 response 是沒有 `Access-Control-Allow-Origin` 的！因此就跳出了我們最前面看到的跨網域的錯誤。

到這邊真相大白了，都是瀏覽器快取的問題。

那為什麼我之前自己測都測不出來呢？因為身為一個前端工程師，devtool 有把「Disable cache」打勾是很合情合理的事，所以我怎麼試都試不出來這個問題。

知道問題的成因之後就比較簡單了，拜了 Google 大神查到了這一篇 Chromium 的 ticket：[CORS Preflight Cache Does not Consider Origin](https://bugs.chromium.org/p/chromium/issues/detail?id=260239)

裡面碰到的問題基本上跟我碰到的差不多，最後給的解法是 response 加上一個 `Vary: Origin`，讓瀏覽器知道如果 Origin 不一樣的話就不要用快取，可是我發現我們早就加了但不知道為什麼沒用。

除此之外也找到幾個類似的問題：

1. [Chrome S3 Cloudfront: No 'Access-Control-Allow-Origin' header on initial XHR request](https://serverfault.com/questions/856904/chrome-s3-cloudfront-no-access-control-allow-origin-header-on-initial-xhr-req)
2. [S3 CORS, always send Vary: Origin](https://stackoverflow.com/questions/31732533/s3-cors-always-send-vary-origin)

後來採用裡面其中一個的解法：「既然 S3 要有 origin header 才能開啟 CORS，那就用 Cloudfront 傳一個固定的 origin 給它吧！這樣每個 response 都一定會有 `Access-Control-Allow-Origin` 了！」

可以參考這篇：[AWS CloudFront + S3 + Allow all CORS](http://strd6.com/2017/05/aws-cloudfront-s3-allow-all-cors/)，基本上就是調一個設定而已。

這招聽起來滿有效的，但其實不是最好的解法，感覺有點骯髒，畢竟 origin 這東西不是這樣用的，為了 S3 的機制硬要這樣做總覺得不是太好。

於是最後我就想到一個東西，同時也解決了心中的一個疑惑。

那就是在`<script>`加上 `crossorigin="anonymous"`，讓`<script>`發出去的 request 也有 origin header！

我以前就看到某些地方會加這個，但始終不懂為什麼要加，因為 script 本來就可以不限制網域，為什麼還要特地加一個 tag 讓它變成像 ajax 那樣的 request？

但沒想到我居然被這個屬性幫助到了，一旦我加了這個，那 script 的載入就會附上 Origin，S3 就會回傳 `Access-Control-Allow-Origin`，也就不會碰到之後的跨網域問題了！

至於這個屬性其他的功用，可參考：[Purpose of the crossorigin attribute …?](https://stackoverflow.com/questions/18336789/purpose-of-the-crossorigin-attribute)

# 總結

要碰到我所碰到的這個問題，你必須同時滿足下面四個條件：

1. 你把靜態檔案放在 S3 上面
2. 你沒有勾選瀏覽器的 Disable cache
3. 你用 script 跟 SW 載入同一個檔案
4. 瀏覽器用快取的 script 的 response 回應 SW 的請求

只要任何一個條件沒有滿足，就不會碰到這問題。換句話說，要踩到這個坑其實也挺困難的。

但坑踩的越多就越強，解決了一個問題就代表你未來會碰到的問題又少了一個。解決掉這個 CORS 相關的問題，我想我以後應該不會再碰到相關的問題了...吧。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好