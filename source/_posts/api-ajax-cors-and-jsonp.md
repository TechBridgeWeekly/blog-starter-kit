---
title: 輕鬆理解 Ajax 與跨來源請求
date: 2017-05-20 15:31:12
tags: 
  - xhr
  - ajax
  - cors
  - jsonp
author: huli
---

# 前言

一般來說在學習寫網頁的時候，最先碰到的會是 HTML 與 CSS，負責把版面刻出來以及美化版面，當基礎打穩之後，會開始學習 JavaScript，試著做出一點互動性的效果。而「互動」除了使用者跟瀏覽器的互動以外，別忘了還有 Client 端跟 Server 端的互動，也就是必須要學會從瀏覽器用 JavaScript 跟後端 Server 拿資料，否則你的網頁資料都只能是寫死的。

這篇的主要預設讀者是網頁前端的初學者，希望能讓本來不太理解怎麼跟 Server 交換資料或是怎麼串 APi 的讀者看完之後，能夠更了解該怎麼跟後端串接。

# 先從舉例開始

在開始之前，我們可以先想想一個問題：

> 為什麼前端必須跟後端交換資料？

其實這跟你做的網頁類型有關，假如說你今天做的是一個官方網站，很可能整個網站都是靜態的，只要 HTML 跟 CSS 就可以了，並不需要跟後端 Server 拿資料。

那我們先假設今天要做的是一個可以瀏覽現在 Twitch 實況列表的網頁好了，如下圖。

![](/img/huli/twitch.png)

如果這個網頁不跟後端拿資料，就代表說網頁的內容都是固定的，無論什麼時候看都一樣。可是這樣的話就不對了嘛，因為這個網頁的目標是顯示出「現在有在開實況的頻道」，所以內容是會跟著改變的。

既然會跟著改變，就必須不斷的去更新資料，從 Server 那邊拿資料回來，接著在前端處理過後顯示。

已經確認有拿資料的必要性之後，就可以問自己兩個問題：

1. 要跟誰拿資料？
2. 要怎麼拿資料？

第一個問題，很明顯的就是跟 Twitch，因為 Twitch 才有你要的這些資料嘛！

那第二個問題，要怎麼拿資料呢？就必須透過 Twitch API 了。

# API

什麼是 API？你可能已經聽過很多次這個名詞，但還是不知道是什麼。先來講講它的全名吧，它的全名是「Application Programming Interface」，中文翻作應用程式介面。

你可能會想說這是什麼鬼東西，怎麼中文英文我都看不懂在幹嘛。但其實這幾個字裡面，最重要的是「介面」兩個字。

介面是什麼？介面就是拿來串接用的，我舉一個例子你就知道了。

電腦上不是有一個 USB 插槽嗎？然後你只要是看到市面上有賣 USB 隨身碟的，都可以買來以後插到 USB 插槽，你的電腦就可以讀取得到。你有想過為什麼嗎？明明就是不同廠商做的東西，可是卻都可以讀得到、都可以插到 USB 插槽裡面。

因為有一項標準叫做 USB 介面，當這套標準訂出來以後，所有廠商只要按照這一套標準來開發，就可以保證能夠連接電腦跟 USB 隨身碟。

API 也是這樣，只是變成程式跟程式之間的串接。例如說今天我寫程式需要讀取檔案好了，我要怎麼讀取檔案？讀取檔案是作業系統提供的功能，因此我可以去串接「讀取檔案的 API」，就可以在我的程式裡面也使用這個功能了。

再多舉幾個例子你可能會更有感覺。

例如說今天我想要讓我的網頁能夠用 Facebook 登入，那要怎麼辦？我就要去串接「Facebook 提供的 API」，就等於說是 Facebook 向外提供給大家的一套介面、一套標準，任何想要接入 Facebook 服務的開發者們，都可以遵循著那套規範拿到自己想要的資料，這個東西就叫做 API。

或是你可能今天是一個飯店管理系統的開發者，你們公司做了一套給飯店用的 ERP，可以管理飯店的訂房狀態等等，就能知道說現在有哪些房間是空的。

而這些資料如果只有自己用太可惜了，於是公司決定把這些資料提供給大型訂房網站，在那些網站上能夠即時顯示這間飯店的房間狀況。所以就必須交換資料，你要提供一個「查詢房間情形的 API」給其他網站，讓他們能夠去串接，才能獲得這些資訊。

講到這邊，大家應該對 API 已經有一些 sense 了，我再多舉幾個例子：

1. 我想要抓到 flickr 上面的照片，所以我要去串接 flickr 的 API
2. Google 要開放讓其他 App 也能用 Google 登入驗證，所以 Google 要提供「Google 登入 API」
3. 我要抓 Twitch 上面現在有哪些頻道，所以要串 Twitch API

# API Documentation

既然已經知道 API 是什麼了，也知道要串接 API，那下一個問題就是「那要怎麼串呢？」

剛剛前面有提過檔案存取的例子，其實這個比較像是呼叫作業系統或是程式語言的函式庫提供的 Function，而這些 Function 你通常都可以在官方文件上查到更詳細的說明，例如說 Node.js 的讀取檔案：

![](/img/huli/fs.png)

（來源：https://nodejs.org/api/fs.html#fs_fs_readdir_path_options_callback）

上面就有寫說你應該呼叫哪一個 Function，應該傳入哪些參數。

API 的串接也是一樣，一定要有文件你才知道怎麼串，不然根本串不起來，因為你連要傳什麼參數都不知道。

我們可以先來看看[Twitch API 文件](https://dev.twitch.tv/docs/v5/guides/using-the-twitch-api/)是怎麼寫的。

裡面說明了你必須要有一個`Client ID`，然後 API Root 的 URL 是 `https://api.twitch.tv/kraken` 等等，這些都是與 API 相關的基本資訊。如果你在左側欄隨便點一個 API，會看到個別 API 的詳細資訊：

![](/img/huli/twitch2.png)

這邊就有寫說網址是什麼，你應該傳的參數是什麼等等，下面還有附上參考範例，這就是一個很完整的 API 文件。

通常在寫網頁的時候，我們都會直接講 API，但其實我們指的是 Web API，也就是透過網路來傳輸的 API。那有沒有非 Web API 呢？有，像我們前面提到的跟作業系統要讀檔的 API，就都是在本機底下執行的，沒有透過任何網路。

不過這其實也不用太在意，反正大家都習慣講 API，聽得懂就好。

現在有了 API 文件，我們就有了所有我們需要的資訊。以上面這個 Twitch 的例子來講，我們只要能夠發送 Request 到`https://api.twitch.tv/kraken/games/top?client_id=xxx`，Twitch 就會傳回目前最熱門的遊戲列表。

我們已經把問題的範圍一步步給縮小了，一開始是「要怎麼跟 Twitch 拿資料」，現在則更細的切分為：「要怎麼利用 JavaScript 發送 Reuqest」

# Ajax

要在瀏覽器上面發送 Request，必須應用到一種技術叫做 Ajax，全名是「Asynchronous JavaScript and XML」，重點在於`Asynchronous`這個單字，非同步。

在講什麼是非同步之前，就要先來提一下什麼是同步。你原本寫的 JavaScript 就幾乎都是同步執行的。意思是他執行到某一行的時候，會等這行執行完畢，才執行到下一行，確保執行順序。

也就是說下面這段程式碼，最後一行需要等很長一段時間才能執行到：

``` js
var count = 10000000;
while(count--) {
  // 做一些耗時的操作
}
  
// 等很久才被執行到
console.log('done')
```

看起來滿有道理的，程式本來不就是一行一行執行的嗎？可是如果今天牽涉到網路操作的話，大家可以思考看看下面這個例子：

``` js
// 假設有個發送 Request 的函式叫做 sendRequest
var result = sendRequest('https://api.twitch.tv/kraken/games/top?client_id=xxx');
  
// 等很久才被執行到
console.log(result);
```

當 JavaScript 執行到`sendRequest`的時候，由於是同步的，就會等待 Response 回來才繼續做事。換句話說，在 Response 還沒回來之前，整個 JavaScript 引擎是不會執行任何東西的！很可怕對吧，你點任何有牽涉到 JavaScript 的東西，都不會有反應，因為 JavaScript 還在等 Response 回來。

所以呢，像是這種已經預期到可能非常耗時間，非常不穩定的操作，就不能用同步的方式來執行，而是要用非同步。

非同步是什麼意思呢？就是執行完之後就不管它了，不等結果回來就繼續執行下一行：

``` js
// 假設有個發送 Request 的函式叫做 sendRequest
var result = sendRequest('https://api.twitch.tv/kraken/games/top?client_id=xxx');
  
// 上面 Request 發送完之後就執行到這一行，所以 result 不會有東西
// 因為 Response 根本沒有回來
console.log(result);
```

這邊需要特別注意的是「非同步的 Function 不能直接透過 return 把結果傳回來」，為什麼？因為像上面這個例子，它發送 Request 之後就會執行到下一行了，這個時候根本就還沒有 Response，是要回傳什麼？

那怎麼辦呢？先聽我舉個很常見的小例子吧！

我之前在新加坡的 Food Court 吃飯的時候，那邊每一張桌子上面都會有桌號。你去點餐的時候，只要跟老闆講說你坐哪一桌，等餐點完成之後老闆就會自己主動送過來。

所以我不需要站在店家門口等，我只要在位子上繼續坐我的事情，反正餐點好了之後老闆會送過來。

非同步的概念也是這樣，我發送 Request 之後（我點餐之後），我不用等 Response 回來（不用等老闆做好），可以繼續做自己的事，等
 Response 回來之後（等餐點做好之後），會自己幫我把結果送過來（老闆會自己送過來）。

在點餐的例子中，老闆可以透過桌號知道應該把資料送到哪邊，那在 JavaScript 裡面呢？可以透過 Function！而這個 Function，我們就稱作 Callback Function，回呼函式。

當非同步的操作完成時，就可以呼叫這個 Function，並且把資料帶進來。

``` js
// 假設有個發送 Request 的函式叫做 sendRequest
sendRequest('https://api.twitch.tv/kraken/games/top?client_id=xxx', callMe);
  
function callMe (response) {
  console.log(response);
}
  
// 或者寫成匿名函式
sendRequest('https://api.twitch.tv/kraken/games/top?client_id=xxx', function (response) {
  console.log(response);
});
```

現在你就知道為什麼網路的操作是非同步，以及什麼是 callback function 了。

# XMLHttpRequest

方才提到 Ajax、非同步以及 callback function 的概念，但還是沒講到要怎麼發送 Request，只寫了一個假的`sendRequest`函式當作參考而已。

要發送 Request 的話，就要透過瀏覽器幫我們準備好的一個物件，叫做`XMLHttpRequest`，範例程式碼如下：

``` js
var request = new XMLHttpRequest();
request.open('GET', `https://api.twitch.tv/kraken/games/top?client_id=xxx`, true);
request.onload = function() {
  if (request.status >= 200 && request.status < 400) {
  
    // Success!
    console.log(request.responseText);
  }
};
request.send();
```

上面的`request.onload`其實就是在指定說當資料回來的時候，要用哪一個 function 去處理。

有了上面這一段程式碼之後，你終於大功告成，終於可以串接 Twitch API，從那邊拿資料下來了！真是可喜可賀，從此之後，你就跟「串接 API」這個技能過著幸福快樂的生活...

才怪。

# Same Origin Policy

正當你以為自己已經對串接 API 駕輕就熟，想說去串接別的 API 試試看好了的時候，才發現一串就出問題了：

![](/img/huli/cors1.png)

```
XMLHttpRequest cannot load 
http://odata.tn.edu.tw/ebookapi/api/getOdataJH/?level=all. 
No 'Access-Control-Allow-Origin' header is present on the 
requested resource. Origin 'null' is therefore not allowed access.
```

咦？為什麼會有這個錯誤呢？

其實是瀏覽器因為安全性的考量，有一個東西叫做[同源政策](https://developer.mozilla.org/zh-TW/docs/Web/JavaScript/Same_origin_policy_for_JavaScript)，Same-origin policy。

意思就是說如果你現在這個網站的跟你要呼叫的 API 的網站「不同源」的時候，瀏覽器一樣會幫你發 Request，但是會把 Response 給擋下來，不讓你的 JavaScript 拿到並且傳回錯誤。

什麼是不同源呢？其實你想簡單一點，只要是 Domain 不一樣就是不同源，或者是一個用`http`一個用`https`也是不同源，端口號不一樣也是不同源。

所以如果你是接別人 API 的話，大多數情形都是不同源的。

這邊我想再強調一點，「你的 Request 還是有發出去的」，而且瀏覽器也「確實有收到 Response」，重點是「瀏覽器因為同源政策，不把結果傳回給你的 JavaScript」。如果沒有瀏覽器的話其實就沒有這些問題，你愛發給誰就發給誰，不管怎樣都拿得到 Response。

好，既然剛剛說了不同源會被擋下來，那 Twitch API 不是也不同源嗎，是怎麼串接成功的？

# CORS

大家都知道其實在不同源之間互相傳輸資料是很常有的事情，像我們串接 Twitch API 就是，我們怎麼可能跟 Twitch API 在同一個 Domain 底下呢？

因此，同源政策的確是規範非同源就被擋下來，但與此同時其實又有另外一個規範，是說：「如果你想在不同 origin 之間傳輸資料的話，你應該怎麼做」，這規範就叫做 CORS。

CORS，全名為 Cross-Origin Resource Sharing，跨來源資源共享。

這套規範跟你說，如果你想開啟跨來源 HTTP 請求的話，Server 必須在 Response 的 Header 裡面加上`Access-Control-Allow-Origin`。

這個字段你應該不陌生才對，覺得陌生的可以拉回去上面看，剛剛的錯誤訊息其實就有講到這一個 Header。

當瀏覽器收到 Response 之後，會先檢查`Access-Control-Allow-Origin`裡面的內容，如果裡面有包含現在這個發起 Request 的 Origin 的話，就會允許通過，讓程式順利接收到 Response。

如果你打開 Devtool 仔細看一開始我們發給 Twitch 的 Request，你會發現 Response 的 Header 大概是長這樣：

```
Content-Type: application/json
Content-Length: 71
Connection: keep-alive
Server: nginx
Access-Control-Allow-Origin: *
Cache-Control: no-cache, no-store, must-revalidate, private
Expires: 0
Pragma: no-cache
Twitch-Trace-Id: e316ddcf2fa38a659fa95af9012c9358
X-Ctxlog-Logid: 1-5920052c-446a91950e3abed21a360bd5
Timing-Allow-Origin: https://www.twitch.tv
```

重點是這一行：`Access-Control-Allow-Origin: *`，星號就代表萬用字元，意思是任何一個 Origin 都接受。所以當瀏覽器接收到這個 Response 之後，比對目前的 Origin 符合`*`這個規則，檢驗通過，允許我們接受跨來源請求的回應。

除了這個 Header 以外，其實還有其他的可以用，例如說`Access-Control-Allow-Headers`跟`Access-Control-Allow-Methods`，就可以定義接受哪些 Request Header 以及接受哪些 Method。

總結一下，如果你想要發起跨來源 HTTP 請求並且順利收到回應的話，需要確保 Server 端有加上`Access-Control-Allow-Origin`，不然 Response 會被瀏覽器給擋下來並且顯示出錯誤訊息。

# Preflight Request

還記得 Twitch 的 API 文件嗎？裡面需要帶一個`client-id`的參數，而文件裡面寫說你可以帶在 GET 的參數上面，也可以帶在 Header 裡，我們來試試看帶在 Header 裡會怎樣吧！打開 Devtool，你會看到一個神奇的現象：

![](/img/huli/cors2.png)

咦？我明明只發了一個 Request，怎麼變兩個了？而且第一個的 Method 居然是`OPTIONS`。只是多加了一個 Header 就多了一個 Request，是為什麼呢？

其實這又跟上面講的 CORS 有關了，CORS 把 Request 分成兩種，一種是簡單請求（simple requests）。什麼是簡單請求呢？其實定義有滿長一串的，我認為有需要用到的時候再看就好，但總之如果你沒有加任何自定義的 Header，而且又是 GET 的話，絕對是簡單請求（這個夠簡單了吧）

反之呢，如果你有加一些自定義的 Header，例如說我們剛剛加的`Client-ID`，這個 Request 就絕對不是簡單請求。

（定義可參考：[MDN: 簡單請求](https://developer.mozilla.org/zh-TW/docs/Web/HTTP/Access_control_CORS#簡單請求)）

從上述分類可知，我們剛剛發起的 Request 因為有帶了 Custom header，所以不會是簡單請求，那為什麼會多一個 Request 呢？

這一個 Request 叫做 Preflight Request，中文翻作「預檢請求」，因為非簡單請求可能會帶有一些使用者資料，因此會先透過 Preflight Request 去確認後續的請求能否送出。

如果這個 Preflight Request 沒有過的話，真的 Request 也就不會發送了，這就是預檢請求的目的。

我舉一個例子，你就會知道為什麼需要這個 Preflight Request 了。

假設今天某個 Server 提供了一個 API 網址叫做：`https://example.com/data/16`，你只要對它發送 GET，就能夠拿到 id 是 16 的資料，只要對它發送 DELETE，就可以把這筆資料刪除。

如果今天沒有 Preflight Request 這個機制的話，我就可以在隨便一個 Domain 的網頁上面發送一個 DELETE 的 Request 給這個 API。剛剛我有強調說瀏覽器的 CORS 機制，還是會幫你發送 Request，但只是 Response 被瀏覽器擋住而已。

因此呢，儘管沒有 Response，但是 Server 端的確收到了這個 Request，因此就會把這筆資料給刪除。

如果有 Preflight Request 的話，在發送出去收到結果的時候，就會知道這個 API 並沒有提供 CORS，因此真的 DELETE 請求就不會送出，到這邊就結束了。

先用一個 OPTIONS 的請求去確認之後的 Request 能不能送出，這就是 Preflight Request 的目的。

# JSONP

最後來講一下 JSONP，這是跨來源請求除了 CORS 以外的另外一種方法，全名叫做：JSON with Padding。

還記得一開始提到的同源政策吧？仔細思考一下會發現，其實有些東西是不受同源政策限制的，例如說`<script>`這個 Tag，我們不是常常引用 CDN 或是 Google Analytics 之類的第三方套件嗎？網址都是其他 Domain 的，但是卻能正常載入。

JSONP 就是利用`<script>`的這個特性來達成跨來源請求的。

今天先想像你有一段 HTML 長這樣：

``` html
<script>
  var response = {
    data: 'test'
  };
</script>
<script>
  console.log(response);
</script>
```

很好懂的一段程式碼，我就不多做解釋了。那如果今天把上面那一段換成一串網址呢？

``` html
<script src="https://another-origin.com/api/games"></script>
<script>
  console.log(response);
</script>
```

如果`https://another-origin.com/api/games`這個網址返回的內容就是剛剛的：

``` js
var response = {
  data: 'test'
};
```

那我不就一樣可以拿到資料了嗎？而且這些資料還是 Server 端控制的，所以 Server 可以給我任何資料。但是這樣用全域變數其實不太好，我們可以借用剛剛的 Callback Function 的概念，改成這樣：

``` html
<script>
  receiveData({
    data: 'test'
  });
</script>
<script>
  function receiveData (response) {
    console.log(response);
  }
</script>
```

所以 JSONP 是什麼？JSONP 其實就是透過上面這種形式，利用`<script>`裡面放資料，透過指定好的 function 把資料給帶回去。你只要把第一段的`<script>`那邊想成是 Server 的回傳值，你就可以理解了。

實務上在操作 JSONP 的時候，Server 通常會提供一個`callback`的參數讓 client 端帶過去。Twitch API 有提供 JSONP 的版本，我們可以直接來看範例：

URL: `https://api.twitch.tv/kraken/games/top?client_id=xxx&callback=aaa&limit=1`

``` js
aaa({"_total":1069,"_links":{"self":"https://api.twitch.tv/kraken/games/top?limit=1","next":"https://api.twitch.tv/kraken/games/top?limit=1\u0026offset=1"},"top":[{"game":{"name":"Dota 2","popularity":63361,"_id":29595,"giantbomb_id":32887,"box":{"large":"https://static-cdn.jtvnw.net/ttv-boxart/Dota%202-272x380.jpg","medium":"https://static-cdn.jtvnw.net/ttv-boxart/Dota%202-136x190.jpg","small":"https://static-cdn.jtvnw.net/ttv-boxart/Dota%202-52x72.jpg","template":"https://static-cdn.jtvnw.net/ttv-boxart/Dota%202-{width}x{height}.jpg"},"logo":{"large":"https://static-cdn.jtvnw.net/ttv-logoart/Dota%202-240x144.jpg","medium":"https://static-cdn.jtvnw.net/ttv-logoart/Dota%202-120x72.jpg","small":"https://static-cdn.jtvnw.net/ttv-logoart/Dota%202-60x36.jpg","template":"https://static-cdn.jtvnw.net/ttv-logoart/Dota%202-{width}x{height}.jpg"},"_links":{},"localized_name":"Dota 2","locale":"zh-tw"},"viewers":65243,"channels":373}]})
```

URL: `https://api.twitch.tv/kraken/games/top?client_id=xxx&callback=receiveData&limit=1`

``` js
receiveData({"_total":1067,"_links":{"self":"https://api.twitch.tv/kraken/games/top?limit=1","next":"https://api.twitch.tv/kraken/games/top?limit=1\u0026offset=1"},"top":[{"game":{"name":"Dota 2","popularity":63361,"_id":29595,"giantbomb_id":32887,"box":{"large":"https://static-cdn.jtvnw.net/ttv-boxart/Dota%202-272x380.jpg","medium":"https://static-cdn.jtvnw.net/ttv-boxart/Dota%202-136x190.jpg","small":"https://static-cdn.jtvnw.net/ttv-boxart/Dota%202-52x72.jpg","template":"https://static-cdn.jtvnw.net/ttv-boxart/Dota%202-{width}x{height}.jpg"},"logo":{"large":"https://static-cdn.jtvnw.net/ttv-logoart/Dota%202-240x144.jpg","medium":"https://static-cdn.jtvnw.net/ttv-logoart/Dota%202-120x72.jpg","small":"https://static-cdn.jtvnw.net/ttv-logoart/Dota%202-60x36.jpg","template":"https://static-cdn.jtvnw.net/ttv-logoart/Dota%202-{width}x{height}.jpg"},"_links":{},"localized_name":"Dota 2","locale":"zh-tw"},"viewers":65622,"channels":376}]})
```

有發現了嗎？它就是透過你帶過去的`callback`這個參數當作函式名稱，把 JavaScript 物件整個傳到 Function 裡面，你就可以在 Function 裡面拿到資料。

結合起來會變這樣：

``` html
<script src="https://api.twitch.tv/kraken/games/top?client_id=xxx&callback=receiveData&limit=1"></script>
<script>
  function receiveData (response) {
    console.log(response);
  }
</script>
```

利用 JSONP，也可以存取跨來源的資料。但 JSONP 的缺點就是你要帶的那些參數永遠都只能用附加在網址上的方式（GET）帶過去，沒辦法用 POST。

如果能用 CORS 的話，還是應該優先考慮 CORS。

# 總結

今天這篇文章的內容就是從抓資料這件事情開始，一步步告訴你應該去哪裡抓？應該怎麼抓？用 API 抓，那什麼是 API？怎麼在 JavaScript 裡面呼叫 Web API？怎麼樣存取跨來源的資料？

一般來說，跟前端抓資料有關的東西我基本上都提到了，不過有個遺珠之憾是沒有提到[Fetch API](https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API)，這是比較新的標準，也是拿來抓資料用的，MDN 上面的介紹是：

> The Fetch API provides an interface for fetching resources (including across the network). It will seem familiar to anyone who has used XMLHttpRequest, but the new API provides a more powerful and flexible feature set.

有興趣的讀者們可以自己去看一下。

希望大家看完這篇之後，會更了解怎麼樣串接後端 API，以及串接的時候可能會碰到哪些困難。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好