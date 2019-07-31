---
title: 讓我們來談談 CSRF 
date: 2017-02-25 01:19:29
tags:
  - csrf
  - security
author: huli
---

Update:
經過朋友指出文章中缺漏的地方，於 2/26 早上新增一段講 SameSite Cookie 的段落。感謝 shik 的提點。

## 前言

最近剛好碰到一些 CSRF 的案例，趁著這次機會好好研究了一下。深入研究之後才發現這個攻擊其實滿可怕的，因為很容易忽略它。但幸好現在有些 Framework 都有內建防禦 CSRF 的功能，可以很簡單的開啟。

但儘管如此，我認為還是有必要瞭解一下 CSRF 到底在幹嘛，是透過怎樣的手段攻擊，以及該如何防禦。就讓我們先來簡單的介紹一下它吧！

CSRF 是一種 Web 上的攻擊手法，全稱是 Cross Site Request Forgery，跨站請求偽造。不要跟 XSS 搞混了，他們兩種是不同的東西，那到底什麼是 CSRF 呢？先從我自身的一個案例談起好了。

## 偷懶的刪除功能
以前我有做個一個簡單的後台頁面，就想成是一個部落格吧！可以發表、刪除以及編輯文章，介面大概長得像這樣：

<img src="/img/huli/csrf1.png" alt="delete" />

可以看到刪除的那個按鈕，點下去之後就可以把一篇文章刪掉。當初因為偷懶，想說如果我把這個功能做成 GET，我就可以直接用一個連結完成刪除這件事，在前端幾乎不用寫到任何程式碼：

``` html
<a href='/delete?id=3'>刪除</a>
```

很方便對吧？然後我在網頁後端那邊做一下驗證，驗證 request 有沒有帶 session id 上來，也驗證這篇文章是不是這個 id 的作者寫的，都符合的話才刪除文章。

嗯，聽起來該做的都做了啊，我都已經做到：「只有作者本人可以刪除自己的文章」了，應該很安全了，難道還有哪裡漏掉了嗎？

沒錯，的確是「只有作者本人可以刪除自己的文章」，但如果他不是自己「主動刪除」，而是在不知情的情況下刪除呢？你可能會覺得我在講什麼東西，怎麼會有這種事情發生，不是作者主動刪的還能怎麼刪？

好，我就來讓你看看還能怎麼刪！

今天假設小黑是一個邪惡的壞蛋，想要讓小明在不知情的情況下就把自己的文章刪掉，該怎麼做呢？

他知道小明很喜歡心理測驗，於是就做了一個心理測驗網站，並且發給小明。但這個心理測驗網站跟其他網站不同的點在於，「開始測驗」的按鈕長得像這樣：

``` html
<a href='https://small-min.blog.com/delete?id=3'>開始測驗</a>
```

小明收到網頁之後很開心，就點擊「開始測驗」。點擊之後瀏覽器就會發送一個 GET 請求給`https://small-min.blog.com/delete?id=3`，並且因為瀏覽器的運行機制，一併把 `small-min.blog.com` 的 cookie 都一起帶上去。

Server 端收到之後檢查了一下 session，發現是小明，而且這篇文章也真的是小明發的，於是就把這篇文章給刪除了。

這就是 CSRF，你現在明明在心理測驗網站，假設是 `https://test.com` 好了，但是卻在不知情的狀況下刪除了 `https://small-min.blog.com` 的文章，你說這可不可怕？超可怕！

這也是為什麼 CSRF 又稱作 one-click attack 的緣故，你只要點一下就中招了。

你可能會說：「可是這樣小明不就知道了嗎，不就連過去部落格了？不符合『不知情的狀況』啊！」

好，那如果我們改成這樣呢：

``` html
<img src='https://small-min.blog.com/delete?id=3' width='0' height='0' />
<a href='/test'>開始測驗</a>
```

在開啟頁面的同時，一樣發送一個刪除的 request 出去，但這次小明是真的完全不知道這件事情。這樣就符合了吧！

CSRF 就是在不同的 domain 底下卻能夠偽造出「使用者本人發出的 request」。要達成這件事也很簡單，因為瀏覽器的機制，你只要發送 request 給某個網域，就會把關聯的 cookie 一起帶上去。如果使用者是登入狀態，那這個 request 就理所當然包含了他的資訊（例如說 session id），這 request 看起來就像是使用者本人發出的。

## 那我把刪除改成 POST 不就好了嗎？

沒錯，聰明！我們不要那麼懶，好好把刪除的功能做成 POST，這樣不就無法透過 `<a>` 或是 `<img>` 來攻擊了嗎？除非有哪個 HTML 元素可以發送 POST request！

有，正好有一個，就叫做 form。

``` html
<form action="https://small-min.blog.com/delete" method="POST">
  <input type="hidden" name="id" value="3"/>
  <input type="submit" value="開始測驗"/>
</form>
```

小明點下去以後，照樣中招，一樣刪除了文章。你可能又會疑惑說，但是這樣小明不就知道了嗎？我跟你一樣很疑惑，於是我 Google 到了這篇：[Example of silently submitting a POST FORM (CSRF)](http://stackoverflow.com/questions/17940811/example-of-silently-submitting-a-post-form-csrf)

這篇提供的範例如下，網頁的世界真是博大精深：

``` html
<iframe style="display:none" name="csrf-frame"></iframe>
<form method='POST' action='https://small-min.blog.com/delete' target="csrf-frame" id="csrf-form">
  <input type='hidden' name='id' value='3'>
  <input type='submit' value='submit'>
</form>
<script>document.getElementById("csrf-form").submit()</script>
```

開一個看不見的 iframe，讓 form submit 之後的結果出現在 iframe 裡面，而且這個 form 還可以自動 submit，完全不需要經過小明的任何操作。

到了這步，你就知道改成 POST 是沒用的。

## 那我後端改成只接收 json 呢？

聰明的你靈機一動：「既然在前端只有 form 可以送出 POST 的話，那我的 api 改成用 json 收資料不就可以了嗎？這樣總不能用 form 了吧！」

[spring 的 document](https://docs.spring.io/spring-security/site/docs/current/reference/html/csrf.html)告訴你：這還是沒用的！

``` html
<form action="https://small-min.blog.com/delete" method="post" enctype="text/plain">
<input name='{"id":3, "ignore_me":"' value='test"}' type='hidden'>
<input type="submit"
  value="delete!"/>
</form>
```

這樣子會產生如下的 request body：

``` js
{ "id": 3,
"ignore_me": "=test"
}
```

但這邊值得注意的一點是，`form`能夠帶的 content type 只有三種：`application/x-www-form-urlencoded`, `multipart/form-data` 跟 `text/plain`。在上面的攻擊中我們用的是最後一種，`text/plain`，如果你在你的後端 Server 有檢查這個 content type 的話，是可以避免掉上面這個攻擊的。

只是，上面這幾個攻擊我們都還沒講到一種情況：如果你的 api 接受 cross origin 的 request 呢？

意思就是，如果你的 api 的 `Access-Control-Allow-Origin` 設成 `*` 的話，代表任何 domain 都可以發送 ajax 到你的 api server，這樣無論你是改成 json，或甚至把 method 改成 PUT, DELETE 都沒有用。

我們舉的例子是刪除文章，這你可能覺得沒什麼，那如果是銀行轉帳呢？攻擊者只要在自己的網頁上寫下轉帳給自己帳號的 code，再把這個網頁散佈出去就好，就可以收到一大堆錢。

講了這麼多，來講該怎麼防禦吧！先從最簡單的「使用者」開始講。

## 使用者的防禦

CSRF 攻擊之所以能成立，是因為使用者在被攻擊的網頁是處於已經登入的狀態，所以才能做出一些行為。雖然說這些攻擊應該由網頁那邊負責處理，但如果你真的很怕，怕網頁會處理不好的話，你可以在每次使用完網站就登出，就可以避免掉 CSRF。

或者呢，關閉執行 js 或把上面這些 pattern 的程式碼過濾掉不要執行，也是一個方法（但應該很難判定哪些是 CSRF 攻擊的程式碼）。

所以使用者能做的其實有限，真的該做事的是 Server 那邊！

## Server 的防禦

CSRF 之所以可怕是因為 CS 兩個字：Cross Site，你可以在任何一個網址底下發動攻擊。CSRF 的防禦就可以從這個方向思考，簡單來說就是：「我要怎麼擋掉從別的 domain 來的 request」

你仔細想想，CSRF 的 reuqest 跟使用者本人發出的 request 有什麼區別？區別在於 domain 的不同，前者是從任意一個 domain 發出的，後者是從同一個 domain 發出的（這邊假設你的 api 跟你的前端網站在同一個 domain）

### 檢查 Referer

request 的 header 裡面會帶一個欄位叫做 referer，代表這個 request 是從哪個地方過來的，可以檢查這個欄位看是不是合法的 domain，不是的話直接 reject 掉即可。

但這個方法要注意的地方有三個，第一個是有些瀏覽器可能不會帶 referer，第二個是有些使用者可能會關閉自動帶 referer 的這個功能，這時候你的 server 就會 reject 掉由真的使用者發出的 request。

第三個是你判定是不是合法 domain 的程式碼必須要保證沒有 bug，例如：

``` js
const referer = request.headers.referer;
if (referer.indexOf('small-min.blog.com') > -1) {
  // pass
}
```

你看出上面這段的問題了嗎？如果攻擊者的網頁是`small-min.blog.com.attack.com`的話，你的檢查就破功了。

所以，檢查 referer 並不是一個很完善的解法

### 加上圖形驗證碼、簡訊驗證碼等等

就跟網路銀行轉帳的時候一樣，都會要你收簡訊驗證碼，多了這一道檢查就可以確保不會被 CSRF 攻擊。

圖形驗證碼也是，攻擊者並不知道圖形驗證碼的答案是什麼，所以就不可能攻擊了。

這是一個很完善的解決方法，但如果使用者每次刪除 blog 都要打一次圖形驗證碼，他們應該會煩死吧！

### 加上 CSRF token

要防止 CSRF 攻擊，我們其實只要確保有些資訊「只有使用者知道」即可。那該怎麼做呢？

我們在 form 裡面加上一個 hidden 的欄位，叫做`csrftoken`，這裡面填的值由 server 隨機產生，並且存在 server 的 session 中。

``` html
<form action="https://small-min.blog.com/delete" method="POST">
  <input type="hidden" name="id" value="3"/>
  <input type="hidden" name="csrftoken" value="fj1iro2jro12ijoi1"/>
  <input type="submit" value="刪除文章"/>
</form>
```

按下 submit 之後，server 比對表單中的`csrftoken`與自己 session 裡面存的是不是一樣的，是的話就代表這的確是由使用者本人發出的 request。這個 csrftoken 由 server 產生，並且每一段不同的 session 就應該要更換一次。

那這個為什麼可以防禦呢？因為攻擊者並不知道 csrftoken 的值是什麼，也猜不出來，所以自然就無法進行攻擊了。

可是有另外一種狀況，假設你的 server 支持 cross origin 的 request，會發生什麼事呢？攻擊者就可以在他的頁面發起一個 request，順利拿到這個 csrf token 並且進行攻擊。不過前提是你的 server 接受這個 domain 的 request。

接著讓我們來看看另外一種解法

### Double Submit Cookie

上一種解法需要 server 的 state，亦即 csrf token 必須被保存在 server 當中，才能驗證正確性。而現在這個解法的好處就是完全不需要 server 儲存東西。

這個解法的前半段與剛剛的相似，由 server 產生一組隨機的 token 並且加在 form 上面。但不同的點在於，除了不用把這個值寫在 session 以外，同時也讓 client side 設定一個名叫 csrftoken 的 cookie，值也是同一組 token。

``` html
Set-Cookie: csrftoken=fj1iro2jro12ijoi1

<form action="https://small-min.blog.com/delete" method="POST">
  <input type="hidden" name="id" value="3"/>
  <input type="hidden" name="csrftoken" value="fj1iro2jro12ijoi1"/>
  <input type="submit" value="刪除文章"/>
</form>
```

你可以仔細思考一下 CSRF 攻擊的 request 與使用者本人發出的 request 有什麼不一樣？不一樣的點就在於，前者來自不同的 domain，後者來自相同的 domain。所以我們只要有辦法區分出這個 request 是不是從同樣的 domain 來，我們就勝利了。

而 Double Submit Cookie 這個解法正是從這個想法出發。

當使用者按下 submit 的時候，server 比對 cookie 內的 csrftoken 與 form 裡面的 csrftoken，檢查是否有值並且相等，就知道是不是使用者發的了。

為什麼呢？假設現在攻擊者想要攻擊，他可以隨便在 form 裡面寫一個 csrf token，這當然沒問題，可是因為瀏覽器的限制，他並不能在他的 domain 設定 `small-min.blog.com` 的 cookie 啊！所以他發上來的 request 的 cookie 裡面就沒有 csrftoken，就會被擋下來。

當然，這個方法看似好用，但也是有缺點的，可以參考：[Double Submit Cookies vulnerabilities](http://security.stackexchange.com/questions/59470/double-submit-cookies-vulnerabilities)，攻擊者如果掌握了你底下任何一個 subdomain，就可以幫你來寫 cookie，並且順利攻擊了。

### client side 的 Double Submit Cookie

會特別提到 client side，是因為我之前所碰到的專案是 Single Page Application，上網搜尋一下就會發現有人在問：「SPA 該如何拿到 CSRF token？」，難道要 server 再提供一個 api 嗎？這樣好像有點怪怪的。

但是呢，我認為我們可以利用 Double Submit Cookie 的精神來解決這個問題。而解決這問題的關鍵就在於：由 client side 來生 csrf token。就不用跟 server api 有任何的互動。

其他的流程都跟之前一樣，生成之後放到 form 裡面以及寫到 cookie。或者說如果你是 SPA 的話，也可以把這資訊直接放到 request header，你就不用在每一個表單都做這件事情，只要統一加一個地方就好。

事實上，我自己常用的 library [axios](https://github.com/mzabriskie/axios) 就有提供這樣的功能，你可以設置 header 名稱跟 cookie 名稱，設定好以後你每一個 request，它都會自動幫你把 header 填上 cookie 裡面的值。

```
 // `xsrfCookieName` is the name of the cookie to use as a value for xsrf token
xsrfCookieName: 'XSRF-TOKEN', // default

// `xsrfHeaderName` is the name of the http header that carries the xsrf token value
xsrfHeaderName: 'X-XSRF-TOKEN', // default
```

那為什麼由 client 來生這個 token 也可以呢？因為這個 token 本身的目的其實不包含任何資訊，只是為了「不讓攻擊者」猜出而已，所以由 client 還是由 server 來生成都是一樣的，只要確保不被猜出來即可。Double Submit Cookie 靠的核心概念是：「攻擊者的沒辦法讀寫目標網站的 cookie，所以 request 的 csrf token 會跟 cookie 內的不一樣」

### browser 本身的防禦

我們剛剛提到了使用者自己可以做的事、網頁前後端可以做的事情，那瀏覽器呢？之所以能成立 CSRF，是因為瀏覽器的機制所導致的，有沒有可能從瀏覽器方面下手，來解決這個問題呢？

有！而且已經有了。而且啟用的方法非常非常簡單。

Google 在 Chrome 51 版的時候正式加入了這個功能：[SameSite cookie](https://www.chromestatus.com/feature/4672634709082112)，對詳細運行原理有興趣的可參考：[draft-west-first-party-cookies-07](https://tools.ietf.org/html/draft-west-first-party-cookies-07)。

先引一下 Google 的說明：

> Same-site cookies (ne "First-Party-Only" (ne "First-Party")) allow servers to mitigate the risk of CSRF and information leakage attacks by asserting that a particular cookie should only be sent with requests initiated from the same registrable domain.

啟用這個功能有多簡單？超級無敵簡單。

你原本設置 Cookie 的 header 長這樣：

```
Set-Cookie: session_id=ewfewjf23o1;
```

你只要在後面多加一個 `SameSite` 就好：

```
Set-Cookie: session_id=ewfewjf23o1; SameSite
```

但其實 `SameSite` 有兩種模式，`Lax`跟`Strict`，默認是後者，你也可以自己指定模式：

```
Set-Cookie: session_id=ewfewjf23o1; SameSite=Strict
Set-Cookie: foo=bar; SameSite=Lax
```

我們先來談談默認的 `Strict`模式，當你加上 `SameSite` 這個關鍵字之後，就代表說「我這個 cookie 只允許 same site 使用，不應該在任何的 cross site request 被加上去」。

意思就是你加上去之後，我們上面所講的`<a href="">`, `<form>`, `new XMLHttpRequest`，只要是瀏覽器驗證不是在同一個 site 底下發出的 request，全部都不會帶上這個 cookie。

可是這樣其實會有個問題，連`<a href="..."`都不會帶上 cookie 的話，當我從 Google 搜尋結果或者是朋友貼給我的連結點進某個網站的時候，因為不會帶 cookie 的關係，所以那個網站就會變成是登出狀態。這樣子的使用者體驗非常不好。

有兩種解法，第一種是跟 Amazon 一樣，準備兩組不同的 cookie，第一組是讓你維持登入狀態，第二組則是做一些敏感操作的時候會需要用到的（例如說購買、設定帳戶等等）。第一組不設定 `SameSite`，所以無論你從哪邊來，都會是登入狀態。但攻擊者就算有第一組 cookie 也不能幹嘛，因為不能做任何操作。第二組因為設定了 `SameSite` 的緣故，所以完全避免掉 CSRF。

但這樣子還是有點小麻煩，所以你可以考慮第二種，就是調整為 `SameSite` 的另一種模式：`Lax`。

Lax 模式放寬了一些限制，例如說`<a>`, `<link rel="prerender">`, `<form method="GET">` 這些都還是會帶上 cookie。但是 POST 方法 的 form，或是只要是 POST, PUT, DELETE 這些方法，就不會帶上 cookie。

所以一方面你可以保有彈性，讓使用者從其他網站連進你的網站時還能夠維持登入狀態，一方面也可以防止掉 CSRF 攻擊。但 `Lax` 模式之下就沒辦法擋掉 GET 形式的 CSRF，這點要特別注意一下。

講到這種比較新的東西，相信大家一定都很想知道瀏覽器的支援度如何，[caniuse](http://caniuse.com/#search=samesite) 告訴我們說：目前只有 Chrome 支援這個新的特性（畢竟是 Google 自己推的方案，自己當然要支持一下）。

雖然瀏覽器的支援度不太高，但日後其他瀏覽器可能也會跟進實做這個方案，不妨在現在就把 `SameSite` 加上去，以後就不用再為 CSRF 煩惱了。

我其實只是大略的介紹一下，[draft-west-first-party-cookies-07](https://tools.ietf.org/html/draft-west-first-party-cookies-07) 裡面講到很多細節，例如說到底怎樣算是 cross site? 一定要在同一個 domain 嗎？那 sub domain 行不行？

好奇的可以自己研究一下，或者是這篇：[SameSite Cookie，防止 CSRF 攻击](http://www.cnblogs.com/ziyunfei/p/5637945.html)也有提到。

SameSite 相關的參考資料：
1. [Preventing CSRF with the same-site cookie attribute](https://www.sjoerdlangkemper.nl/2016/04/14/preventing-csrf-with-samesite-cookie-attribute/)
2. [再见，CSRF：讲解set-cookie中的SameSite属性](http://bobao.360.cn/learning/detail/2844.html)
3. [SameSite Cookie，防止 CSRF 攻击](http://www.cnblogs.com/ziyunfei/p/5637945.html)
4. [SameSite——防御 CSRF & XSSI 新机制](https://rlilyyy.github.io/2016/07/10/SameSite-Cookie%E2%80%94%E2%80%94%E9%98%B2%E5%BE%A1-CSRF-XSSI/)
5. [Cross-Site Request Forgery is dead!](https://scotthelme.co.uk/csrf-is-dead/)

## 總結

這篇主要介紹了 CSRF 的攻擊原理以及兩種防禦方法，針對比較常見的場景做介紹。一般在做網頁開發的時候，比起 XSS，CSRF 是一個比較常被忽略的重點。在網頁上有任何比較重要的操作時，都要特別留意是否有被 CSRF 的風險。

這次找了很多參考資料，但發現跟 CSRF 有關的文章其實都大同小異，想知道更細節的地方需要花很多的心力去找，但幸好 Stackoverflow 上面也有不少資料可以參考。因為我在資訊安全這塊沒有涉獵太多，如果文章有哪部分講錯的話，還麻煩各位在留言不吝指出。

也感謝我朋友 shik 的指點，告訴我有 SameSite 這麼一個東西，讓我補上最後一段。

希望這篇文章能讓大家對 CSRF 有更全面的認識。

## 參考資料
1. [Cross-Site Request Forgery (CSRF)][1]
2. [Cross-Site Request Forgery (CSRF) Prevention Cheat Sheet][2]
3. [一次较为深刻的CSRF认识](http://m.2cto.com/article/201505/400902.html)
4. [[技術分享] Cross-site Request Forgery (Part 2)](http://cyrilwang.pixnet.net/blog/post/31813672)
5. [Spring Security Reference](http://docs.spring.io/spring-security/site/docs/3.2.5.RELEASE/reference/htmlsingle/#csrf)
6. [CSRF 攻击的应对之道](https://www.ibm.com/developerworks/cn/web/1102_niugang_csrf/)

[1]: https://www.owasp.org/index.php/Cross-Site_Request_Forgery_(CSRF)#Prevention_measures_that_do_NOT_work
[2]: https://www.owasp.org/index.php/Cross-Site_Request_Forgery_(CSRF)_Prevention_Cheat_Sheet

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好