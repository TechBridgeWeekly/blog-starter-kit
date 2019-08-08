---
title: 淺談 Session 與 Cookie：一起來讀 RFC
date: 2019-08-10 10:59:32
tags:
    - session
    - cookie
    - web
author: huli
---

## 前言

這是一系列共三篇的文章，我稱之為 Session 與 Cookie 三部曲。系列文的目標是想要由淺入深來談談這個經典議題，從理解概念一直到理解實作方式。這是系列文的第二篇，三篇的完整連結如下：

1. [白話 Session 與 Cookie：從經營雜貨店開始](https://medium.com/@hulitw/session-and-cookie-15e47ed838bc)
2. [淺談 Session 與 Cookie：一起來讀 RFC](https://github.com/aszx87410/blog/issues/45)
3. [深入 Session 與 Cookie：Node.js、PHP 與 Rails 的實作
](https://github.com/aszx87410/blog/issues/46)

在上一篇裡面，我們提到了 Session 的意思：

> Session 是什麼？就是一種讓 Request 變成 stateful 的機制。以小明的例子來說，Session 就是一種讓客人之間能互相關聯起來的機制。在故事裡面我們用了紙條跟手機裡的資訊來比喻，有多種方式可以達成 Session。

其實在寫這系列的時候，「Session 最明確的定義是什麼」困擾了我一陣子，而且我到現在都還不能完全肯定自己到底是不是對的。在我心中有兩個解釋都滿合理的，第一個解釋就是上一篇跟大家講的，Session 是一種讓 Request 變成 stateful 的「機制」，而 Session 的第二種解釋（也是比較貼近英文原文的解釋），就像是「具有狀態的一段期間」，或者是「上下文」，在 Session 裡面的東西可以放在一起看。

有一種說法認為 Session 的原意的確是第二種，但是在 Web 領域中 Session 轉變成了一種「機制」，所以兩個意思都通。但我自己其實是比較傾向第二種才是唯一正確的解釋方法，從頭到尾第二種都是對的，第一種則是誤解。

舉個例子來說，如果你有用過 Google Analytics，裡面有個名詞叫做「工作階段」，英文原名就叫做 Session，而 Google 對 Session 的解釋是這樣的：

![](/img/huli/session-rfc/ga.png)

（來源：[Analytics (分析) 定義網頁工作階段的方式](https://support.google.com/analytics/answer/2731565?hl=zh-Hant)）

它把 Session 定義為「指定期間內在網站上發生的多項使用者互動」，並且說可以把 Session 當作一個容器（Container）。雖然說 Google Analytics 的 Session 跟 Web 技術上所使用的 Session 本來就不同，但我認為多少可以互相參考。而這個 Session 的定義與我前面所說的「具有狀態的一段期間」或者是「上下文」其實是雷同的。

那為什麼儘管我比較偏向這個定義，卻在上一篇裡面隻字不提，還把 Session 定義成我眼中的「誤解」？

第一個原因是搞不好兩種解釋都說得通，所以有可能兩個都是對的。第二個原因是我所認為的 Session 精確定義非常不好解釋，因為概念太抽象了，而且我認為若是提了這個解釋，只會把你對 Session 的理解越搞越亂，因此上一篇才沒有提到這個。第三個原因是我認為解釋成機制也可以，而且比較好理解，就算它真的是錯誤的，造成的影響也沒那麼大。

總之呢，我認為對完全沒有基礎的人來說，把 Session 理解成一種機制就可以了。但是對於像我這種想要追根究底的人來說，我想知道的是最正確的理解，而且必須是有憑有據的。

要怎樣才叫做有憑有據呢？去看當年談論 Cookie 與 Session 的 RFC 文件應該夠有憑有據了吧？RFC 文件可是要經歷過一系列討論與審核之後才能誕生，我想不到有哪邊的解釋能比 RFC 更具有說服力。

在這篇文章中我們會來稍微讀一下三份 RFC：

1. [RFC 2109](https://tools.ietf.org/html/rfc2109)
2. [RFC 2965](https://tools.ietf.org/html/rfc2965)
3. [RFC 6265](https://tools.ietf.org/html/rfc6265)

為什麼要讀三份呢？因為這三份都是跟 Cookie 相關的文件，2109 是最早的一份，後來出現一些問題所以被新的 2965 取代，過了十年後有了 6265，是目前最新的標準。

我認為讀東西這樣讀能夠事半功倍，因為最早期的時候東西應該會最少，理解上也比較容易，找資料也好找。例如說要讀 React 原始碼我會推薦從最早的 0.xx 版本開始讀，讀 ECMAScript 也可以從 ES3 開始，也可以順便知道演進的過程。

前情提要大概就到這邊了，本文的目標就是來讀 RFC，看看裡面是怎麼說 Cookie 與 Session 的。裡面我會對原文做一些翻譯，但畢竟翻譯是項專業，我翻的很差而且一定有錯誤，所以推薦還是要看原文，翻譯只能當作輔助。如果有哪邊錯的很離譜歡迎指出，我會十分感謝。

## RFC 2109

[RFC 2109](https://tools.ietf.org/html/rfc2109) 發佈於 1997 年 2 月，那是個還沒有 Ajax 的年代，是個 Netscape 還稱霸瀏覽器市場的年代。

這份文件的標題叫做：「HTTP State Management Mechanism」，直翻就是 HTTP 狀態管理機制。

先來看摘要的部分：

> This document specifies a way to create a stateful session with HTTP requests and responses.  It describes two new headers, Cookie and Set-Cookie, which carry state information between participating origin servers and user agents.  The method described here differs from Netscape's Cookie proposal, but it can interoperate with HTTP/1.0 user agents that use Netscape's method.  (See the HISTORICAL section.)
> 
> 這份文件規定了一種利用 HTTP request 與 response 建立有狀態的 session 的方法，並介紹了兩個新的 header：Cookie 跟 Set-Cookie，藉由這兩個 header 在 server 與 user agent（通常就是指瀏覽器）之間傳遞資訊。這邊所提到的方法與 Netscape 的 Cookie 提案不同，但可以互相兼容。

（每次翻譯翻一翻就會不想翻了...因為總覺得自己翻譯得不夠精確，翻譯真滴難）

摘要寫得很明確了，簡單來說就是引入 Cookie 與 Set-Cookie 兩個 Header 來建立 Session。會特別提到 Netscape 是因為 Cookie 這東西最早出現是 Netscape 自己實作的，只可惜我能找到的連結都死掉了，無緣看到 Netscape 的 Cookie 長什麼樣子。

再來第二個部分 TERMINOLOGY 就是規定一些專有名詞的用法，可以稍微掃過去就好，重點在第三個部分 STATE AND SESSIONS：

> This document describes a way to create stateful sessions with HTTP requests and responses.  Currently, HTTP servers respond to each client request without relating that request to previous or subsequent requests; the technique allows clients and servers that wish to exchange state information to place HTTP requests and responses within a larger context, which we term a "session".  
> 
> 這份文件規定了一種利用 HTTP request 與 response 建立有狀態的 session 的方法。目前 HTTP 伺服器獨立回應每一個 Request，不把它與其他 Request 關聯，而個方法允許想要交換狀態資訊的 Server 與 Client 把 HTTP Request 與 Response 放在一個更大的 context 底下，我們稱之為一個 session。（這段我覺得超級難翻...而且一定翻得不好）

> This context might be used to create, for example, a "shopping cart", in which user selections can be aggregated before purchase, or a magazine browsing system, in which a user's previous reading affects which offerings are presented.
> 
> 舉例來說，這個 context 可以用來建立購物車的功能，在購買前可以知道使用者選了哪些物品，或者是雜誌瀏覽系統，從以前讀過的東西推薦可能喜歡的內容。

這邊對於 Session 的定義就如同我前面所講的那樣，Session 是「具有狀態的一段期間」，或者是「上下文」，就是上面所提到的 context，在這個 context 裡面的Request 與 Response 可以放在一起看，於是他們之間就有了狀態。

> There are, of course, many different potential contexts and thus many different potential types of session.  The designers' paradigm for sessions created by the exchange of cookies has these key attributes:
> 
> 1. Each session has a beginning and an end.
> 2. Each session is relatively short-lived.
> 3. Either the user agent or the origin server may terminate a session.
> 4. The session is implicit in the exchange of state information.
> 
> 有很多種不同型態的 session，而藉由交換 cookie 所建立的 session 有幾個重點：
> 
> 1. 每個 session 都有開始與結束
> 2. 每個 session 都是相對短暫的
> 3. 瀏覽器或伺服器任何一方都可以終止這個 session
> 4. Session 蘊含了交換狀態資訊的概念在裡面

這邊就是稍微介紹了一下 Session 的特性而已。若是我們把 Session 理解為是一種「機制」，那該如何解釋上面的段落？「每個 Session 機制都是相對短暫的？」，聽起來有點怪怪的，所以這也是為什麼我會說 Session 當作機制來解有一點奇怪。

接下來第四個章節很多部分都是在講那些 Header 的規格，這邊我們跳過不看，只節選幾個我認為比較重要的段落出來：

> 4.2.1  General
> 
> The origin server initiates a session, if it so desires. (...) >
> To initiate a session, the origin server returns an extra response header to the client, Set-Cookie.  (The details follow later.)
> 
> A user agent returns a Cookie request header (see below) to the origin server if it chooses to continue a session.

> 如果 Server 需要的話，它可以初始化一個 session，而初始化的方法是回傳一個 Set-Cookie 的 Header，若是瀏覽器決定繼續這個 session 的話，可以回傳 Cookie 這個 Header

簡單來說就是你把伺服器把狀態放在 Set-Cookie 這個 Header 裡面送去瀏覽器，而瀏覽器在之後的 Request 把 Cookie 帶上去，這樣子就成立一個 Session 了，因為後續的 Request 就有了狀態。

再來可以看一下第五個章節 EXAMPLES 的部分，我們來看其中一個例子，這邊的例子比較簡單，我就直接翻中文了，想看原文可以到這裡：[5.1 Example 1](https://tools.ietf.org/html/rfc2109#section-5.1)。

### 第一步：瀏覽器 -> 伺服器

```
POST /acme/login HTTP/1.1
[form data]
```

使用者透過表單登入。

### 第二步：伺服器 -> 瀏覽器

```
HTTP/1.1 200 OK
Set-Cookie: Customer="WILE_E_COYOTE"; Version="1"; Path="/acme"
```

登入成功，伺服器發送 Set-Cookie Header 並設置資訊，儲存了使用者的身份。


### 第三步：瀏覽器 -> 伺服器

```
POST /acme/pickitem HTTP/1.1
Cookie: $Version="1"; Customer="WILE_E_COYOTE"; $Path="/acme"
[form data]
```

使用者把某個物品加入購物車。

### 第四步：伺服器 -> 瀏覽器

```
HTTP/1.1 200 OK
Set-Cookie: Part_Number="Rocket_Launcher_0001"; Version="1"; Path="/acme"
```

伺服器再設置一個 Cookie 來儲存剛剛加入購物車的東西。

### 第五步：瀏覽器 -> 伺服器

```
POST /acme/shipping HTTP/1.1
Cookie: $Version="1";  
Customer="WILE_E_COYOTE"; $Path="/acme";  
Part_Number="Rocket_Launcher_0001"; $Path="/acme"  
[form data]
```

使用者利用表單選擇商品的運送方式。

### 第六步：伺服器 -> 瀏覽器

```
HTTP/1.1 200 OK
Set-Cookie: Shipping="FedEx"; Version="1"; Path="/acme"
```

設置新的 Cookie 來儲存運送方式。

### 第七步：瀏覽器 -> 伺服器

```
POST /acme/process HTTP/1.1
Cookie: $Version="1";  
Customer="WILE_E_COYOTE"; $Path="/acme";  
Part_Number="Rocket_Launcher_0001"; $Path="/acme" 
Shipping="FedEx"; $Path="/acme" 
[form data]
```

使用者選擇結帳。

### 第八步：伺服器 -> 瀏覽器

```
HTTP/1.1 200 OK
```

根據瀏覽器帶上來的 Cookie 得知用戶資料、購買品項以及運送方式，交易完成！

上面這個範例大致說明了 Cookie 的運作方式，就是透過伺服器傳送 Set-Cookie 這個 header 來設置資訊，並且靠瀏覽器傳送 Cookie header 把之前儲存的資訊一併帶上來，這樣子就有了狀態，就開啟了一段 Session。

接著來看第六部分：IMPLEMENTATION CONSIDERATIONS，講到實作上的一些考量，這邊一樣截取片段：

> 6.1  Set-Cookie Content
> 
> The session information can obviously be clear or encoded text that describes state.  However, if it grows too large, it can become unwieldy.  Therefore, an implementor might choose for the session information to be a key to a server-side resource.  Of course, using a database creates some problems that this state management specification was meant to avoid, namely:
> 
> 1. keeping real state on the server side;
> 2. how and when to garbage-collect the database entry, in case the user agent terminates the session by, for example, exiting.
> 
> 存在 cookie 裡的 session 資訊可以是明文或是編碼過後的文字，但如果存的東西太多會變得太笨重。所以，你可以選擇只在 session 資訊裡放一個對應到伺服器資源的 key。但這種方式會造成一些這個狀態管理機制本來就想避免掉的一些問題，主要是：
> 
> 1. 把狀態放在 server
> 2. 如何以及何時把沒有用的狀態資訊清掉

其實這種方式就是我們在上一篇所提到的兩個不同的方法：Cookie-based session 以及 SessionID，前者就是存太多東西會變得笨重，後者就是需要把狀態放在 Server。

兩種方式其實各有優劣，但比較常使用的其實還是 SessionID 那種方式，也就是原文提到的：「session information to be a key to a server-side resource」。

好，其他都是有關安全性或是跟隱私有關的部分，跟我們這篇要談的議題有點差異，因此我就不特別提了。

讓我們先來整理一下上面所看到的東西。

首先，Cookie 的發明就是為了要建立 Session 而生的，因為在這之前要建立 Session 只能透過我上一篇提到的那些方式，例如說用網址列帶資訊，或者是在 form 裡面放一個 hidden 的欄位。為了簡化這些行為才有了 Cookie 的誕生。

而實際方式就是 Server 回傳 Set-Cookie 的 header，User agent 把這些資訊儲存起來之後，在後續的 Request 都加上一個 Cookie header，這就是我們上一篇中所提到的「紙條」，每次都會帶著這個紙條，就讓 Request 之間有了狀態。

至於要在 Cookie 裡放什麼狀態都行，但如果放太多的東西可以考慮把這些狀態移到 Server 去，只在 Cookie 裡放一個可以對應的 ID。這就是我們之前所說的 Session ID 與 Session Data。

## RFC 2965

[RFC 2965](https://tools.ietf.org/html/rfc2965) 誕生於 2000 年，不過他的內容跟 RFC 2109 其實相去不遠，大概有八成的內容都是一樣的。

為什麼呢？

在 RFC 2109 出來之後不久他們發現了 IE3 與 Netscape3 對於這份「新的」Cookie 標準（舊的指的是 Netscape 原本自己的那套規範）實作不同，例如說以下這一段：

```
Set-cookie: xx="1=2\&3-4";
Comment="blah";
Version=1; Max-Age=15552000; Path=/;
Expires=Sun, 27 Apr 1997 01:16:23 GMT
```

在 IE 裡面會把 Cookie 設置成這樣：`Cookie: Max-Age=15552000`，在 Netscape Navigator 裡面則是我們預期的：`Cookie: xx="1=2\&3-4".`，於是他們就要想辦法來修正這個行為。

最後就有了 RFC 2965 的出現，解決方式是引入了兩個新的 Header：Cookie2 跟 Set-Cookie2，其餘部分都與 RFC 2109 差不多。

因此 2965 我們可以跳過不看，直接來看最新的 RFC 6265。

## RFC 6265

[RFC 6265](https://tools.ietf.org/html/rfc6265) 是 2011 年出現的文件，跟上一份相隔 11 年。

而這份文件可以說是把 Cookie 規則再翻新了一遍，修改的幅度很大，在 Introduction 裡面就有說明了：

> Prior to this document, there were at least three descriptions of cookies: the so-called "Netscape cookie specification" [Netscape], RFC 2109 [RFC2109], and RFC 2965 [RFC2965].  However, none of these documents describe how the Cookie and Set-Cookie headers are actually used on the Internet (see [Kri2001] for historical context).
> 
> 在這份文件之前，至少存在著三份不同的 Cookie 規格，第一個是 Netscape 的規格，再來是 RFC 2109 以及 2965。可是沒有一份文件真的在描述現今我們如何使用 Cookie 與 Set-Cookie。

有些我們現在在用的屬性，在 RFC 2965 都是不存在的，例如說 HttpOnly。這份規範把很多東西都定義的比較明確，有興趣的讀者可以自己去看。

接著我們來看一些有趣的地方好了，第一個是 3.1 Examples，裡面提到的範例直接使用了 SessionID：

> 3.1.  Examples
> 
> Using the Set-Cookie header, a server can send the user agent a short string in an HTTP response that the user agent will return in future HTTP requests that are within the scope of the cookie.  For example, the server can send the user agent a "session identifier" named SID with the value 31d4d96e407aad42.  The user agent then returns the session identifier in subsequent requests.
> 
> 伺服器可以利用 Set-Cookie header 設置一個簡短的字串，而瀏覽器會在後續的 Request 把這個字串傳上來。舉例來說，伺服器可以傳送一個「session identifier」稱之為 SID，內容是 31d4d96e407aad42，而瀏覽器就會在之後的 Request 把這個 sessionID 傳上來。

底下還有提供更完整的範例，但有點長我就不翻了。其實我很推薦大家自己把這整份文件都看完，因為這整份文件定義的就是現在我們在使用的 Cookie 規格，你可以從規格裡面知道最正確的資訊。

例如說：

> 4.1.2.5.  The Secure Attribute
> 
> The Secure attribute limits the scope of the cookie to "secure" channels (where "secure" is defined by the user agent).  When a cookie has the Secure attribute, the user agent will include the cookie in an HTTP request only if the request is transmitted over a secure channel (typically HTTP over Transport Layer Security (TLS)[RFC2818]).
> 
> Secure 這個屬性限制了 Cookie 只能在「安全」的管道上傳輸（安全的定義由 user agent 自己定義）。當一個 Cookie 有了 Secure 這個屬性，只有當 Request 在安全的管道（通常指的是 TLS）中傳輸時才會把 cookie 放進 HTTP Request 裡面。

這邊我們就可以看到規格與實作的差異。規格只說了「什麼是安全由 user agent 自己定義」，而沒有強制規範說「一定要在 HTTPS 的時候才能傳輸」。所以一般我們所認知的「Secure 就是代表一定要 HTTPS 才會被傳送」其實指的是主流瀏覽器的實作，而不是 RFC 的規範。

所以如果想完整回答「設置 Secure 屬性代表什麼」這個問題，可以這樣回答：

> 代表這個 Cookie 只能透過 secure 的管道被傳輸，至於什麼是 secure，RFC 上寫說由瀏覽器自行定義。依據目前主流的實作，就是指只能透過 HTTPS 來傳送

再來我們來看跟我們切身相關的一個東西：

> 7.Privacy Considerations
> 
> Cookies are often criticized for letting servers track users.  For example, a number of "web analytics" companies use cookies to recognize when a user returns to a web site or visits another web site.  Although cookies are not the only mechanism servers can use to track users across HTTP requests, cookies facilitate tracking because they are persistent across user agent sessions and can be shared between hosts.
> 
> Cookie 因為可以被用來追蹤使用者而飽受批評。舉例來說，很多在做 web analytics 的公司用 cookie 來辨認用戶造訪了哪些網站。雖然 cookie 不是唯一能追蹤使用者的技術，但因為它可以在不同 host 被共享的特性，的確促進了這種追蹤的行為。
> 
> 7.1. Third-Party Cookies
> 
> Particularly worrisome are so-called "third-party" cookies.  In rendering an HTML document, a user agent often requests resources from other servers (such as advertising networks).  These third-party servers can use cookies to track the user even if the user never visits the server directly.  For example, if a user visits a site that contains content from a third party and then later visits another site that contains content from the same third party, the third party can track the user between the two sites.
> 
> 最令人擔心的就是第三方 cookie。在渲染 HTML 頁面時，瀏覽器常會發送一些 Request 去其他的 Server（例如說廣告商的伺服器），所以儘管這些使用者從來沒有直接造訪這些網站，這些網站可以利用 Cookie 來追蹤使用者。舉例來說，使用者造訪了有跟某廣告商合作的 A 網站，然後又去了跟同個廣告商有合作的 B 網站，廣告商就可以在這兩個網站之間追蹤使用者。
> 
> Some user agents restrict how third-party cookies behave.  For example, some of these user agents refuse to send the Cookie header in third-party requests.  Others refuse to process the Set-Cookie header in responses to third-party requests.  User agents vary widely in their third-party cookie policies.  This document grants user agents wide latitude to experiment with third-party cookie policies that balance the privacy and compatibility needs of their users. However, this document does not endorse any particular third-party cookie policy.
> 
> 有些瀏覽器會限制第三方 Cookie。舉例來說，有些不發送 Cookie header 給第三方，有些則是不處理第三方的 Set-Cookie header。每一個瀏覽器對於第三方 cookie 的處理方式都不太一樣，而這份文件給了瀏覽器很大的空間去實驗什麼是對使用者最好的策略，試圖在隱私與兼容性之間取得一個平衡。不過這份文件不會認可任何一個特定的第三方 cookie 處理方式。
> 
> Third-party cookie blocking policies are often ineffective at achieving their privacy goals if servers attempt to work around their restrictions to track users.  In particular, two collaborating servers can often track users without using cookies at all by injecting identifying information into dynamic URLs.
> 
> 如果 Server 用一些 workaround 追蹤使用者的話，阻擋第三方 cookie 的策略其實不是那麼有用。例如說他們可以把資訊附加在 URL 上面來追蹤用戶，而不透過 Cookie。

其實當初在 RFC 2109 就有談論過第三方 cookie 的議題，只是那時候叫做 [Unverifiable Transactions](https://tools.ietf.org/html/rfc2109#section-4.3.5)，看到的時候我有嚇了一跳，在 1997 年剛有 cookie 的時候就已經提到了第三方 cookie 的問題。

畢竟這個問題感覺在近期才比較被廣泛討論，而且在近幾年 Safari 跟 Firefox 才預設阻擋第三方 cookie。甚至連 Facebook 之後的解法 dynamic URLs 都早已出現在 RFC 6265 上面（我超討厭那串 fbcid...）。

最後我們來看一些跟安全性相關的東西，都在 8.Security Considerations 裡面：

> 8.4. Session Identifiers
> 
> Instead of storing session information directly in a cookie (where it might be exposed to or replayed by an attacker), servers commonly store a nonce (or "session identifier") in a cookie.  When the server receives an HTTP request with a nonce, the server can look up state information associated with the cookie using the nonce as a key.
> 
> Using session identifier cookies limits the damage an attacker can cause if the attacker learns the contents of a cookie because the nonce is useful only for interacting with the server (unlike non- nonce cookie content, which might itself be sensitive).  Furthermore, using a single nonce prevents an attacker from "splicing" together cookie content from two interactions with the server, which could cause the server to behave unexpectedly.
> 
> Using session identifiers is not without risk.  For example, the server SHOULD take care to avoid "session fixation" vulnerabilities. A session fixation attack proceeds in three steps.  First, the attacker transplants a session identifier from his or her user agent to the victim's user agent.  Second, the victim uses that session identifier to interact with the server, possibly imbuing the session identifier with the user's credentials or confidential information. Third, the attacker uses the session identifier to interact with server directly, possibly obtaining the user's authority or confidential information.



關於作者： 
[@huli](https://medium.com/@hulitw) 野生工程師，相信分享與交流能讓世界變得更美好