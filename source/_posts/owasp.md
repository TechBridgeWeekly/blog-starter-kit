---
title: Top issues on OWASP
date: 2017-05-07 07:12:25
tags:
	- security
author: jyt0532
---

今天來講講 web application security。基本上這篇講的是所有開發者都要知道的最基本知識。最近電腦工程領域實在太過火紅，網頁開發更是前仆後繼，但並不是每個開發者都對網路安全有 sense，如果你搜尋 "password ext:xls" 就會發現挺多不可思議的連結。

![Alt text](/img/jyt0532/awkward.gif)

但誰無年少，誰不輕狂。看完後把冷汗擦一擦，讓這篇文章介紹目前最常見的幾個安全問題。

### OWASP

講到網路安全一定要知道的 [Project OWASP: Open Web Application Security Project](https://www.owasp.org/index.php/Main_Page)。裡面介紹了各種不同的網路安全問題、每幾年的十大安全問題排名跟範例，完全免費，基本上是所有做 security 的聖經。

值得一提的是他們每幾年就會統計最常見的網路安全問題，
先來看一下 2010 的跟 2013 的( 2013 是最新的 list )。

<img src="/img/jyt0532/owasp.jpg" width="800">

基本上榜上有名的都挺固定的，劉伯溫說忠臣要比奸臣更奸，要擋住駭客的唯一方式就是比駭客厲害。今天就來瞭解一下這些到底是什麼。以下的 code sample 主要來自於 [AppsecTutorialSeries](https://www.youtube.com/channel/UC5xIEA6L0C2IG3iWgs8M2cA)。


### A1: SQL Injection

其實 Injection 包含了很多種，除了 SQL injection 之外還有 code injection、os command injection、xml injection 等等。
但最惡名昭彰的當然就是 sql injection。簡單的 code 如下

```cpp
void queryDB(string name){
	string sql = "select * from users where name = '" + name + "'";
	doQuery(sql);
}
```

很好，簡單易懂。

如果 server 拿到的 name 是 **John**，那 sql 就是 “select * from users where name = John”。

如果 server 拿到的 name 是 **Steve**，那 sql 就是 “select * from users where name = Steve”。

如果 server 拿到的 name 是 **John or 1 = 1**，那 sql 就是 “select * from users where name = John or 1 = 1”。

這下 high 了，因為 where 的 condition always true。第三個 sql 回傳 database 的所有 row 的所有 column。

![Alt text](/img/jyt0532/laugh.gif)

現在還笑得出來，那要是 server 拿到的 name 是 **John;DROP TABLE users;**。

![Alt text](/img/jyt0532/screaming.gif)

要是這真的發生了怎麼辦，別怕！這時候還有一個辦法，趕快上這個網站 [Linkedin](https://www.linkedin.com/jobs/) 找份新工作吧！你的 user 都不見了。

同樣的 case 可以 apply 到很多地方。如果你把使用者輸入給你的東西原封不動拿去 run os command line 就是 command injection。把 user 輸入給你的東西原封不動寫進 log 就會有 log injection。
很多初出茅廬的 developer 不知道或是忘記預防 injection 的問題，這也是為什麼 injection 是 OWASP 多年來的不動榜首。


### SQL Injection 預防

最基本的一件事情就是把你 web server 的 error log 關起來(不要 return 給 client)。不要讓任何訊息洩露。如果你沒關起來，hacker 大概隨便試兩次就知道你的 table name 了。

然後要怎麼防呢？當然就是要好好 validate user input。在 CS 領域有個用來嚇初學者的冠冕堂皇的名詞，叫做 parameterized query。

其實概念也很簡單，就是你 server 端先把你的 sql query 寫好。至於 username 之類的變數呢？就等 validate 之後再丟進去，好處就是 code 有 code 的 syntax 跟允許的 character，data 有 data 的 syntax 跟允許的 character，code 跟 data 分清楚你之後的 test 也會比較好寫。

至於各種語言怎麼 implement 請參考 [Query Parameterization Cheat Sheet](https://www.owasp.org/index.php/Query_Parameterization_Cheat_Sheet)


### A2: Broken Authentication and Session Management

這個 category 包含所有的盜帳號，不論你帳密是被猜到還是被竊聽到都算。

這裡要先介紹 session，session 可以抽象的想成 client 跟 server 的一次 conversation，每次的 conversation 開始的時候會先問帳密，認證了之後會生成一個 session id，也就是這次對話的 id。只要是同一個 session id 你就不需要每傳一次訊息都重新登入。

懂了之後呢，開始介紹這個類型的可能被攻擊的點。這個 category 有幾種類型：

1.**傳輸未加密**，傳輸內容被中間人竊聽。之前都有提過，解法就是用 [https](https://www.jyt0532.com/2017/03/08/https/) 或 [ssh](https://www.jyt0532.com/2017/03/09/ssh/)。

2.**session 在 user 登出之後沒有清理掉，或是 session id 不會 time out**，更慘的是 session id 直接寫在 URL 裡面。比如說：

```url
http://example.com/myprofile?sessionid=abcde
```

任何人只要拿到這個 session id，他根本不用知道你的帳密，就可以直接假裝是你，把你戶頭的錢轉走之類的。

3.**存密碼的時候沒有 hash 或是 hash 的時候沒有 add salt**。關於資料庫密碼存儲安全問題可以參考 [用什麼樣的密碼比較安全呢](https://www.jyt0532.com/2017/02/19/password-security/) 可以暴力硬破。

### A3: Cross-Site Scripting(XSS)

XSS 是 injection 的一種。也就是 Script injection。
那為什麼要從 Injection 的 category 單獨出來呢？因為剛剛 A1 講的 Injection 的攻擊目標是 server。比如 SQL injection 的目標是 database server、command injection 的目標是 webserver 等等。但 XSS 的目標是**其他使用者**。

那要怎麼 inject 呢？也是很簡單，比如說一個網站的某頁有很多 comments。每個人都可以寫 comment，而且寫的 comment 都會被其他人看到。 

那今天如果 hacker 也寫了 comment，

```
I like this post<script>/*Bad code*/</script>
```

因為 webserver 會 load 所有 comment 給所有在瀏覽這一頁的 user，所以這個 script 就會被執行。而且 user 不知道(因為 html 不會 show script tag)，那你想得到的 javascript 能做的事他都能做了，偷 cookie、session 或是覆蓋一個假的 login panel 在真的 login panel 上。你帳密輸入完就直接傳到 hacker 的 server 去，完全看不出來，因為你 browser 的網址是正確的。

inject 到 HTML element 只是其中一種可能，也可能 inject 到 html tag 的 attribute，也可以 inject 到 CSS，也可以inject 到網站要 redirect 的 URL。

### Cross-Site Scripting 預防

[XSS Prevention Cheat Sheet](http://bit.ly/1iVkGlc)洋洋灑灑寫了 7 個 rule，但其實 Rule#0 最重要。就是好好 validate 所有的 user input，剩下的都是防禦各個可能被 inject 的點的實作細節。

### A4: Insecure Direct Object Reference

如果 server 直接用 user 給的變數去 access 檔案，就可以去猜 server 存其他檔案的檔名或資料夾，舉個例子：
```url
http://example.com/showimage?image=img1
```
這是網站要給你看的 image1 的 link，那你想看 img2 就直接把後面改成 img2，如果可以看得到那就是 A4。

更慘的是如果允許輸入這種：
```url
http://example.com/showimage?image=../../password/password.txt
```
那就更精彩了。

### Insecure Direct Object Reference 預防

Access control 也就是每個檔案的存取權限設定好，或是最重要的 validate user input。

### A5: Security Misconfiguration

像是你安裝一個 database，會有 default 的帳密比如說 user: admin，密碼: admin 你沒有把它拿掉，或是 server error 的時候你把所有 error message 全部傳給client。


### Security Misconfiguration 解法

1. 認真看doc。

2. 所有檔案等等的權限都開到最低，不需要的 port 都關掉，只提供需要提供的東西。

### A6: Sensitive Data Exposure

傳輸未加密或密碼沒有用 hash 或沒有加 salt：[老問題](https://www.jyt0532.com/2017/02/19/password-security/)

### A7: Missing Function Level Access Control

網頁沒有做好 access control，比如說一些 webserver 的 default path：

/log

/phpmyadmin 

/admin等等，這些權限沒關就屬於這類。

有沒有開始覺得都大同小異了。

### A8: Cross Site Request Forgery 

這個比較有名，我覺得排在 A8 算是低估它了，因為通常這個攻擊如果成功了，傷害會比較巨大。

基本上我們跟 server 能做的互動，取決於 server 開放給我們的權限。
比如說 A 可以轉錢到別人信箱，可是不能改 B 的密碼，也不能把 C 的錢轉給自己。

可是今天要是有權限的話就不一樣了，什麼時候會有權限呢？就是你的 cookie/session 還有效的時候(比如你現在開 facebook 不用輸入密碼，因為你 cookie 還有效)，讓你在你不知情的情況下送 http request。

這就是你很常收到垃圾郵件或惡意郵件的時候，他都會要你點個 link，那絕不是只是要你點廣告衝流量而已。如果你點了，剛好你另外一個網站(比如銀行)的 cookie 沒有到期，他就可以假裝是你，送 http request。

### CSRF 解法

1. 一個 request 過來前先看一下他的 header，先看 Origin 的值跟現在這個網頁的網址或 Domain 一不一樣，沒有 Origin 值就看 Referer值。不一樣的話就很可能是CSRF。

2. 對於非 get 的重要 request，要求提供驗證碼，或是重新驗證使用者。


### A9: Using Components with Known Vulnerabilities

就是你用了不安全的第三方軟體，明明他們已經承認有問題了你還不換或不更新，就會被攻擊。

### A10: Unvalidated Redirects and Forwards

有些 request 會把使用者導到其他網站或頁面，他直接抓取網址的 url 的 parameter 來 redirect，就很好被攻擊。因為任何人都可以 fake http request 裡的 parameter。比如說如果你的網站有一個會自動 redirect 的 code：

``` java
response.sendRedirect(request.getParameter("url"));
```

那 hacker 就可以生一個超連結給你的使用者。

```url
http://example.com/example.php?url=http://malicious.example.com
```
你的使用者看到網址前面，選擇信任你。但是你卻把他導到惡意網站，身敗名裂。

### Unvalidated Redirects and Forwards 解法

就是乖乖 validate，不然就不要 redirect。

### 總結

做學問最重要的就是總結，異中求同，同中求異。

A1 A4 A7 A10: user 的 input 可能是惡意的，網址亂打、檔案亂存取等等。

A2 A6: 密碼學沒學好。

A3 A8: 偷偷拿你的 cookie/session 做壞事。

A5 A9: 乖乖看 doc 乖乖更新。

我認為這主題的投資報酬率實在是挺高的，因為重要的問題都是換湯不換藥，即使之後會出新的top10也大概就是這十項再交換順序。如果有幾項沒聽過，總覺得跟人家聊天搭不上話，手腳不好施展。這篇文章的都看懂就夠了，真的有需要寫 server 的時候你再去深入研究細節即可。

## 延伸閱讀

1. [網路安全(1) - 基礎密碼學](http://blog.techbridge.cc/2017/04/16/simple-cryptography/)
2. [用什麼樣的密碼比較安全呢](https://www.jyt0532.com/2017/02/19/password-security/)

關於作者：
[@jyt0532](https://www.jyt0532.com/) 後端工程師，喜歡學習新知挑戰新事物，最近覺得 Anti pattern 比 Design pattern 有趣。
