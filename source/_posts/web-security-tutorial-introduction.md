---
layout: post
title: Web 資訊安全（Security）簡明入門教學指南
date: 2016-11-05 20:26:00
tags: Web, Security
author: kdchang
---

![Web 資訊安全（Security）簡明入門教學指南](/img/kdchang/website-security.jpg "Web 資訊安全（Security）簡明入門教學指南")

# 前言
隨著越來越多的服務和資料連上網路，Web 資訊安全已經是 Web 開發中一個重要的環節，然而許多開發者往往希望專注在應用程式的研發，而忽略了資訊安全的重要性。不過若是沒有嚴謹地考慮資訊安全的問題，等到事情發生後反而會造成更嚴重的財務和名譽上的損失。本文希望整理一些 Web 常見資訊安全（Security）的議題和學習資源和讀者一起教學相長，下次建構網路服務時可以更留心 Web 的資訊安全，甚至努力成為一個好的白帽駭客（White Hat Hacker）。

# 常見 Web 資訊安全（Security）議題
一般而言 Web 資訊安全（Security）需要符合三點安全要素：
1. 保密性：透過加密等方法確保資料的保密性
2. 完整性：要求使用者取得的資料是完整不可被竄改的
3. 可用性：保證網站服務的持續可訪問性

以下列出常見影響 Web 資訊安全（Security）的攻擊手法：

1. SQL Injection
	
	使用惡意的 SQL 語法去影響資料庫內容：
	
	```sql
	// -- 為忽略掉後面的 SQL 
	/user/profile?id=1";DROP TABLE user--

	SELECT * FROM USER WHERE id = "1"; DROP TABLE user--
	```

	使用者登入：

	```sql
	// password" AND 1=1-- 
	SELECT * FROM USER WHERE username = "Mark"; AND 1=1-- AND PASSWORD="1234"
	```

	簡易防範方式：不信任使用者輸入的資料，確保使用者輸入都要 `escape` 掉，目前許多成熟 Web 框架都有支援 ORM 服務，大部分都基本防範了 SQL Injection。

2. XSS（Cross-Site Scripting）
	XSS 亦即將惡意程式碼注入到網頁，讓看到網頁的使用者會受影響，常見的受災戶包括論壇、討論區等網路服務。事實上 XSS 概念很簡單，透過表單輸入建立一些惡意網址、惡意圖片網址或是 JavsScript 程式碼在 HTML 中注入，當使用者觀看頁面時即會觸發。

	```html
	<IMG SRC=""onerror="alert('XSS')">
	```

	更多關於 XSS 資料可以參考 [XSS Filter Evasion Cheat Sheet](https://www.owasp.org/index.php/XSS_Filter_Evasion_Cheat_Sheet)。另外也有[簡體中文版](https://jiji262.github.io/wooyun_articles/drops/XSS%20Filter%20Evasion%20Cheat%20Sheet%20%E4%B8%AD%E6%96%87%E7%89%88.html) 

	簡易防範方式：不信任使用者輸入的資料，將所有輸入內容編碼並過濾。

3. CSRF

	[CSRF 跨站請求偽造](https://zh.wikipedia.org/wiki/%E8%B7%A8%E7%AB%99%E8%AF%B7%E6%B1%82%E4%BC%AA%E9%80%A0) 又被稱為 one-click attack 或者 session riding，通常縮寫為 CSRF 或者 XSRF， 是一種挾制用戶在當前已登入的 Web 應用程式上執行非本意的操作的攻擊方法。

	舉維基百科上的例子：假如一家銀行用以執行轉帳操作的URL地址如下： 

	`http://www.examplebank.com/withdraw?account=AccoutName&amount=10000&for=PayeeName`

	那麼，一個惡意攻擊者可以在另一個網站上放置如下代碼： 

	`<img src="http://www.examplebank.com/withdraw?account=Mark&amount=10000&for=Bob">`

	若是使用者的登入資訊尚未過期的話就會損失 10000 元的金額。

	簡易防範方式：

	1. 檢查 Referer 欄位
	這是比較基本的驗證方式，通常 HTTP 標頭中有一個 Referer 欄位，其應該和 Request 位置在同一個網域下，因此可以透過驗證是否是在同一個網域來驗證是否為惡意的請求，但會有被更改偽裝的可能。

	2. 添加驗證 token
	一般現在許多的 Web Framework 都有提供在表單加入由 Server 生成的隨機驗證 CSRF 的碼，可以協助防止 CSRF 攻擊。

4. DoS
	[Dos 阻斷式攻擊（Denial of Service Attack）](https://zh.wikipedia.org/zh-tw/%E9%98%BB%E6%96%B7%E6%9C%8D%E5%8B%99%E6%94%BB%E6%93%8A)又稱為洪水攻擊，是一種網路攻擊手法，其目的在於使目標電腦的網路或系統資源耗盡，使服務暫時中斷或停止，導致真正的使用者無法使用服務。

	根據維基百科：DoS 攻擊可以具體分成兩種形式：`頻寬消耗型` 以及 `資源消耗型`，它們都是透過大量合法或偽造的請求占用大量網路以及器材資源，以達到癱瘓網路以及系統的目的。

	頻寬消耗攻擊又分洪泛攻擊或放大攻擊：洪泛攻擊的特點是利用殭屍程式傳送大量流量至受損的受害者系統，目的在於堵塞其頻寬。放大攻擊和洪泛攻擊類似，是通過惡意放大流量限制受害者系統的頻寬；其特點是利用殭屍程式通過偽造的源 IP（即攻擊目標）向某些存在漏洞的伺服器傳送請求，伺服器在處理請求後向偽造的源 IP 傳送應答，由於這些服務的特殊性導致應答包比請求包更長，因此使用少量的頻寬就能使伺服器傳送大量的 Response 到目標主機上。

	資源消耗型又分為協定分析攻擊（SYN flood，SYN 洪水）、LAND attack、CC 攻擊、殭屍網路攻擊、Application level floods（應用程式級洪水攻擊）等。

	簡易防範方式：

	1. 防火牆
		設定規則阻擋簡單攻擊

	2. 交換機
		大多交換機有限制存取控制功能

	3. 路由器
		大多路由器有限制存取控制功能

	4. 黑洞啟動
		將請求轉到空介面或是不存在的位置

5. 檔案上傳漏洞
	許多網路應用程式可以讓使用者上傳檔案到伺服器端，由於我們不知道使用者會上傳什麼類型的檔案，若不留意就會造成很大的問題。
	
	簡單防範方式：

	1. 阻止非法文件上傳 

		- 設定檔名白名單
		- 文件標頭判斷

	2. 阻止非法文件執行 

		- 存儲目錄與 Web 應用分離
		- 存儲目錄無執行權限
		- 文件重命名
		- 圖片壓縮

6. 加密安全
	有許多的網路服務有提供會員註冊的服務，當使用者使用註冊時注意不要將密碼明碼存入資料庫。若是你使用的服務會在忘記密碼時寄明碼密碼給你很有可能該服務就是使用明碼加密，此時就很容易會榮登[我的密碼沒加密](http://plainpass.com/)的網站。不過儘管將密碼加密也未必安全，像是網路上就存在一些[ 破解網站](http://www.cmd5.com/)、[彩虹表](https://zh.wikipedia.org/wiki/%E5%BD%A9%E8%99%B9%E8%A1%A8) 可以破解加密的密碼。所以通常我們會針對不同使用者使用隨機產生的 salt 字串來加鹽後加密的方式來提高密碼的強健性。

	```
	sha3(salt + gap + password)
	```

# 簡易資安入侵流程

1. 偵查（Reconnaissance）
攻擊者準備攻擊之前進行的調查，使用 Google 或是社交工程找尋目標的相關資訊，以利之後的攻擊

2. 掃描（Scanning）
掃描目標主機的弱點，取得主機作業系統、服務和運作狀況等相關資訊

3. 取得權限（Gaining Access）
利用系統弱點取得主機權限

4. 維持權限（Maintaining Access）
維持目前取得的權限，以便日後再次存取而不需繁雜的步驟

5. 清除足跡（Clearing Tracks）
清除入侵的痕跡

# 總結
以上整理一些 Web 常見資訊安全（Security）的議題和學習資源和讀者一起教學相長，成為一個好的白帽駭客（White Hat Hacker）。隨著網路科技的發展，資訊安全的議題只會越來越重要，當下次當有產品要上線到正式環境時，不妨使用 [The Security Checklist](https://github.com/FallibleInc/security-guide-for-developers/blob/master/security-checklist.md) 確認一下有哪些資安注意事項是我們沒有注意到的。

# 延伸閱讀
1. [Web Security 網站安全基礎篇（一）](http://newsletter.ascc.sinica.edu.tw/news/read_news.php?nid=1909)
2. [Web Security網站安全基礎篇（二）](http://newsletter.ascc.sinica.edu.tw/news/read_news.php?nid=1917)
3. [3個免費的 Web資訊安全自動化測試工具](https://www.qa-knowhow.com/?p=2975)
4. [HITCON 2016 投影片 - Bug Bounty 獎金獵人甘苦談 那些年我回報過的漏洞](http://blog.orange.tw/2016/07/hitcon-2016-slides-bug-bounty-hunter.html)
5. [FallibleInc/security-guide-for-developers](https://github.com/FallibleInc/security-guide-for-developers/blob/master/security-checklist.md)
6. [[資訊安全]防範Cross-Site-Scripting(XSS)](https://dotblogs.com.tw/jimmyyu/2009/08/16/10098)
7. [網站防範XSS攻擊的關鍵思考](http://www.ithome.com.tw/node/66888)

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校，目前專注在 Mobile 和 IoT 應用開發。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 