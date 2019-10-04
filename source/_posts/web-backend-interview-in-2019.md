---
title: 2019 Web Backend 面試總結
date: 2019-10-04 17:03:44
tags:
- Web
- Backend
- Interview
author: frankyang
---

## 前言

會想分享這篇整理，主要是因為今年是我第一次面試工作，在準備期間我曾無比焦慮，看過一篇又一篇的面經，總覺得怎麼還是這麼多不懂的，而且看了也未必在面試中回答得出來，後來我發現，原來是對於面試會考哪些類別的問題沒有概念，所以才感到焦慮。希望透過這篇後端面試總結，讓即將上路或是正在途中的人，可以有個方向，知道如何準備！

我會將後端面試分為六大類：

1. 演算法（Algorithm）
2. 程式語言（Programming Language）
3. 作業系統（Operating System）
4. 網路（Network）
5. 系統設計（System Design）
6. 特定軟體（像是 MySQL, Redis 等）

以下會針對這六大類，分別介紹並提供一些準備的資源！

## 演算法（Algorithm）

演算法考題就是大家常說的 *[LeetCode](https://leetcode.com)*，我面試的公司中，有的每一關都有類似 LeetCode 的考題，有的則會把這類考題集中在第一關。關於 LeetCode 考題，會建議*別急著刷題，先把演算法的理論與應用理解後，再開始刷題，效率會更好！*

針對理解演算法，會推薦*[極客時間 - 數據結構與算法之美](https://time.geekbang.org/column/intro/126)*，我覺得這是在「有限時間內」準備演算法的好教材！在介紹每個演算法之前，作者都會以一個實際應用的問題開頭，像是介紹 Stack 時，作者會先問「如何實現瀏覽器中上一頁、下一頁的功能？」，然後才介紹 Stack，最後再針對開頭的問題解答，這種學習方式能讓讀者對 Stack 能處理哪類型的問題更有感覺！很多時候面試中的問題並不是直接給 input 和 output，而是給一個應用情境，這時候帶著作者給的問題學習，效果會比較好！

針對刷題，我會推薦以下連結：

1. [LeetCode Learn](https://leetcode.com/explore/learn/)，這裡有針對基礎算法整理出來的考題，像是 Queue & Stack, Linked List, Hash Table 等，對於一開始認識題型與認識解題技巧有很大的幫助。
2. [TechBridge - LeetCode 刷題 Pattern](https://blog.techbridge.cc/tags/Leetcode/)，如果多刷點題，會發現不同的題型常常用到一樣的技巧，這時候認識刷題 Pattern 就很有幫助，透過刷題 Pattern 可以知道哪些類型的問題，都可以用同樣的 Pattern 處理！
3. [LeetCode Interview](https://leetcode.com/explore/interview/)，最後就是針對想要的公司準備，如果 LeetCode 沒有整理到的，可以直接搜尋「某某公司 面經」，面經中提到的題目雖然不會一直重複出，但面試前先看過以往題目的難度，心裡至少會有個底。

## 程式語言（Programming Language）

程式語言的部分，會建議先看過招聘說明，通常會提到希望面試者會哪些語言，這時候再針對那個語言搜尋「某某語言 面經」，像是 Python 面經、NodeJS 面經等。這部分通常會針對該語言的特性提問，例如 Python 的 Immutable v.s. Mutable 或是 NodeJS Event Loop 等，通常每個語言都會有個經典考題是垃圾回收機制（Garbage Collection），最後要時常關注語言的 Latest Released 有沒有加入哪些新特性，這些也很容易成為面試題目！

## 作業系統（Operating System）

作業系統的經典考題如下：

- Process v.s. Thread v.s. Coroutine
- Inter-Process Communication
- 調度策略（Scheduler）
- 死鎖（Deadlock）條件，以及如何解死鎖

作業系統的考題很大很雜，有時間的話，可以重新打基礎翻閱恐龍本（Operating System Concepts），但沒時間的話，建議可以上網看別人的整理，像是想理解 Process v.s. Thread v.s. Coroutine 相關的問題，很推薦 *[Scheduling In Go 系列文章](https://www.ardanlabs.com/blog/2018/08/scheduling-in-go-part1.html)*，除了說明 Golang 怎麼實現 Coroutine，也順便理解 Golang 怎麼把 IO-Bound 的問題轉化為 CPU-Bound。

## 網路（Network）

網路相關的經典考題如下：

- TCP 三次握手與四次揮手過程，為什麼要三次握手？為什麼要四次揮手？
- TCP v.s. UDP
- TCP 如何實現流量控制
- HTTP 有哪些狀態碼
- HTTP v.s. HTTPS
- HTTPS 加密過程
- HTTP v.s. HTTP2
- 瀏覽器打開網頁的過程

藉由網路相關考題，推薦一個面試技巧 - *如果兩個項目之間有演進關係，先說前一項技術遇到什麼問題，再說後一項技術怎麼解決*。以「HTTP v.s. HTTPS」為例，不要只是回答「HTTPS 有加密」，而是先說明 HTTP 遇到什麼問題：

- 無法驗證身份
- 訊息沒加密
- 無法驗證訊息完整性

然後再說明後一項技術 HTTPS 如何解決這些問題：

- 非對稱加密 - 驗證身份
- 對稱加密 - 將訊息加密
- 雜湊（Hash）- 驗證訊息完整性

## 系統設計（System Design）

系統設計是我覺得最沒有標準答案的題目，原因是只要一個新技術沒有取代舊技術，那兩種技術間就會有選擇權衡的問題，像是 SQL v.s. NoSQL、Process v.s. Thread，兩種技術間沒有絕對的好與壞，只有適不適合某種情境而已，而系統設計相關的問題，就是要分析各種不同組合的技術，並說明為什麼要選某一種組合。

由於大多數學校沒有系統設計相關的課程，所以我會推薦先看*[極客時間 - 從 0 開始學架構](https://time.geekbang.org/column/intro/81)*，這個專欄有點像系統設計的工具箱，介紹不同種類的問題有哪些解決方式，先要知道有哪些工具可以用，這樣看系統設計的問題，才不會覺得是東補一塊西補一塊。

實際演練系統設計相關的面試題，我會推薦 *[Grokking the System Design Interview](https://www.educative.io/courses/grokking-the-system-design-interview)*，這個課程會帶大家設計各種知名系統，並且也會說明實際遇到系統設計的面試題，要怎麼回答比較好。以下是我的一些心得：

1. 跟面試官確定系統需要哪些功能
2. 評估 DAU（Daily Active User）、流量、QPS / TPS、硬碟儲存空間、記憶體空間等
3. 設計 API
4. 設計 Data Model
5. High Level Design，大致說明系統需要哪些 Component
6. Detailed Design，針對每個 API 進行架構設計，像是微信發紅包跟搶紅包的架構就不太一樣，有時更細緻的設計像是如何為每條 Tweet 生成 ID，這兩個問題分別在我上面提到的兩個課程都有。

最後要注意的是，系統設計是個演進的過程，不管在面試中還是實際設計時都一樣，所以在面試中要不斷的跟面試官來回討論，不要一直自顧自地想著要一步到位，有什麼想法就先拋出來跟面試官討論，面試官通常會回答這個方法可能有哪些問題，然後你再根據這些問題提出解決方案。

## 特定軟體

面試官通常會依據你做過的專案來提問特定軟體的問題，像是說明某個網頁專案用到 Redis 時，面試官可能就會問「那你知道 Memcached 嗎？他們之間有什麼差異，為什麼選擇 Redis？」，然後就會出現一系列更進階的考題，像是「你知道 Redis 的 Sorted Set 怎麼實現的嗎？複雜度如何？」等等。有些招聘說明會特別提到團隊用了哪些軟體，這時候也可以針對這些軟體特別準備。以下列出特定軟體可能的面試問題：

- Database
    - MySQL v.s. PostgreSQL
    - Index 有什麼好處？Database 怎麼做 Index？為什麼不要每個欄位都做 Index？
    - BTree v.s. B+Tree
    - SQL 相關的一些考題
    - MySQL 如何實現 Lock？
- Memory Database
    - [Redis v.s. Memcached](https://aws.amazon.com/elasticache/redis-vs-memcached/)
    - Redis 的 Sorted Set 如何實現？
- Nginx
    - Nginx 的實現原理是什麼？為什麼一台 Nginx Server 就可能撐上萬的 QPS？

## 面經資源

最後介紹一下找面經的好地方～

- [LeetCode Discuss](https://leetcode.com/discuss/interview-question)，在 LeetCode 的討論板上，常常會有各大公司的面試分享。
- [一畝三分地](https://www.1point3acres.com/bbs/forum-145-1.html)，這個網站主要是針對各個公司的面經，雖然是以北美的公司為主，但如果對於進世界級的大公司有興趣，在上面可以找到很多資源。
- [掘金](https://juejin.im)，這個網站則是以某個技術的面經為主，試著搜尋「Python 面经」，因為是對岸網站，所以搜尋上用簡體字，會比較容易找到想要的資源。

## 總結

面試是一個互相選擇的過程，別總覺得是公司在挑你，其實你也在挑公司！如果面試官問你有沒有什麼問題？別說沒有，試著多問點問題，瞭解公司或團隊的狀況，畢竟進去一間公司後，發覺公司不好要離開，可能都要耗上半年的時間，所以要慎選公司！*[Tech Interview Handbook](https://yangshun.github.io/tech-interview-handbook/questions-to-ask)* 有一些面試中可以反問面試官的問題可以參考。多點自信，其實你也在面試面試官！祝大家面試都順利！

關於作者：
[@frankyang](https://www.linkedin.com/in/poan-yang/) 後端工程師，熱愛閱讀及嘗試新科技
