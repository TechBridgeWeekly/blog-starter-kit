---
title: 搶救茶壺大作戰：418 I am a teapot
date: 2019-06-15 08:00:00
tags:
    - HTTP
author: huli
---

## 前言

有許多的 HTTP Status Code 大家都耳熟能詳，例如說 404 Not Found、500 Internal Server Error 以及 200 OK 等等。

在眾多的狀態碼之中，有一個擺明就是來搞笑的：418 I'm a teapot。

但你知道嗎，它不在 HTTP 標準裡面，所以根本不是標準的 HTTP 狀態碼。你可能會想說：「我都看過 RFC 了，怎麼會不是？」。但那份 RFC 也跟 HTTP 一點關係都沒有，不過滿多人都沒注意到這點。

我一開始也沒注意到這件事，以為 418 是 HTTP 標準的其中一部分，一直到 2017 年 8 月時有人在 Node.js 的 GitHub 發了一個 Issue：[418 I'm A Teapot](https://github.com/nodejs/node/issues/14644) 我才注意到。

Issue 裡面提到希望能移除對 418 的 support，而發起 Issue 的作者在被人告知 Go 也這樣搞的時候，也跑去 Go 發了一個 Issue。

那時候這起要求移除 418 狀態碼的事件其實引發了不小的風波，而大部分人其實是反對移除這個狀態碼的。甚至還有人做了一個 [save418.com](http://save418.com/)，想要拯救 418。

前陣子花了點時間研究一下整件事情的來龍去脈，在整理的過程中也發現無論贊成或是反對，這其中的理由都很值得我們去思考，因此在此總結成一篇文章跟大家分享。

## 418 的由來

418 的由來可以追溯到 1998 年 4 月 1 日愚人節的這一份文件：[RFC2324, Hyper Text Coffee Pot Control Protocol (HTCPCP/1.0)](https://tools.ietf.org/html/rfc2324)，HTCPCP 是 Hyper Text Coffee Pot Control Protocol 的簡稱，總之這份 RFC 描述了一個叫做 HTCPCP 的協定，建立在 HTTP 之上，並且可以利用這個協定來泡咖啡。

講到 418 的部分在 Section 2.3.2：

> 2.3.2 418 I'm a teapot

>   Any attempt to brew coffee with a teapot should result in the error
   code "418 I'm a teapot". The resulting entity body MAY be short and
   stout.

大意就是如果有人想用茶壺來泡咖啡，你應該回個它一個 418 的狀態碼，我是個茶壺，你幹嘛拿我來泡咖啡？

這邊值得注意的只有一件事，那就是 418 是在 HTCPCP 這個協定裡面，並不是 HTTP。所以 418 並不是 HTTP 協定的標準狀態碼。

## 移除 418 的風波

在 2017 年 8 月 5 日，Mark Nottingham 在 Node.js 的 GitHub 發了這樣的一個 [Issue](https://github.com/nodejs/node/issues/14644)：

> Node implements the 418 I'm a Teapot status code in a few places.

> Its source is RFC2324, Hyper Text Coffee Pot Control Protocol (HTCPCP/1.0). Note the title - HTCPCP/1.0 is not HTTP/1.x.

> HTCPCP was an April 1 joke by Larry to illustrate how people were abusing HTTP in various ways. Ironically, it's not being used to abuse HTTP itself -- people are implementing parts of HTCPCP in their HTTP stacks.

> In particular, Node's support for the HTCPCP 418 I'm a Teapot status code has been used as an argument in the HTTP Working Group to preclude use of 418 in HTTP for real-world purposes.

> While we have a number of spare 4xx HTTP status codes that are unregistered now, the semantics of HTTP are something that (hopefully) are going to last for a long time, so one day we may need this code point.

> Please consider removing support for 418 from Node, since it's not a HTTP status code (even by its own definition). I know it's amusing, I know that a few people have knocked up implementations for fun, but it shouldn't pollute the core protocol; folks can extend Node easily enough if they want to play with non-standard semantics.

> Thanks,

裡面請求 Node 把 418 的支援移除，理由是 418 並不是 HTTP 標準的狀態碼，而且 4xx 的狀態碼雖然還有很多，但若是我們希望 HTTP 能盡量活得長久，我們終究有一天需要用到這個狀態碼的。

底下引起了一番討論之後，有人指出 Go 也實作了 418，因此 Mark Nottingham 就跑到了 Go 的 GitHub 去，也發了一個相似的 Issue：[net/http: remove support for status code 418 I'm a Teapot](https://github.com/golang/go/issues/21326)。

這兩個 Issue 其實都很值得一看，裡面有許多很有建設性的討論。下面我整理幾個支持與反對的論點。

### 反對移除：418 是無害的

> 418 是個無害的彩蛋，而且很有趣，離我家的 418 遠一點！

我覺得這論點滿無力的，只要證明 418 其實是有害的就好。

### 支持移除：萬一以後有人要用 418 怎麼辦？

> 你說 418 無害，不對啊，如果我們希望 HTTP 能活得久，那遲早會有 418 會需要被用到的一天，到那天他就是別的意思了。就算你把 418 保留起來，也是少了一個狀態碼可以用

這一點我覺得滿有趣的。的確，照這種說法 418 佔了一個位置，以後能用的狀態碼就少了一個。但問題是這「一個」重要嗎？可以搭配下面的反對論點一起看。

### 反對移除：418 只佔了一個空間，問題不在 418

如果 4xx 都快用完的那天真的來臨了，該檢討的是 HTTP 的設計，還是檢討狀態碼不夠用？如果真的只剩一個可以用，是不是代表還有更大的問題該解決？

之所以這點我覺得很有趣，是因為這跟我們平時在寫程式會碰到的問題滿像的。有時候你會擔心自己是不是過早最佳化（Premature Optimization）或是過度工程化（Over Engineering），做了完全不需要用到的優化。

> 假設今天有一個程式，用 1~100 這 100 個數字來表示不同狀態。隨著時間我們會需要不同的數字來表示不同狀態，所以能用的數字會愈來愈少，而我們又希望這程式能夠活得長久。在這種情況下，你贊成我們拿其中一個數字來當彩蛋嗎？

如果你反對，認為每一個數字都很重要，不該隨意拿一個數字出來當彩蛋，就代表你認為 418 是該被移除的。

但我自己對這題的看法是一個數字根本無關緊要。

理由是，若是你真的把 99 個數字用完了，就算我拿去當彩蛋的數字還你，你依然在不久後會用完所有的數字。到那個時候，你還是需要找新的解法。所以只差一個數字根本差不了多少。

### 支持移除：418 不在 HTTP 標準內

這是我覺得最有力的論點。

大家都知道 418 是個彩蛋，也知道它很有趣，可是它終究不是 HTTP 標準的一部分。今天你如果要實作一個「遵循 HTTP 標準」的程式，你就不應該把 418 放進去，因為它不在裡面。在 [IANA](https://www.iana.org/assignments/http-status-codes/http-status-codes.xhtml) 裡面 418 也是 Unassigned 的狀態。

如果你今天是市井小民，想要在自己家的伺服器或是 App 裡面實作 418，那沒有人會干擾你。但對於 Node.js、Go 這種專案來說，就應該遵守規範來開發。

這點也可以延伸到平常開發產品時會碰到的問題。如果 PM 規格寫不清楚，工程師要嘛自己通靈，要嘛就叫 PM 把模糊的地方寫清楚一點，最好是不要有任何個人的解釋空間，越清楚越好。

當今天 PM 把規格書寫得超級清楚，工程師卻自己偷偷加了一個額外的彩蛋，這是合理的嗎？這彩蛋可能無關緊要，可能只有工程師自己知道怎麼打開，但無論如何還是超出了規格之外。

在思考 418 的去留問題時，你可能只看見 418。但我認為你在碰到 418 問題時所做出的選擇，都跟你平時開發會碰到的問題有關。而且有趣的是，你有可能在 418 時選了 A，卻在類似的開發問題上選了 B，兩者是互相衝突的。

以我個人的立場來說，418 不在標準內這個理由很有力。不過以情感上來說我是不希望它被移除的，幸好還有一個反對的論點也滿有力的。

### 反對移除：418 已經被誤用太久了

在做版本更新時，有一個很重要的點是要維持向下相容（backward compatible），如果不是什麼太重要的事，盡量不要有 breaking change。

而這個論點講的是 418 作為一個「被誤認為是 HTTP 標準」的狀態碼已經十幾年了，所以每一個主流的函式庫幾乎都有支援 418（你看 Node.js 跟 Go 都有支援），若是今天把 418 的支援拔掉了，那以前使用到 418 的 Server 怎麼辦？  

這論點我也覺得滿有力的，418 已經被誤用太久，拔掉之後會產生的問題似乎比維持現狀還要多。從這點來看，它是不該被移除的。

## 418 的後續發展及現況

當初 Mark Nottingham 發表了希望移除 418 的 Issue 之後，有些人覺得他是來鬧的，吃飽太閒才會把想法動到 418 身上去。

但如果你點進他的 GitHub，可以看到他的自我介紹：

> I work on HTTP specifications and implementations.

他原本就參與了各個跟 HTTP 標準相關的組織，而且在這個領域做了不少的貢獻。

在社群掀起了反對聲浪以後，他也決定從原本的移除 418 轉變為保留 418 的立場：

> So, I poked a couple of implementations to see if they'd remove 418's "teapot" semantics, and there was a reaction (to put it mildly).

> I think we need to reserve 418 to make it clear it can't be used for the foreseeable future

（來源：[http-wg 的 mailing list: Reserving 418
](https://lists.w3.org/Archives/Public/ietf-http-wg/2017JulSep/0332.html)）

於是起草了一份文件：[Reserving the 418 HTTP Status Code](https://tools.ietf.org/id/draft-nottingham-thanks-larry-00.html)，裡面說明要把 418 的狀態設定為保留，不能被其他人註冊走：

> [RFC2324] was an April 1 RFC that lampooned the various ways HTTP was abused; one such abuse was the definition of the application-specific 418 (I’m a Teapot) status code.

> In the intervening years, this status code has been widely implemented as an “easter egg”, and therefore is effectively consumed by this use.

> This document changes 418 to the status of “Reserved” in the IANA HTTP Status Code registry to reflect that.

當初研究這整件事情到這裡時，看到這份草稿的資訊發現已經過期了（Expires: February 12, 2018），到 [IANA HTTP Status Code registry](https://www.iana.org/assignments/http-status-codes/http-status-codes.xhtml) 看，發現 418 一樣是 Unassigned。

線索就到這邊全部斷掉了，所以 418 最後到底怎麼樣了？會變成保留中嗎？

於是我寫了封信去問 Mark Nottingham 本人，他只丟給我一個連結：https://github.com/httpwg/http-core/issues/43。

從這個 Issue 可以找到這個 PR：[Reserve 418 status code](https://github.com/httpwg/http-core/pull/149/files)，裡面更動了`draft-ietf-httpbis-semantics-latest.xml`這個檔案。而在 httpwg 的網站上也可以找到現在最新的草稿：https://httpwg.org/http-core/draft-ietf-httpbis-semantics-latest.html。

在最新的草稿裡面，多了這段：

> 9.5.19. 418 (Unused)
> 
> [RFC2324] was an April 1 RFC that lampooned the various ways HTTP was abused; one such abuse was the definition of an application-specific 418 status code. In the intervening years, this status code has been widely implemented as an "Easter Egg", and therefore is effectively consumed by this use.

> Therefore, the 418 status code is reserved in the IANA HTTP Status Code registry. This indicates that the status code cannot be assigned to other applications currently. If future circumstances require its use (e.g., exhaustion of 4NN status codes), it can be re-assigned to another use.

看起來是把 418 先保留起來，但如果日後 4XX 的狀態碼真的用完，還是可以把 418 拿去做其他的用途。

而 httpwg 的網站上也能找到目前最新的 HTTP/1.1 的標準：[Hypertext Transfer Protocol (HTTP/1.1): Semantics and Content](https://httpwg.org/specs/rfc7231.html)，裡面是沒有 418 的。

因此我自己的猜測是在最新的草稿裡面已經把 418 放進去了並且設成保留，但是還沒有正式發表（背後應該還有一堆流程，這部分要去研究 HTTP Working Group 的規章才能知道），不過在日後應該可以看見草稿發布並成為正式的標準。

## 總結

這樣看下來，418 I am a teapot 依舊不會是 HTTP 的標準。畢竟應該有些人跟我想的一樣，只要把 418 I am a teapot 變成 HTTP 標準的一部分，問題就解決了，但最後沒有這樣做我猜是會碰到一些問題（至於是什麼問題我也不知道，有人知道的話麻煩提點，感謝）。

最後的結論應該是 418 這個狀態碼依然會繼續以 I am a teapot 存在於各個主流的 HTTP 實作裡面，但依舊不是 HTTP 標準的一部分。在標準裡面 418 狀態碼是被設定為 (Unused) 而且暫時被保留著，不會被其他用途給取代。

這篇的目的主要就是想記錄一下 418 狀態碼的過去以及現在，並且讓大家知道它並不是 HTTP 標準的一部分。除此之外，在研究的過程中也聯想到了許多開發上會碰到的問題，其實背後的核心概念都是差不多的。

其實在寫這篇的時候猶豫了許久，因為很怕自己有地方會寫錯（參考資料太多太豐富），不過想起了之前忘記在哪看到的一句話：「比起提問，有個更快能得到正確答案的方法。那就是講一個錯的答案，就會有人來糾正你了」。

延伸閱讀：

1. [HN 的討論](https://news.ycombinator.com/item?id=14987460)
2. [HN 的討論 - 2 ](https://news.ycombinator.com/item?id=15004907)


關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好