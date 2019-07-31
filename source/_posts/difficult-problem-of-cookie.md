---
title: 我遇過的最難的 Cookie 問題
date: 2017-03-24 21:52:24
tags:
  - cookie
author: huli
---

## 前言

幾個禮拜前我在工作上碰到了一些跟 Cookie 有關的問題，在這之前，我原本想說：Cookie 不就那樣嘛，就算有些屬性不太熟悉，上網找一下資料就好了，哪有什麼跟 Cookie 有關的難題？

然而事實證明我錯了。我還真的碰到了一個讓我解超久的 Cookie 問題。

相信看到這邊，很多人應該躍躍欲試了，那我就先來考一下大家：

> 什麼情形下，Cookie 會寫不進去？

像是語法錯誤那種顯而易見的就不用說了，除此之外你可能會答說：寫完全不同 domain 的 Cookie。例如說你的網頁在 `http://a.com` 卻硬要寫 `http://b.com` 的 Cookie，這種情形當然寫不進去。

或者，你可能會回答：不在 https 卻想加上 `Secure` flag 的 Cookie。  
沒錯，像是這種情形也會寫不進去。

除了這些，你還能想到什麼嗎？

如果想不太到，那就聽我娓娓道來吧！

## 悲劇的開始

在一個月前我寫了一篇跟 CSRF 有關的文章（[讓我們來談談 CSRF](http://blog.techbridge.cc/2017/02/25/csrf-introduction/)），正是因為工作上需要實作 CSRF 的防禦，所以趁機研究了一下。簡單來說，就是要在 Cookie 設置一個 `csrftoken`。

可是那天我卻發現，我怎麼寫都寫不進去。

我的測試網站的網址是：`http://test.huli.com`，拿來寫 Cookie 的 script 是：

``` js
document.cookie = "csrftoken=11111111; expires=Wed, 29 Mar 2020 10:03:33 GMT; domain=.huli.com; path=/"
```

我就只是想對`.huli.com`寫一個名稱是`csrftoken`的 Cookie。而我碰到的問題，就是怎麼寫都寫不進去。

這段語法完全沒有問題，我檢查過好幾遍了，但就是不知道為什麼寫不進去。我們開頭講的那幾種 case 這邊都完全沒碰到。這只是一個簡單的 http 網站，而且是寫自己 domain 的 Cookie，怎麼會寫不進去？

剛開始碰到這情形，我還想說會不會是我電腦的靈異現象，在其他人的電腦上就好了，就暫時沒有管它，直到有一天 PM 跟我說：「咦，這個頁面怎麼壞了？」，我仔細檢查後才發現是因為他也寫不進去這個 Cookie，導致 server 沒有收到 `csrftoken` 而驗證失敗。

好了，看來現在已經確認不是我電腦上的問題了，而是大家都會這樣。可是，卻有其他人是正常的。其他人都可以，但就只有我跟 PM 兩個人不行。

幸好見過小風小浪的我知道，每次碰到這種詭異的問題，先開無痕模式再說，至少可以知道你的瀏覽器不會被其他因素給干擾。打開無痕模式之後發現，可以了，可以設定 Cookie 了。在一般情況下不行設定，但是開無痕瀏覽模式卻可以。

這就真的很奇怪了，到底為什麼不行呢？而且若是我把 Cookie 換了一個名字，叫做`csrftoken2`，就可以寫入了！就唯獨`csrftoken`這個名稱不行，可是 Cookie 總不可能有保留字這種東西吧！就算真的有，`csrftoken`也絕對不會是保留字。

這一切都太詭異了，到底`csrftoken`這個名字有什麼問題？到底為什麼寫不進去？

於是我就去拜了 Google 大神，用`cookie 不能寫`、`cookie can not set`、`unable set cookie`等等的關鍵字去搜尋，卻都一無所獲，找到的答案都跟我的情況完全不一樣。

我用 Chrome devtool 看了，明明`http://test.huli.com`就沒有任何的 Cookie，怎麼會寫不進去呢？

在經歷過一陣亂找資料之後，我還稍微去翻了 cookie 的 rfc：[HTTP State Management Mechanism](https://tools.ietf.org/html/rfc6265)，但還是沒有找到相關資料。

最後不知道哪來的靈感，我就去 Chrome 的設定那邊檢視所有 `huli.com` 的 Cookie，並且一個一個看過之後刪掉。刪完之後，就可以正常寫入 Cookie 了。

仔細想想其實還滿合理的，畢竟無痕模式可以，就代表是以前做的一些事情會影響到寫 Cookie 這件事，再經由刪除 Cookie 就可以確認問題一定是出在其他有關的 Domain 身上，推測是其他 Domain 做了一些事情，才會造成 `http://test.huli.com` 沒辦法寫入 Cookie。

後來我回想起剛剛刪掉的那幾個 Cookie，發現存在一個也叫做`csrftoken`的同名 cookie。

## 撥雲見日

難得讓我找到了一點線索，當然要跟著這條線索繼續查下去。

回想了一下，發現是另外一個負責後台管理的網站叫做：`https://admin.huli.com`寫的，因為是用 [django](https://docs.djangoproject.com/en/1.10/ref/settings/#std:setting-CSRF_COOKIE_NAME)的關係，所以開啟 CSRF 防護之後預設的 Cookie 名稱就是`csrftoken`。

仔細再用 Chrome devtool 看了一下，這個 Cookie 設置了`Secure`，Domain是 `.admin.huli.com`。看起來也沒什麼異狀。

然而，在拜訪這個網站之後，我再試著去 `http://test.huli.com`，發現又沒辦法寫入 Cookie 了，甚至原本的 Cookie 也離奇地消失了。

太棒了！看來我離真相越來越近了！

我把這個`.admin.huli.com`的同名 Cookie 刪掉之後，去拜訪我自己的`http://test.huli.com`，發現一切都正常。Cookie 可以正常寫入。

看來答案很明顯了，那就是：

> 只要`.admin.huli.com`的那個同名 Cookie 存在，`http://test.huli.com`就沒辦法對`.huli.com`寫入同名的 Cookie。

解法其實到這邊就很明顯了，第一個是改一個 Cookie 名稱，第二個是改一個 Domain。

有關於第二個解法，還記得我們在 `http://test.huli.com` 是寫入 `.huli.com` 這個 Domain 的 Cookie 嗎？只要改成寫入 `.test.huli.com` 這個 Domain，一樣可以正常運作。

所以若是講得更詳細一點，這個寫不進去 Cookie 的問題就發生在：

>當有一個 Domain 為`.admin.huli.com`並設置成`Secure`的 Cookie 已經存在的時候，`http://test.huli.com`就沒辦法對`.huli.com`寫入同名的 Cookie。

在大概確認問題以後，我就開始調整各個變因，看能不能查出到底是哪一個環節出了問題，最後我發現兩個重點：

1. 其實只有 Chrome 不能寫，Safari, Firefox 都可以
2. `Secure` 這個 flag 沒有設置的話，就可以寫

## 深入追查

既然有了只有 Chrome 會發生這種情形的這個有力線索，就可以循著這條線繼續追查下去，那怎麼追查呢？

沒錯，就是最簡單直接的方法：去找 Chromium 的原始碼！

以前看過很多文章都是查問題查一查最後查到 Source code 去，終於輪到我也有這一天了。可是 Chromium 的原始碼這麼一大包，該如何找起呢？

於是我決定先 Google：`chromium cookie`，在第一筆搜尋結果發現了很有幫助的資料：[CookieMonster](https://www.chromium.org/developers/design-documents/network-stack/cookiemonster)。這篇文章有詳細說明了 Chromium 的 Cookie 機制是怎麼運作的，並且說明核心就是一個叫做 `CookieMonster` 的東西。

再來就可以直接去看 Source code 了，可以在 `/net/cookies` 找到 [cookie_monster.cc](https://chromium.googlesource.com/chromium/src/+/master/net/cookies/cookie_monster.cc)。

還記得剛剛發現的問題重點之一，推測是跟`Secure`這個 flag 有關，所以直接用 `Secure` 當關鍵字下去搜尋，可以在中間的部分發現一個 `DeleteAnyEquivalentCookie` 的 function，以下節錄[部分原始碼](https://chromium.googlesource.com/chromium/src/+/master/net/cookies/cookie_monster.cc#1625)，1625 行到 1647 行：

``` c
// If the cookie is being set from an insecure scheme, then if a cookie
// already exists with the same name and it is Secure, then the cookie
// should *not* be updated if they domain-match and ignoring the path
// attribute.
//
// See: https://tools.ietf.org/html/draft-ietf-httpbis-cookie-alone
if (cc->IsSecure() && !source_url.SchemeIsCryptographic() &&
    ecc.IsEquivalentForSecureCookieMatching(*cc)) {
  skipped_secure_cookie = true;
  histogram_cookie_delete_equivalent_->Add(
      COOKIE_DELETE_EQUIVALENT_SKIPPING_SECURE);
  // If the cookie is equivalent to the new cookie and wouldn't have been
  // skipped for being HTTP-only, record that it is a skipped secure cookie
  // that would have been deleted otherwise.
  if (ecc.IsEquivalent(*cc)) {
    found_equivalent_cookie = true;
    if (!skip_httponly || !cc->IsHttpOnly()) {
      histogram_cookie_delete_equivalent_->Add(
          COOKIE_DELETE_EQUIVALENT_WOULD_HAVE_DELETED);
    }
  }
} 
```

這邊很貼心的幫你加上了註釋，說是：

> 如果有個 cookie 是來自 insecure scheme，並且已經存在一個同名又設置為 Secure 又 domain-match 的 cookie 的話，這個 cookie 就不該被設置

雖然不太理解 `domain-match` 指的到底是怎樣才算 match，但看來我們碰到的寫不進去 Cookie 的問題就是在這一段發生的。而且還有貼心附上參考資料：https://tools.ietf.org/html/draft-ietf-httpbis-cookie-alone
標題為：「Deprecate modification of 'secure' cookies from non-secure origins」。

內容不長，很快就可以看完，以下節錄其中一小段：

```
Section 8.5 and Section 8.6 of [RFC6265] spell out some of the
drawbacks of cookies' implementation: due to historical accident,
non-secure origins can set cookies which will be delivered to secure
origins in a manner indistinguishable from cookies set by that origin
itself.  This enables a number of attacks, which have been recently
spelled out in some detail in [COOKIE-INTEGRITY].
```
附註中的參考資料是這個：[Cookies Lack Integrity: Real-World Implications](https://www.usenix.org/conference/usenixsecurity15/technical-sessions/presentation/zheng)，裡面有附一段二十幾分鐘的影片，可以看一看，看完之後就會知道為什麼不能寫入了。

如果你還沒看，這邊可以幫大家做一個總結。要知道為什麼剛開始那個 case 不能寫入 Cookie，可以先想想看如果可以寫入，會發生什麼事情。

假如 `http://test.huli.com` 成功寫入 `.huli.com` 的 `csrftoken` 這個 cookie 的話，對 `http://test.huli.com` 似乎沒什麼影響，就多帶一個 Cookie 上去，看起來合情合理。

可是呢，卻對 `https://admin.huli.com` 有些影響。

原本 `.admin.huli.com` 並且設置為 `Secure` 的 Cookie 還是會在，但現在多了個 `.huli.com` 又是同名的 Cookie。當 `https://admin.huli.com` 送 request 的時候，就會把這兩個 Cookie 一併帶上去。所以 Server 收到的時候可能會是這樣：

```
csrftoken=cookie_from_test_huli_com; csrftoken=cookie_from_admin_huli_com
```

但碰到同名 Cookie 的時候，很多人都會只取第一個處理，所以 Server side 收到的 `csrftoken` 就會是 `cookie_from_test_huli_com`。

意思就是說，儘管你在 `https://admin.huli.com` 用 `Secure` 的方式寫了一個 Cookie，卻被其他不安全的來源（`http://test.huli.com`）給覆蓋過去了！

那蓋掉 Cookie 可以做什麼呢？舉幾個上面參考資料給的例子（但我不確定有沒有理解錯誤，有錯的話請指正），第一個是 Gmail 的視窗不是分成兩部分嗎，一部分是信箱，另外一部分是 Hangouts。攻擊者可以利用上面講的手法把原來使用者的 cookie 蓋掉，換成自己的 session cookie，可是因為 Hangouts 跟 Gmail 本身的 domain 不一樣，所以 Gmail 還是使用者的帳號，Hangouts 卻已經變成攻擊者的帳號了。

被攻擊的人就很有可能在不知情的狀況下利用攻擊者的帳號來發送訊息，攻擊者就可以看到那些訊息了。

第二個例子是某間銀行網站，假如在使用者要新增信用卡的時候把 session cookie 換成攻擊者的，那這張信用卡就新增到攻擊者的帳戶去了！

大概就是這樣，總之都是透過把原本的 cookie 遮蔽住，讓 server side 使用新的 cookie 的攻擊方法。

## 總結

我一開始碰到這個問題的時候真的滿苦惱的，因為怎麼想都想不到為什麼一個語法完全沒錯的指令沒辦法寫入 Cookie，而且`https://admin.huli.com`這個網站我平常也很少用到，根本不會想到是它的問題。

但這次把問題解掉之後重新回來看，其實過程中就有一些蛛絲馬跡可循，例如說可以透過「清掉 Cookie 就沒事」這點得知應該是跟其他 Cookie 有干擾，也可以從別的瀏覽器可以寫入這點得知應該是 Chrome 的一些機制。

過程中的每個線索都會帶你找到新的路，只要堅持走下去，一定能成功闖出迷宮。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好