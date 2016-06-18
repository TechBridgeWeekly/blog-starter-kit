---
title: 資料庫的好夥伴：Redis
date: 2016-06-18 10:40:38
tags: Redis, Database, Cache
author: huli
---

## 前言
[Redis](http://redis.io/) 是一個 in-memory 的 key-value database，因此常常被用在需要快取（Cache）一些資料的場合，可以減輕許多後端資料庫的壓力。這篇就來簡單介紹一下 Redis 提供哪些好用的東西，以及可以應用在什麼地方。

## 常用指令
[Redis 的官網](http://redis.io/commands)列出了支援的每一條指令，我們先來看看最簡單的：

### SET, GET

```
redis> SET mykey "Hello"
redis> GET mykey
"Hello"
```
前面有提到說 Redis 是一個 key-value pair 的資料庫，因此最簡單的 SET 就是設定某個 key 的值是多少，要取出來的話就用 GET 就好。

### INCR, DECR

```
redis> SET mykey "10"
redis> DECR mykey
(integer) 9
redis> INCR mykey
(integer) 10
```

顧名思義就是針對某個 key 加一或減一的意思，像是程式語言裡面的`mykey++`跟`mykey--`。  
還有 `INCRBY` 與 `DECRBY`，可以指定你要加減的數量是多少。

### HSET, HGET

```
redis> HSET mydata name "nick"
redis> HSET mydata nickname "nicknick"
redis> HGET mydata name
"nick"
```

H 就是 Hashmap 的意思，所以你可以存取一個 value 底下的 field，讓你可以更多元的使用，例如說你可以定義 key 的規則是：POST + 文章 id，裡面就可以存這篇文章的讚數、回覆數等等，就不用每一次都去 Database 裡面重新抓取。

### SADD, SCARD

```
redis> SADD myset "nick"
redis> SADD myset "peter"
redis> SADD myset "nick"
redis> SCARD myset
(integer) 2
```

SADD 的 S 就是 `Set` 的意思，這邊的 `Set` 指的是資料結構學過的那個 `Set`，裡面不會有重複的內容。

### LPUSH, RPUSH, LSET, LRANGE

```
redis> LPUSH mylist "a"
redis> LPUSH mylist "b"
redis> RPUSH mylist "c"
redis> LRANGE mylist 0 -1
1) "b"
2) "a"
3) "c"
redis> LSET mylist 0 "d"
redis> LRANGE mylist 0 -1
1) "d"
2) "a"
3) "c"
```

這邊的資料結構是 `List`，你可以選擇從左邊或是右邊 push 值進去，對應到的指令就是 `LPUSH` 與 `RPUSH`，`LSET` 則是指定某個 index 的 value 是多少。  

`LRANGE` 可以印出指定範圍的值，支援`-1`這種形式，表示最後一個值。

## 實際應用
Redis 好用的地方就在於速度很快，所以若是開發上碰到一些場合需要速度很快的話，你可以先考慮看看 Redis 是不是能夠幫助到你，以下就舉幾個我實際使用過的例子。  

### 短網址系統
其實短網址的原理非常簡單，就是一個 hash 對應到一個網址，hash 是用隨機產生，幾碼或是要有什麼符號可以自己決定，接著就把這組對應關係存到資料庫裡面，別人 query 相應的 key 時，你就 redirect 到相應的網址去就好了。  

也因為是這種 key-value 的一對一關係，所以非常適合使用 Redis。  
如果你不用像是 Redis 這種的 key-value cache，就必須「每一次」都從 Database 去 query。若是資料量小還好，但資料量一變大的時候，時間一定會增加，資料庫的負荷也會增加，因此在資料庫跟邏輯層之間引進一層快取，我認為是很好的選擇。  

實作的過程也很簡單，  

1. 使用者新增短網址，系統亂數產生 abc123 對應到 http://techbridge.cc
2. 把 key=abc123, value=http://techbridge.cc 寫進資料庫
3. 同上，但是是儲存在 Redis
4. 當有使用者點擊：abc123 這個網址時，先去 Redis 查有沒有這個 key
5. 有的話，redirect 到對應的網址
6. 沒有的話，只好去資料庫查，查完之後記得寫一份到 Redis

若是你的資料有超級多筆，又不想花很多的錢準備一台記憶體很大的 Redis Server（資料庫是用硬碟儲存，Redis 是存在記憶體，以儲存成本來說，資料庫會便宜許多），你可以使用 Redis 的 `Expire` 這個功能。  

當你在儲存資料的時候，你可以新增一個 `Expire time` 的參數，當這個時間一到之後，這個 key 就會自動被清除。舉例來說，短網址的 expire 可以設定成 7 天，當某個網址 7 天內都沒有被任何使用者訪問的話，就會自動被刪除。  

這樣的好處是你可以減少記憶體的使用量，只保持某些「熱資料」會存在 Redis，其他比較冷門、不常被訪問的數據，就存在 Database，等到被訪問的時候再寫到 Redis 即可。  

### 統計系統
其實上面講到的短網址服務，除了縮網址這個功能，還有另一個重點，那就是：統計資料。例如說 Google 短網址，會提供給你：造訪次數、圖表、用什麼裝置等等，這些才是短網址服務的核心。  

如果要做這個功能，那你勢必要記錄每一次 Request，或至少要把 Request 的內容（用什麼手機、時間點、IP）記錄下來，才有數據可以給使用者看。  

存在 Database，讀取也是每次都從 Database 讀的話，就會造成一些效能上的 issue，例如說每次 refresh 統計頁面，你就必須重新：`select conut(*) from short_url where id="abc123"`一次，才能抓到總共有多少人造訪。  

還記得我們提過的 `INCR` 嗎？這不是就派上用場了！可以自己定義 key 的格式，例如說：abc123:visit 代表 abc123 這個短網址的總共造訪次數，接著，只要在每一次的 Request 都執行：`INCR abc123:visit`，這個 key 裡面就是你要的數字了，以後都從 Redis 讀取就好。  

除了這個以外，假設你想要提供「不重複 IP 訪問次數」，前面提到的 `Set` 就很適合。可以把每一個 Request 的來源 IP 都丟進一個 Set，只要用 `SCARD` 就可以知道有多少不重複 IP 了，很方便對吧！

### 高即時性排名系統
我曾經做過一個專案，需求如下：  

1. 中午 12 點開放使用者進入網站，並且回答一題問題
2. 回答完後會看到自己的排名（依答題時間排序），照名次獲得獎品
3. 只有前 300 名有獎品，之後都沒有

可以先想一下有哪些地方會需要跟資料庫溝通  

1. 進入網站時，要先檢查是否超過 300 名，有的話提示活動結束（select count(*)...）
2. 接著檢查使用者是否已答題過，已答題過的話就顯示排名（select .. where id=..）
3. 若沒答過，顯示答題頁面
4. 答題結束之後，顯示使用者名次（insert into .. id=..）

由於這個活動只有前 300 名有獎品，預估使用者有 10000 人的話，可能在 10 秒內這個活動就結束了！  

10 秒內你的資料庫必須「同時承受」這麼多個 query，可能會有點吃不消，而且仔細檢視之後，會發現很多地方其實沒有必要用資料庫來做，或者是說，用 Redis 來做會更好！  

例如說，可以這樣規劃：

1. 用一個 key：isOver 儲存活動是否結束
2. 用 account 當做 key，裡面儲存使用者的名次

上面的流程就可以改寫成：

1. 進入網站時，去 Redis 讀取 isOver，查看活動是否結束
2. 檢查使用者是否答題過，看 Redis 的使用者帳號這個 key 有沒有資料
3. 若沒答過題且答完題，寫入資料庫，並且把名次寫入 Redis
4. 若是這個使用者的排名>=300，設定 isOver = true

原本需要三個對資料庫的操作，現在縮減到只剩下最必要的一個，其他都可以交給 Redis 處理。而且又因為 Redis 是 in-memory 的資料庫，反應速度非常快！再加上我們的 key 並不多（一萬多個 key 而已），用到的記憶體很少。  

就這樣，透過 Redis 的幫助，很順利的就可以解決原本資料庫負載太重可能會很慢甚至掛掉的問題。

## 總結

若是下次你有些專案使用者很多，或是需要很快速的返回資訊，但是又怕資料庫撐不住，不妨想想是不是能夠導入 Redis，或其他也是做快取的 service。其實在很多場合，如果快取運用得宜的話，可以減輕很多資料庫的負擔，同時也加快響應的速度。

若是你對 Redis 很有興趣，可以參考 [Redis 設計與實現](http://redisbook.com/) 這個網站。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好