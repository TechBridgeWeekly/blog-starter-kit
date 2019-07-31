---
title: 直播協定 hls 筆記
date: 2016-12-03 01:22:49
tags:
   - hls
   - streaming
   - videojs
   - hlsjs
author: huli
---

# 前言
最近剛好在做直播相關的東西，雖然說是做前端，但還是必須懂一些直播的原理  
至少要知道有哪些格式，以及各種格式的優缺點是什麼，做起來也會比較踏實

這篇就簡單記錄一些心得跟資料，如果想比較深入了解 hls 的，可以參考下面這兩篇文章：

1. [直播协议的选择：RTMP vs. HLS](http://www.samirchen.com/ios-rtmp-vs-hls/)
2. [在线视频之HLS协议—学习笔记：M3U8格式讲解及实际应用分析](http://www.eduve.org/knowledge/732)

# hls 是什麼？
我覺得以直播來說，hls 是一個相當好懂的協定，其實就是透過一個 `.m3u8` 的播放列表，然後裡面有多個 `.ts` 的檔案  
你只要照著播放列表裡面給你的檔案順序播放就好了，聽起來很容易吧！

為了讓大家更明白，直接附上擷取自某處的播放列表：

```
#EXTM3U
#EXT-X-VERSION:3
#EXT-X-ALLOW-CACHE:YES
#EXT-X-MEDIA-SEQUENCE:4454
#EXT-X-TARGETDURATION:4
#EXTINF:3.998, no desc
25133_src/4460.ts
#EXTINF:3.992, no desc
25133_src/4461.ts
#EXTINF:3.985, no desc
25133_src/4462.ts
#EXTINF:3.979, no desc
25133_src/4463.ts
#EXTINF:3.996, no desc
25133_src/4464.ts
```

就算你沒看過這個格式，你大概看一下也可以猜出來它在做什麼  
每一個 ts 就是一個片段，然後 `#EXTINF:3.996` 代表這個片段的時間長度  
`#EXT-X-TARGETDURATION:4`，這邊的數字必須比播放清單中的任何一個影片的時間都大。代表播放器應該每隔幾秒去抓一次新的播放清單  

例如說，下一次抓到的可能會長這樣：  

```
#EXTM3U
#EXT-X-VERSION:3
#EXT-X-ALLOW-CACHE:YES
#EXT-X-MEDIA-SEQUENCE:4455
#EXT-X-TARGETDURATION:4
#EXTINF:3.992, no desc
25133_src/4461.ts
#EXTINF:3.985, no desc
25133_src/4462.ts
#EXTINF:3.979, no desc
25133_src/4463.ts
#EXTINF:3.996, no desc
25133_src/4464.ts
#EXTINF:3.998, no desc
25133_src/4465.ts
```

就是最後面多了一個片段。所以只要一直維持這個規則，就能夠不斷取到新的片段  
那如果很不巧的，server 沒有及時產生出播放列表怎麼辦呢？

例如說在第 4 秒的時候去拿，發現沒更新，server 在第 4.5 秒才把新的播放片段產生出來。如果發生這種「拿了播放清單，但長的一樣」的情形，就會把抓取的時間減一半，直到抓到為止。像以上情形，第 4 秒沒拿到新的，就會隔 2 秒之後再去抓。

這個規則可以參考：[HTTP Live Streaming draft-pantos-http-live-streaming-20](https://tools.ietf.org/html/draft-pantos-http-live-streaming-20#section-6.3.4)

> When a client loads a Playlist file for the first time or reloads a
   Playlist file and finds that it has changed since the last time it
   was loaded, the client MUST wait for at least the target duration
   before attempting to reload the Playlist file again, measured from
   the last time the client began loading the Playlist file.


>If the client reloads a Playlist file and finds that it has not
   changed then it MUST wait for a period of one-half the target
   duration before retrying.


至於做直播最關心的延遲問題，也可以直接從這個播放列表直接推測出來  
以上面的例子來說，一共有 5 個片段，每一個片段 4 秒，延遲就是 20 秒  
Apple 官方建議的是 3 個片段，每個片段 10 秒

> What duration should media files be?
A duration of 10 seconds of media per file seems to strike a reasonable balance for most broadcast content.

> How many files should be listed in the index file during a continuous, ongoing session?
The normal recommendation is 3, but the optimum number may be larger.

可參考：[Apple: HTTP Live Streaming Overview](https://developer.apple.com/library/content/documentation/NetworkingInternet/Conceptual/StreamingMediaGuide/Introduction/Introduction.html#//apple_ref/doc/uid/TP40008332-CH1-SW1)

不過依照官方的建議，就會有 30 秒的延遲，當然延遲越久直播的狀況會越好，可是體驗也會比較差一點。因此，我們可以來看看幾個直播網站都是怎麼設定的

先來看看直播大頭：[Twitch](twitch.tv)  

```
#EXTM3U
#EXT-X-VERSION:3
#EXT-X-TARGETDURATION:5
#ID3-EQUIV-TDTG:2016-11-26T02:40:23
#EXT-X-MEDIA-SEQUENCE:376
#EXT-X-TWITCH-ELAPSED-SYSTEM-SECS:1511.137
#EXT-X-TWITCH-ELAPSED-SECS:1508.980
#EXT-X-TWITCH-TOTAL-SECS:1535.137
#EXTINF:4.000,
index-0000000377-6zCW.ts
#EXTINF:4.000,
index-0000000378-vHZS.ts
#EXTINF:4.000,
index-0000000379-Gkgv.ts
#EXTINF:4.000,
index-0000000380-PNoG.ts
#EXTINF:4.000,
index-0000000381-h58g.ts
#EXTINF:4.000,
index-0000000382-W88t.ts
```

6 個片段乘上 4 秒 = 24 秒  
可是如果你仔細觀察（開 chrome devtool 就可以了），實際上 twtich 的播放器在拿到列表以後，會直接嘗試從「倒數第三個」片段開始載入，所以延遲就縮短為 3 個片段乘上 4 秒 = 12 秒了  

再來看看台灣的 [livehouse.in](https://livehouse.in)

```
#EXTM3U
#EXT-X-VERSION:3
#EXT-X-ALLOW-CACHE:NO
#EXT-X-MEDIA-SEQUENCE:2291
#EXT-X-TARGETDURATION:6

#EXTINF:5.2090001106262207,
1480116261segment_v02291.ts
#EXTINF:5.2080001831054688,
1480116261segment_v02292.ts
#EXTINF:5.2080001831054688,
1480116261segment_v02293.ts
```

5 乘上 3 = 15 秒  

所以一般用 hls 的直播網站，延遲大概都會在 10~20 秒這個區間以內  
我猜比這個短的話對 server 壓力可能很大，而且網速慢的話，看起來會很卡  
比這個長的話雖然很順，但是使用者體驗不太好，延遲太高  
所以能找到最好的延遲就是在這個區間內了

最後，我們來看看如果要在網頁上播放的話，有哪些選擇  
因為現在已經是個 flash 快死掉的年代了，所以如果可以的話，首選當然是 html5  
瀏覽器支援度不夠高的話再 fallback 回去 flash

先來介紹一下現成的商業授權播放器，例如說 [jwplayer](https://www.jwplayer.com/) 或是 [flowplayer](https://flowplayer.org/)，都是很不錯的選項。尤其是當 open source 的方案出現問題你又修不好的時候，就會很希望公司花錢買一個商業播放器，一切問題都搞定。

open source 的方案大概就是 [videojs](http://videojs.com) 一支獨秀了，有沒有其他的後起之秀我是不知道啦，有的話麻煩推薦一下。

然後因為 hls 這個格式瀏覽器本身是沒辦法播放的（除非是 iOS 或是 Safari），所以要搭配一些 plugin  
videojs 官方有一個 [videojs-contrib-hls](https://github.com/videojs/videojs-contrib-hls)，加上去之後就可以播放了，但我自己用過以後感覺不是很好。

最後選擇了知名的影音網站 [dailymotion](http://www.dailymotion.com/sg) 提供的開源解決方案 [hls.js](https://github.com/dailymotion/hls.js/tree/master)。

[這一篇](http://engineering.dailymotion.com/introducing-hls-js/)是他們官方的部落格，有介紹說為什麼要自己寫一套，以及解決了哪些問題，滿值得一看的，可以順便了解一下。

另外，hls 也支援在不同的頻寬下的自動切換，它的檔案會長成這樣：

```
#EXTM3U
#EXT-X-STREAM-INF:BANDWIDTH=1280000,AVERAGE-BANDWIDTH=1000000
http://example.com/low.m3u8
#EXT-X-STREAM-INF:BANDWIDTH=2560000,AVERAGE-BANDWIDTH=2000000
http://example.com/mid.m3u8
#EXT-X-STREAM-INF:BANDWIDTH=7680000,AVERAGE-BANDWIDTH=6000000
http://example.com/hi.m3u8
#EXT-X-STREAM-INF:BANDWIDTH=65000,CODECS="mp4a.40.5"
http://example.com/audio-only.m3u8
```

提供不同頻寬下應該載入的檔案，例如說網路狀況很好，就載入高畫質的，不好的話就載入低畫質。很多播放器也都可以支援這類型的檔案清單，可以自動偵測網速調整畫質，是個很厲害的功能。

# 結論
其實在直播這一塊，前端工程師能做的真的不多。要真的做到極致的話，大概就是去優化播放器了，可是這是一件超級困難的事情，因為你必須要懂一大堆東西才能做好這一塊，而且要付出的成本也很高。

建議大家還是先找現成的開源播放器來用，至少碰到問題還有可能自己修，也不用自己再動手做一個。開源的解決方案不行的話，再參考商業上授權的播放器，通常就很夠用了。如果都還是不行，在時間跟成本都允許的情況下，再自己做一個播放器出來。


關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好