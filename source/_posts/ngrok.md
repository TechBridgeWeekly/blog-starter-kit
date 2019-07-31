---
title: 測試 webhook 不再煩惱：ngrok
date: 2018-05-24 22:37:27
tags:
    - ngrok
author: huli
---

# 前言

以前在開發 Chatbot 的時候，發現最困難的不是寫程式，而是測試。

為什麼呢？因為 Chatbot 的原理就是接收到訊息時發送一個 request 到你指定的位置（webhook），你再 call API 然後回訊息。

可是呢，這個 webhook url 通常都會有兩個需求：

1. 必須要有一個 domain（不能只有 IP）
2. 要是 https

![](/img/huli/ngrok/line.png)

因此測試的流程就變成這樣：

1. 在 local 端先 coding，用肉眼 debug 看似一切都沒問題
2. 上傳到 server
3. 在聊天平台上測試，看有沒有收到訊息
4. 看 server 的 log debug
5. 發現問題，改 code，回到步驟 1

真正的 debug 流程是發生在 server 上面，因為聊天平台只能接受真的 domain，所以你也沒辦法在你的 localhost 測試。

現在的狀況應該好很多了，一開始 Chatbot 剛出來，大家都還在摸索然後官方文件又寫得不清不清楚的時候，來來往往就會花很多時間。你有可能會傳錯參數，或是搞錯 function。

正當我感到絕望的時候，有朋友跟我說了：你為什麼不用 ngrok 呢？

# Debug 的救星：ngrok

一句話介紹什麼是 ngrok：

> 一個可以把你 localhost 對應到 https public domain 的服務

例如說把你的 `localhost:5000` 對應到 `https://fj2rijo3.ngrok.com`。

這樣子做之後的好處就顯而易見了，你只要把 webhook 的 URL 填對應到的那個（完全符合那兩個條件），就可以在 localhost 上面測試了！

除此之外，使用上也超級無敵簡單。

你就去[ngrok 官網](https://ngrok.com/download)註冊帳號、下載一個程式，設定好 auth token 之後，就可以靠一個指令做到我剛剛講的事了。

```
./ngrok http 5000 
```

只要這行指令，就可以把你的 5000 port 對應到一個 ngrok 提供的 https domain。

![](/img/huli/ngrok/cmd.png)

除此之外，在 console 也會不斷印出 request 的訊息，讓你可以簡單看到有哪些 request 以及 server 回覆的狀態碼是什麼。

總之呢，有了這個 ngrok 的 https domain 以後，你就可以把這個網址填到 webhook url 上面去測試了！

# 總結

這次會寫出這篇文章是因為我最近又再次體驗到了 ngrok 的強大之處。這陣子在 debug 一個 PWA，發現 service worker 在 localhost 完全正常，可是 deploy 到 server 去的時候 lighthouse 的偵測就失敗了。

如果按照正常的流程，儘管你只是改一行 code，也要花費十分鐘跑 CI/CD，所以一小時之內你只有六次改 code 的機會，不能再多了。

後來我想起有 ngrok 這個十分好用的東西，就把 ngrok 對應到 localhost 來測了，好處就是你改任何 code 都可以即時反應，效率真的是好很多。

下次如果你也需要一個 https 的網址來 debug，那我誠心推薦你 ngrok。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好