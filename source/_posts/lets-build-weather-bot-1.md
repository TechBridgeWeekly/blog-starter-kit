---
title: 來寫個氣象機器人吧！(Part 1)
date: 2018-05-10 18:00:00
tags:
  - line-bot
  - messenger-bot
  - weather
  - bot
author: tigercosmos
---
# Let's build a weather bot!

![weather bot](/img/tigercosmos/weather_bot.png)

## 簡介

沒有人會懷疑了解天氣的重要性，我們總是看氣象預報或查天氣來決定等下外出時要不要帶傘，衣服要穿多厚是否需要帶件外套，或是需不需要先擦防曬油戴一副墨鏡出門等等。

想查天氣的時候我想大部分人可能就是 Google 一下、上中央氣象局網站、使用 APP、看新聞。或是有種很潮的做法是問 Siri，不過效果可能不太好 ⋯⋯。

![siri air](/img/tigercosmos/siri_air.png)

其實還有一種做法是建立一個[天氣機器人](https://www.facebook.com/weather.bot.tw/)，其實概念就是聊天機器人，讓我們可以和機器人對話取得天氣資訊。你可能會問那和查 App 或問 Siri 差在哪裡呢？首先氣象機器人因為著重在氣象資訊上，所以資料處理非常細膩，尤其是台灣地區的資料是處理得非常完善，並且國外地區也支援。再來就是聊天機器人最好玩的地方就是，大家可以一起跟機器人對話，這也是聊天機器人最好玩的地方！

![bot demo 1](/img/tigercosmos/bot_demo1.png)

機器人在群組中可以和大家一起互動，當有人提到天氣資訊的時候，機器人會自動回應，不僅少了查詢的時間，大家也都能一起得到氣象資訊。此外也可以做個人查詢使用，並且支援跨平台，可以在 Line 或 Messenger 甚至其他聊天軟體上使用。

可以先玩看看，第一次使用請先輸入 `help` 來查看指令：

- [氣象機器人 Messenger Bot 連結](https://m.me/weather.bot.tw)
- [氣象機器人 Line Bot 連結](https://line.me/R/ti/p/%40lbz9453s)

接下來我會介紹如何建立一個氣象機器人，一一介紹聊天機器人建立、使用 Bottender 框架、部署、比較細節的設定、氣象資料的來源和處理。本篇屬於進階的探討文章，基礎的部分不會特別著墨，但我會將需必備的基礎資料都提供給大家。

## 如何建立機器人

想要建立一個氣象機器人，最重要的當然就是先有個機器人（bot）。一般的聊天機器人原理大致都相同，你會需要建立一個伺服器，就類似網頁伺服器一樣，只是現在是伺服機器人。然後伺服器可以透過 Webhook 機制與聊天平台溝通，聊天平台藉由 Webhook 中的要求傳送給平台使用者。舉例來說，當某個人傳訊息給機器人，聊天平台的伺服器就會知道，並且透過 API 傳訊息告訴你的伺服器，說有人傳訊息給你喔！你的伺服器經過一番處理之後，可以呼叫 API 告訴聊天平台說，請讓機器人回覆他「xxxxx」，這個過程就是所謂 Webhook 了。

由於我想做跨平台的機器人，意味者要能在 Line、Messenger、Telegram、Slack 等等聊天平台上都能使用，但是每家平台的 API 都不盡相同，每個平台都要特別去針對他來改真的很麻煩，最佳的實作方式是使用框架（framework）當作底層，待會會細談。我們先來看怎麼申請、寫簡單的機器人吧！

這邊我針對 Line 和 Ｍessenger 一起做介紹，Slack、Telegram 等等其他的由於我沒有去實作（考慮到使用人數的效益）以及原理和做法其實也不會差太多，就留給大家自行查詢了。

### Line Bot

想使用 Line Bot 的話，要先申請 [Line@](https://developers.line.me/en/) 帳號，可以參考 OOXX 寫的 [LINE BOT 實戰 ( 原理篇 )](http://www.oxxostudio.tw/articles/201701/line-bot.html)。這篇介紹非常完整，照著文章步驟，使用 NodeJS 可以快速建立一個 Hello World Bot。這邊可以試著寫一個，或是等下會介紹用框架來做。

Line Bot 建立非常方便，如果要開放公開給大家使用也很容易，只要讓別人加你的機器人好友就好。完全不需要其他的審查或申請（除非要申請認證帳號），相較於 Messenger 可說是非常友善。不過免費帳號只能夠「回應」，不能讓機器人主動「推播」，這是與 Messenger 非常不同的地方，如果要做到跨平台且都支援主動推播，就只好買付費服務了。

另外 Line Bot 所有 Post API 只接受內容為 HTTPS 的 URL，例如你要傳送圖片給使用者，你可以在 API 中附上圖片的 URL，但必須是 HTTPS 的連結，如果是 HTTP 的話，Line Bot 會不反應，且不會報錯誤訊息。希望大家不要在這地方被雷到了。

### Messenger Bot

Messenger Bot 相較於 Line Bot 來說更加強大，因為他有更多 API 可以呼叫，它支援語意分析等服務。不過 Messenger Bot 不直接支援群組聊天（可以透過外掛辦到），並且申請步驟真的煩瑣很多。首先要用 Messenger Bot 的話要申請 Facebook Developer，並且申請開發應用服務，並且機器人必須綁定一個粉絲專頁。詳細申請步驟可以參考[超簡易-Messenger-API-初探](https://blog.arvinh.info/2016/04/17/%E8%B6%85%E7%B0%A1%E6%98%93-Messenger-API-%E5%88%9D%E6%8E%A2/)。可以只看如何申請，實作的部份先了解就即可。因為等下會介紹用框架做。

另外就是如果只要自己測試機器人的話，Webhook 設定好伺服器有架好就可以看到可以和機器人對話。但是如果要公開給使用者用的話，要先通過臉書的審查，可以看臉書官方文件「[提交您的 Messenger Bot](https://developers.facebook.com/docs/messenger-platform/app-review)」來完成，通常要好幾天審查通過之後，你的 Messenger Bot 才能被其他使用者使用。

### Bottender 框架

如同先前提到，這是個百家爭鳴的時代，我們的力氣不足以應付各種標準，於是乎就誕生了框架。框架幫我們把最底層的麻煩事都打理好了，並且替我們考量到跨平台的議題。比方說就傳送訊息來說，每個平台需要呼叫的 API 不盡相同，原本你可能需要個別設定，但是在使用 Bottender 這類框架之後，一些參數設定好之後（token、secret 之類的），一律只需要呼叫 `context.sendText` 函數，你什麼都別管，他就幫你搞定了！即使你只使用單一平台，框架幫你處理底層的 API 設定也非常方便，不一定要跨平台才能使用。

稍微介紹一下，這邊引用 [Bottender 團隊寫的文章](https://blog.yoctol.com/yoctol-2017-review-1-6e876ad4de11)：

> 優拓投入近半年心力的產品 — 多平台 Chatbot 框架「Bottender」終於公開並在 [GitHub 開源](https://github.com/Yoctol/Bottender)。Bottender 是我們分析多套既有開發框架 (Framework) 後，綜合了 Chatbot 開發上的實務經驗，以「Learn Once, Write Anywhere」為核心理念，所打造出的一款開源 Framework，具備靈活、模組化、多平台等優勢。

使用 Bottender 真的很簡單，我假設你會用 NodeJS with Express，那麼概念根本是一樣的，這邊我們簡單建立一個 Line 的 Hello World Bot：

```js
const { LineBot } = require('Bottender');
const { createServer } = require('Bottender/express');
  
const bot = new LineBot({
  channelSecret: process.env.channelSecret, // 填上 Channel Secret
  accessToken: process.env.accessToken // 填上 Access Token
});
  
bot.onEvent(async context => {
  await context.sendText('Hello World');
});
  
const server = createServer(bot);
  
server.listen(5000, () => {
  console.log('server is running on 5000 port...');
});
```

這樣寫完就是最簡單的 Hello World Bot 了，相較於上面幾篇我說只需要看概念就好的教學，是不是簡約多了？

參數設定的地方，可以看到 `process.env.channelSecret` 這樣的寫法，這是代表我們不會把真的 Secret 放在程式碼中，所以我們讓程式從環境變數中取得，環境變數通常可以在你架設伺服器的服務商網站設定，以 Heroku 為例的話，建立一個 App 之後，可以在 `Setting > Config Vars` 中設定：

![heroku demo1](/img/tigercosmos/heroku_demo1.png)

那假設我們想要跨平台呢？

理論上做法應該有好多種，這邊分享我的作法，不過可能不是最聰明的，我在 Heroku 建立兩個 APP，一個是給 Line 另個給 Messenger，然後兩個 Heroku 都綁定 Github repo 的 master：

```js
const {
    LineBot,
    MessengerBot
} = require('Bottender');
  
const bot = (process.env.chatbotPlatform == 'messenger') ?
            new MessengerBot({
                accessToken: process.env.messengerAccessToken,
                appSecret: process.env.messengerAppSecret,
            }) : new LineBot({
                channelSecret: process.env.lineChannelSecret,
                accessToken: process.env.lineChannelAccessToken
            });
  
bot.onEvent(async context => {
  await context.sendText('Hello World');
});
  
const server = createServer(bot);
  
server.listen(5000, () => {
  console.log('server is running on 5000 port...');
});
```

因為我想要做到一份程式碼可以跨平台，這樣我只需要更新 master 主幹，就可以同時讓兩個 Heroku APP 一起更新，不需要再分別修改不同分支，因為是可以讓 Heroku 各別綁定不同的分支，但我不想要這樣。

所以我在環境變數中設定這個 Heroku 是給哪個平台用的，程式碼運行時會從環境變數 `process.env.chatbotPlatform` 中得知，並初始化對應的平台，因此可以看到我把 `LineBot` 和 `MessengerBot` 同時寫進去了。這樣就是一種可以跨平台的方法。

至於 `bot.onEvent()` 則是處理和使用者對話的部分，不同平台都可以共用。如果有特別需要針對不同平台客製的話，再用 `if` 判斷即可，或是特別寫函數處理，如同以下問題：

#### Bottender 小細節

這邊必須提一下，由於我是 Line 免費版搭配 Messenger，所以 Line 只能回覆不能推播，但是臉書卻沒有這限制。這個特性使得我們用 Bottender 要特別注意，因為 Bottender 中沒有特別去區分，假設今天我要回覆照片，在 Line 上只能用 `context.replyImage`，但 Messenger 要使用 `context.sendImage`，可是我卻不能直接讓 Line 也使用 `context.sendImage`，因為主動推播要使用付費版的 Line，並且 Messenger 並不能使用 `context.replyImage`。於是就有了這樣尷尬的情況發生了！

為了讓平台可以共用，我又自己包裝了一層函數：

```js
async function platformReplyImage(context, url) {
    if (process.env.chatroomPlatform == 'messenger') {
        await context.sendImage(url)
    } else {
        await context.replyImage(url);
    }
}
```

如此一來就解決了上述問題，此問題也發生在 `sendText` 和 `replyText` 中，希望 Bottender 可以針對這塊改善，我也發了一個 [issue](https://github.com/Yoctol/Bottender/issues/264) 表達我的想法。

## 未完待續

下一篇：[來寫個氣象機器人吧！(Part 2)](/2018/05/21/lets-build-weather-bot-2/)，將介紹氣象機器人的核心，氣象資料取得與處理。

> 你可以在這邊查看[氣象機器人的 Github Repo](https://github.com/ntu-as-cooklab/weather-bot)

---
> ***關於作者***
**劉安齊**
軟體工程師，熱愛寫程式，更喜歡推廣程式讓更多人學會。歡迎追蹤 **[微中子](https://www.facebook.com/CodingNeutrino/)**，我會在上面分享各種新知與最新作品，也可以去逛逛我的 **[個人網站](http://tigercosmos.xyz)** 或 **[Github](https://github.com/tigercosmos)**。