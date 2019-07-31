---
title: 利用 Wit.ai 讓你的 Messenger Bot 更聰明！
date: 2016-07-02 19:32:59
tags:
  - Messenger
  - Wit.ai
  - Bot
author: ArvinH
---

今天我們要讓我們的 Chat Bot 更加聰明，利用被 Facebook 收購的 Wit.ai 所提供之 API，可以很方便的讓 Chat Bot 有了 NLP 的支援，讓他/她更加聰明！

實際上 Wit.ai 的介面並沒有我想像中的好用，需要很有耐心地把官網上的教學一步步做完，並且了解他所定義的名詞代表之含義，雖然寫得很詳細，但畢竟是英文，因此就記錄一下整個過程，並跟大家分享。


<!-- 
  Wit.ai 操作步驟
-->
## Step 1 註冊 Wit.ai 帳號

先到 [Wit.ai](https://wit.ai/) 的官方網站註冊一個帳號，有 GitHub 與 Facebook 可以選擇。

![Wit.ai](/img/arvinh/wit.aifp.png "Wit.ai")

## Step 2 Dashboard 設定

接著你會進到你的 Dashboard

![Dashboard](/img/arvinh/witDashboard.png "Dashboard")

點選右上角的 + 號，進入 App 設定頁面，進行簡單的設定，基本上只要設定 名字 與 描述，語言等等之後還能修改。
這邊要提一下，我本來想設定成 Chinese，但後來在建立機器人對話故事時，發現他的中文支援好像還不是很完善，常常判斷不出 Entity，因此這邊還是先以英文為例子，如果有高手知道怎麼解的話也歡迎告訴我！

![Setting](/img/arvinh/setting.png "Setting")

## Step 3 創建對話情境 (Story)

繼續，設定完後就進入到編輯界面，在 Wit.ai 裡面，你的機器人與一般使用者的對話情境，都叫做 `Story`，你可以透過創建 Story 來定義出在這個情境下，你的機器人要怎麼跟使用者對話。

![Create Story](/img/arvinh/createStory.png "Create Story")

![Story](/img/arvinh/story-1.png "Story")
整個介面就像是一個對話視窗，看起來頗親切，左邊是 User says，右邊是 Bot sends, Box executes 與 Jump。
先簡單介紹，看完後面的例子會更清楚。

* **User says**： 顧名思義就是定義 User 會說的話，並且你可以設定 User 的句子中，有哪些關鍵字是你需要的、哪些文字是代表什麼含意，在 Wit.ai 的世界中，這樣的東西叫做 `Entity`，後面會再度說明。

* **Bot sends**： 就是機器人要回覆的句子，這邊可以帶入一些參數，像是 user 所提及的一些關鍵字，或是機器人向外呼叫 API 所得到的結果。

* **Box executes**： 就是讓你定義機器人要執行的 function，真正的實作不會在這邊，這邊只是定義名稱，以及要接收的 context 與 吐回的變數名稱。

* **Jump**： 則是讓你能夠在滿足設定的一些條件之下，跳回到某個 Box executes 或是 Box sends 的步驟去執行。

## Step 4 定義使用者語句

接下來我們先定義 User 可能會對我們的機器人說的話，像是使用者可能會跟機器人打招呼，我們就可以在對話框的 User says 輸入 Hello，並且 highlight 起來以後設定 Entity，Enity 在 Wit.ai 裡面，就是用來讓系統判斷使用者輸入句子時，哪些關鍵字是要抽取出來做處理的，你可以依照該關鍵字的特型來設定相對應的 Entity 類別。
這邊我們就自定義一個 Entity 名稱叫做 greeting，當然 Wit.ai 也有許多內建好的 Entity，當你點選 `Add a new entity` 時，他會有提示。

![Greeting Story](/img/arvinh/greeting-story.gif "Greeting Story")

你可能會想說，打招呼又不會只說 Hello，你這樣設定的話，我照之前方法 hardcode 寫在 server side 就好了呀，要 Wit.ai 幹麻。

Wit.ai 當然沒有這麼簡單，介面上方的 Tab 是不是有個**學士帽**寫著 `Understanding`？ 在這個地方你有三種方式可以用來訓練你的機器人：

* **增加例句**：

在上方寫著 `Test how your bot understands a sentence` 的地方輸入更多的例句，並且如同前面步驟般去定義 Entity，這邊要注意的是，當你輸入完例句後，記得點選下方綠色的 Validate，讓 Wit.ai 去記住你的例句。成功的話就會看到下方 Entity 的 Values 欄位會多出你剛剛例句中所抓取到的關鍵字（以下圖例子來說就是 Hi 也被我們納進 greeting 這個 entity內了，只要之後 user 輸入 Hello 或是 Hi，都是屬於 greeting）

![understands example](/img/arvinh/understand-ex.gif "understands example")

* **增加 Entity 的 Keyword 與 Synonyms**：

你也可以點選下方的 Entity 名稱，進去手動增加關鍵字或是同義詞。
**關鍵字**與**同義詞**的關係有點像是父子類別，這邊舉個比較易懂的例子，如下圖，我們有個 Entity 叫做 Beer，底下的 關鍵字是 啤酒 與 紅酒，當使用者喊出啤酒的時候，機器人就會知道是屬於 Beer 這個 Entity。

但啤酒有很多種種類，我們可以在同義詞這邊增加：蜂蜜啤酒，這樣當使用者輸入 蜂蜜啤酒 時，機器人就會判斷目前的 Beer Entity 的 Value 為 啤酒，而非紅酒。
相同的，我們也可以設定 葡萄酒 為 紅酒 的同義詞，讓使用者喊出 葡萄酒 時，機器人會判斷為 紅酒。

**要注意的是，機器人記住的 Entity Value 會是以 Keyword 為主，也就是你輸入蜂蜜啤酒，但對機器人來說，偵測到的 Beer Entity，其值為 啤酒，而非蜂蜜啤酒。**

![understands keyword setting](/img/arvinh/beerexp.gif "understands keyword setting")

* **設定 Entity 的 Search strategy**：

最後在設定 Entity 的地方還有 Search strategy 可以選擇，意思是你希望 Wit.ai 要怎麼樣從句子中找出這個 Entity 。

1. trait: 如果你想設定的 Entity 並不是由單一一個關鍵字就可以判斷，也不是靠句子中幾個關鍵字或是子句能夠辨別，而是需要整個句子來判定的話，就要設定成 trait，像是今天的例子裡面，想要問新聞，這種使用者的 **意圖** 就很適合設定成 trait。

官網範例：[出處](https://wit.ai/docs/recipes#categorize-the-user-intent) 
![example: trait](/img/arvinh/trait.gif "example: trait")

2. free-text: 如果你想要擷取使用者句子中的某段文字，而該段文字並不是特定的關鍵字時，就要設定 free-text，像是 User 說：“Tell Jordan that I will be home in ten minutes”，而你想要擷取 ”I will be home in ten miutes"，這時就可以把想要擷取的句子選取起來，設定為 free-text，要注意的是，free-text 一定要搭配 keywords 一起使用，有點像是告訴 Wit.ai 說這段話都算是 keywords，但不一定要 exactly match 才能觸發。

官網範例：[出處](https://wit.ai/docs/recipes#extract-an-entity-that-is-a-substring-of-the-message) 
![example: free-text](/img/arvinh/freetext.gif "example: free-text")

3. keywords: 要完全符合你預先設定的關鍵字才會觸發。

官網範例：[出處](https://wit.ai/docs/recipes#categorize-the-user-intent) 
![example: keywords](/img/arvinh/keyword.gif "example: keywords")

<!-- 
  問新聞story -> bot回應 -> branch介紹
-->
## Step 5 定義機器人回覆語句

介紹了這麼多瑣碎的東西後，回過頭來看看我們要怎麼設定機器人的回覆。以最前面的例子來說，當機器人收到 greeting 的 Entity 後，可以讓用相同的 entity value 回覆，並加上簡單的介紹。
點選下方的 Bot sends，對話框就會出現機器人的部分，你可以在裡面輸入機器人的回覆語句，想要的變數可以用`{ }`包起來，這邊我們直接使用 greeting 這個 entity，這樣就能用同樣的 Entity 去回覆。

![機器人回覆](/img/arvinh/botsends.gif "機器人回覆")

畫面右下方有個浮動的按鈕 “Press `~` to chat with your bot”，可以讓你即時測試一下。

![機器人回覆測試](/img/arvinh/botsentResult.gif "機器人回覆測試")

## Step 6 設定機器人執行動作

當然機器人不能只單純回話，要能夠執行動作，這邊我們創建另一個對話情境，設定讓我們的機器人幫忙找新聞！
這邊我先設定好一個使用者語句與相關 Entity，接著先讓 Bot executes 動作，也就是讓他執行一個 Funtion，這邊只是定義 Function 名稱以及 輸出 的參數，實際的實作要等到後面撰寫程式時才需要。

![機器人執行函數](/img/arvinh/botexec.gif "機器人執行函數")

從上圖來看，我設定了一個 `getNews` 的函式，並且設定一個`context`為 newsResult，代表這個 function 會有一個變數 `newsResult` 可以供外部與自己使用。此外，機器人會先回覆一個訊息，其中包含你的 `search_query` entity 之 value 

設定完一樣要進行一下測試，當你輸入使用者語句後，機器人會執行函式，並問你要執行哪個 Context，這時你就點選剛剛設定的 `newsResult` 當作回覆，教導機器人記住這個 `context`

![機器人執行測試](/img/arvinh/botexectest.gif "機器人執行測試")

若使用者沒有說他想找什麼新聞怎麼辦呢？這時候就是另用另一個 Context 來判斷了！你可以設定一些 `context branch`，透過 先前提到的 **Jump** ，讓機器人根據 `Context` 的不同來執行不同回覆。
透過定義一個 `missNews` 的 Context，告訴機器人，當沒有 `search_query` 時，可以怎麼做。

如下圖，你需要設定一個 `BookMark` 讓你的機器人可以 `Jump` 到那個 Context下。

![Branch Context](/img/arvinh/botJump.gif "Branch Context")

設定完後依然需要先測試一下，訓練一下你的 Bot。在這邊你要告訴機器人目前是哪個 Context 。要注意的是，你必須要把非當下必要的 Context 移除，像是下圖中，在 User 回答 Brexits後，需要把 missNews 這個 context 點選掉，這樣 Bot 才會正常的跳回 `getNews`。

![Branch Context Test](/img/arvinh/botJumpTest.gif "Branch Context Test")

## Step 7 套用 API 與實作 Function

前面幾個步驟做完後，就有個基本的使用者與機器人互動情境，接下來就可以開始實作函式，並套用 API 了。
這邊以 Node.js 為例子，你需要先到你的專案底下加入 `node-wit` 這個 package。

```sh
		npm install --save node-wit
```

之後可以先測試一下，修改官方的 `example/quickstart.js`，實作出 `getNews` 函式，這邊先簡單 echo 一下就好。程式碼短短的，你需要注意的是 `actions` 這個 object，裡面定義了 Bot 要執行的動作函式，`send` 是用來讓 Bot 回話的，這一定要有，而我們自己定義的 `getNews` 就定義在下方。

`getNews` 裡面利用 `firstEntityValue` 從接收到的 entities 中找出你要的，這邊我們要的當然是 `search_query` 的值。接著就可以去進行需要的處理，呼叫 API 等等。

**唯一要注意的就是這邊需要使用 Promise 回傳喔！**

```js
const actions = {
  send(request, response) {
    const {sessionId, context, entities} = request;
    const {text, quickreplies} = response;
    return new Promise(function(resolve, reject) {
      console.log('sending...', JSON.stringify(response));
      return resolve();
    });
  },
  getNews({context, entities}) {
    return new Promise(function(resolve, reject) {
      var search_query = firstEntityValue(entities, 'search_query')
      if (search_query) {
        context.newsResult = search_query + '最近很多人討論...' ; // we should call a real API here
      } else {
        // To-do
      }
      return resolve(context);
    });
  },
};

```
執行 `node example/quickstart.js <Wit.ai server-side Token>`
就會得到以下結果。

![example/quickstart.js Test](/img/arvinh/cli.png "example/quickstart.js Test")

在這邊先打岔一下，我們回到 Wit.ai 的 Dashboard 看一下，會發現 Inbox 上面有個小紅點？
Wit.ai 會在這個地方紀錄 User 傳送進來的句子，並且讓你在這邊操作它，也就是說，你可以在這邊利用 User 傳入的句子來 training 你的機器人！讓他直接從使用者身上學習！我覺得很棒的一個功能！

![Inbox](/img/arvinh/inbox.png "Inbox")

ok，鏡頭再轉回到程式碼。

已經知道怎麼實作函式了，那就接著把他跟 Messenger api 結合吧！

其實跟剛剛的 quickstart.js 比較不一樣的的地方在於，你必須記錄起來每一個 fb user 的 session，這樣 Wit.ai Bot 才會知道要回傳給哪個 FB user。
```js
// This will contain all user sessions.
// Each session has an entry:
// sessionId -> {fbid: facebookUserId, context: sessionState}
const sessions = {};

const findOrCreateSession = (fbid) => {
  let sessionId;
  // Let's see if we already have a session for the user fbid
  Object.keys(sessions).forEach(k => {
    if (sessions[k].fbid === fbid) {
      // Yep, got it!
      sessionId = k;
    }
  });
  if (!sessionId) {
    // No session found for user fbid, let's create a new one
    sessionId = new Date().toISOString();
    sessions[sessionId] = {fbid: fbid, context: {}};
  }
  return sessionId;
};
```

接著我們其實就只要修改先前的 `quickstart.js` 以及 先前實作過的 messenger API 的部分code即可。
因為我們的使用情境會讓 Bot 在接收訊息時，立刻先回傳文字，接著才會回傳查詢結果，而查詢結果則需要利用 Messenger API 傳送 **GenericMessage** 的結果，因此會需要兩種 return Method。
```js

const firstEntityValue = (entities, entity) => {
  const val = entities && entities[entity] &&
              Array.isArray(entities[entity]) &&
              entities[entity].length > 0 &&
              entities[entity][0].value;
  if (!val) {
    return null;
  }
  return typeof val === 'object' ? val.value : val;
};

// Our bot actions
const actions = {

	// Wit.ai 的 action 中，一定要實作的 send method，用來讓機器人說話
  send(request, response) {

  	const {sessionId, context, entities} = request;
    const {text, quickreplies} = response;
    // find out user id
    const recipientId = sessions[sessionId].fbid;
    if (recipientId) {

      // 這邊需要判斷要回傳的訊息是否為查詢結果
      // 若 context 中帶有 newsResult 那就是要回傳查詢結果
      // 因此就要呼叫 sendNewsMessagePromise() 來回傳 GenericMessage
      if (context.newsResult) {
        // fbBotUtil.sendNewsMessagePromise 這邊是 Messenger API 的相關實作
        return fbBotUtil.sendNewsMessagePromise(recipientId, context.newsResult)
          .then(() => null)
          .catch((err) => {
            console.error(
              'Oops! An error occurred while forwarding the response to',
              recipientId,
              ':',
              err.stack || err
            );
          });  
        } else {
          // 直接回傳普通文字
          return fbBotUtil.sendTextMessagePromise(recipientId, text)
            .then(() => null)
            .catch((err) => {
              console.error(
                'Oops! An error occurred while forwarding the response to',
                recipientId,
                ':',
                err.stack || err
              );
            });
        }  
    } else {
      console.error('Oops! Couldn\'t find user for session:', sessionId);
      // Giving the wheel back to our bot
      return Promise.resolve()
    }
  },

  // 我們自定義的 getNews action
  getNews({context, entities}) {
    return new Promise(function(resolve, reject) {
      var search_query = firstEntityValue(entities, 'search_query')
      if (search_query) {

      	// 這邊是去呼叫api
      	// fetchr 是我實作的一個小函式，利用 import.io 去抓 Yahoo news 的搜尋結果。
      	// 不是這篇重點我就先略過啦~
        fetchr(search_query, function(data) {
          context.newsResult = data;
          console.log('context newsResult', context.newsResult);
          delete context.missNews;
        });
        
      } else {
        context.missNews = true;
        delete context.newsResult;
      }
      return resolve(context);
    });
  },
};

```

實作完 Actions 的部分，記得到 router 裡面去增加 Wit.ai 的相關 Setting

```javascript
const wit = new Wit({
  accessToken: <Wit.ai TOKEN>,
  actions,
});
```
```javascript
router.post('/', function (req, res) {
  messaging_events = req.body.entry[0].messaging;
  for (i = 0; i < messaging_events.length; i++) {
    event = req.body.entry[0].messaging[i];
      sender = event.sender.id;
      var sessionId = findOrCreateSession(sender);
      if (event.message && event.message.text) {
        text = event.message.text;
        // Handle a text message from this sender
        wit.runActions(
          sessionId, // the user's current session
          text, // the user's message
          sessions[sessionId].context // the user's current session state
        ).then((context) => {
          // Our bot did everything it has to do.
          // Now it's waiting for further messages to proceed.
          console.log('Waiting for next user messages');
          // Updating the user's current session state
          sessions[sessionId].context = context;
        })
        .catch((err) => {
          console.error('Oops! Got an error from Wit: ', err.stack || err);
        })
      }
  }
  res.sendStatus(200);
});
```
上面這大串 code 其實就是接收到你在 Messenger POST 出去的訊息後，呼叫定義好的 `wit.runActions`，然後就可以讓 Wit.ai 幫你分析 User 的語句，並且回覆出去。

最後這邊放一下送出 我這邊用到的 fbBotUtil.sendNewsMessagePromise，也就是送出 messenger GenericMessage 的程式碼

[Messenger GenericMessage API Usage](https://gist.github.com/ArvinH/77e12ff1c4f8b0d77c2d15de36224ff5)

## Final Result

剛剛我們設定的語句，是不是就透過 Messenger 送出來了呢～

![messenger with Wit.ai](/img/arvinh/fbmessenger.gif "messenger with Wit.ai")

## One more thing

最後介紹一個方便的工具，ngrok。

ngrok 可以讓你把 localhost 轉成外網可以存取的網址，也支援 https，因此我們 Debug 就方便多了，不需要每次都把程式 Deploy 到遠端機器以後才能測試，log 也能直接在本機端終端機看到！

他的設定超簡單，到 [https://ngrok.com/download](https://ngrok.com/download) 把程式下載回來，並且執行 `./ngrok http PORT`

會出現如下畫面，連 https 的網址都有！這樣一來，facebook要求的 https webhook 就不成問題了，當然實際上運行還是要去用 SSL 憑證啦...

![ngrok](/img/arvinh/ngrok.png "ngrok")


參考資料

* [Wit.ai Doc](https://wit.ai/docs/)
* [打造我的聊天機器人！系列二：Wit-AI 的語句對話實作](https://medium.com/maker-cup/%E4%BD%BF%E7%94%A8wit-ai%E4%BE%86%E9%80%B2%E8%A1%8C%E8%AA%9E%E5%8F%A5%E5%B0%8D%E8%A9%B1-1ea692cb871e)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化