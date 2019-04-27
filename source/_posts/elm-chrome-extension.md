---
title: 利用 Elm 製作 Chrome Extension
date: 2019-04-22 22:47:05
tags:
  - Elm
  - Chrome extension
---

## 前言

直接先來個成果圖：

![result](/img/arvinh/elm-extension.gif)

今年二月移轉陣地到日本來工作，搬到一個人生地不熟的地方，初始開銷少不了，除了交通、伙食、房租外，還需要添購許多傢俱。而當然，所有東西都是以日幣標價，就算 Amazon JP 已經非常方便的有簡體版的語系可以切換，價格上還是得以日幣顯示。

對我來說，還沒有辦法習慣以日幣來衡量物品的價值，像是我能很快知道一千日幣大約三百台幣，但看到一個東西標價兩萬九日幣，我沒辦法很快速的理解這東西是多貴或多便宜，腦袋還是會想把它轉成台幣來衡量。

所以就造成我在 Amazon JP 或是 Nitori 的網站挑選商品時，常常要複製價格，然後開啟另一個 tab 來搜尋日幣轉換台幣的網站，再填入轉換。

一次兩次還好，第三次就無法忍受了，引用 "Refactoring — Improving the Design of Existing Code" 這本書中的一句話：*"Three strikes, then you refactor."*

稍微找了一下 Chrome web store，並沒有看到適合的 extension，所以決定自己打造一個。

而剛好在前幾週參加了同事主辦的 Meetup，其中有個 Topic 是介紹 Elm 這個語言，以前雖然也聽過介紹，但這次聽到實際用在 Production 的案例後，提高我不少興趣，加上適逢公司的 hackday，索性就利用這機會來練習一下，用 Elm 來製作我想要的 Chrome Extension！

今天就用這篇文章記錄一下使用 Elm 製作 Chrome Extension 的方法與心得，不過畢竟是 hackday 作品，功能尚需完善就是。

## 什麼是 Elm？

![Elm Website](/img/arvinh/elm-official-site.png)

不知道有沒有人跟我一樣，之所以知道 Elm，是因為 Redux 的作者說他受其啟發？

跟 Redux 作者一樣，Elm 的作者 - Evan Czaplicki 也是個年輕奇才，Elm 是他在哈佛的論文，推薦大家看一下他 [2012 年的演講影片](https://www.infoq.com/presentations/Elm)，可以了解他為何創造這門語言，聽聽作者本人親口介紹。

Elm 是一個強型態的函數式語言，透過編譯，將最終程式碼轉化為 JavaScript，因此執行在任何 Web 平台。主要有以下特點：

* No Runtime Exceptions
  * 在編譯期間，Elm 會利用 Type inference 偵測各種狀態，有任何型別上的錯誤都會被抓出來，更棒的是，Elm 提供的 hint 很人性化，例如：
  ![error hint](/img/arvinh/elm-error-hint.png)

* Great Performance
  * 如同 React 一般，Elm 也有自己的 Virtual DOM 實作，根據官網所說，由於 Elm 的所有值都是 immutable，有助於他們能編譯出最佳化的 JavaScript，讓 runtime 變快。
  ![benchmark](/img/arvinh/elm-benchmark.png) *source: [elm-lang.org](https://elm-lang.org/)*
  
* Enforced Semantic Versioning
  * Elm 強大的 type system 讓他能自動偵測是否有 API 的更動，並根據你 release 時與上一版本的差異，進行 Semantic Versioning，讓你不會不小心因為錯誤的 PATCH release 造成別人的困擾！

* Small Assets
  * Elm 宣稱其 Optimize 的編譯，能產生比 React 16.4 更小的 Assets。*source: [elm-lang.org](https://elm-lang.org/)*

* JavaScript Interop
  * 如同 React 一樣，你也可以只在一個小小的 div 上掛載 Elm application，藉此來測試 project 中的某一個小部分，採用上可以循序漸進，不會有太大風險，並且由於編譯成 JavaScript，Elm 能方便的與一般的 JavaScript 溝通。（基本上我也是看到這點才有信心 Elm 能拿來製作 Chrome extension 的）

## 先偷看一下 Elm 的程式碼到底長怎樣

```elm
view : Model -> Html Msg
view model =
    Html.div
        [ Html.Attributes.class "Content"
        ]
        [
          Html.text ("Hello Elm!")
        ]
```

這是一個簡單的 Elm view，第一行是描述這個 View 所接收的參數型別，以及回傳的參數型別，第二行後才是實際的程式碼。

看起來非常畸形。

一開始很難理解，沒關係，我寫完這個 extension 以後也還是覺得很難理解。

畢竟會需要花不少時間習慣連 HTML 都是函數的概念，裡面的 `HTML.div` 就是一個函數，他接受兩個陣列，一個用來定義 div 的 attribute，一個用來描述其 child。在後面的實作範例內，我們會看到更多類似的語法。

## Elm 基本安裝使用

Elm 的安裝很簡單，官網提供各平台的 [installer](https://guide.elm-lang.org/install.html)，安裝完後就能在你的 Terminal 中使用以下幾個指令：

**`elm repl`** - 顧名思義， Elm 的 repl：

![elm-repl](/img/arvinh/elm-repl.png)

**`elm reactor`** - 類似 react 的 create-react-app，會快速幫你建立一個基礎的 Elm Architecture。

**`elm make`** - elm 的 compile 指令，將你的 Elm code 編譯成 HTML 或是 JavaScript：

```bash
elm make Main.elm --output=main.js
```

**`elm install`** - 類似 `npm install`，幫你下載所需的套件。所有的 Elm packages 都在 [package.elm-lang.org](https://package.elm-lang.org/) 中。而你專案中 packages 的 dependencies 會在自動紀錄在 `elm.json` 中。

## Elm Architecture

現在的前端開發者對於 Elm 的架構應該都會覺得不陌生，因為就很清楚的分為 Model、View、Update：

* **Model** - 你的 application 的 state
* **Update** - 更新 state 邏輯的部分
* **View** - 綁定 State 更新 HTML

一個基本的 Elm 程式大概就是包含這三大區塊：

```elm
import Html exposing (..)
-- MODEL
type alias Model = { ... }
-- UPDATE
type Msg = Reset | ...
update : Msg -> Model -> Model
update msg model =
  case msg of
    Reset -> ...
    ...
-- VIEW
view : Model -> Html Msg
view model =
  ...
```

在接下來的範例內，也是遵循這樣的方式撰寫。

## 學習資源

既然語法這麼困難，要學習的話，勢必要有完善的 Docs 或是範例參考，但可惜的是，雖然 Elm 從 2012 出來到現在也將近七年了，使用人數還是小眾，要找尋範例或是教學都蠻困難的，推薦的方式是先去[官網閱讀 guide line](https://guide.elm-lang.org)，理解基本概念與最新版本的 API，並大致瀏覽一下語法，接著到 [Beginning Elm](https://elmprogramming.com/) 釐清一下不懂的部分，因為這網站將蠻多觀念以圖像化的方式說明，我覺得會比官方網站的 guide 好懂。

不過要注意的一點是，[Beginning Elm](https://elmprogramming.com/) 的內容基本上是 Elm 0.18 的版本，並非現在最新的 0.19，所以在語法與一些 core lib 的用法上會不一樣，但觀念的理解還是共通的，所以能交互參照。

這也是我在實作 Elm 遇到的一個阻礙，網路上大多的教學集中在 2017，當時還是 0.18 版本，因此許多範例現在都沒辦法正常運作，除非你降版。

## 開始實作！

無論如何，頭洗下去了還是得繼續實作。

有實作過 Chrome extension 的人應該都知道，基本的三大元素就是：`background.js`、`content.js` 與 `popup.html`，其彼此之間環境互相獨立，透過 `postMessage` 來溝通。

而既然 Elm 能編譯成 JavaScript，我就想我應該可以分別撰寫 Elm application 然後 compile 成 `background.js`，`content.js`，這做法可能會有點奇怪，既然要用 Elm，你又寫 JavaScript？

但我覺得這樣反而比較能在漸進式的學習 Elm，一些較困難理解的部分就還是交由 JavaScript 處理。

有了想法後開始搜尋範例，雖然新版的範例不多，但好在還是能從舊版中找到符合我心中所想的範例，只是得花點時間 upgrade 成新版。

[elm-chrome-extension](https://github.com/danneu/elm-chrome-extension) 是我找到最符合我預期的範例。

而這是我翻新成 0.19 後的版本 [elm-chrome-extension 0.19 版](https://github.com/ArvinH/elm-chrome-extension)。

### 基本架構

![structure](/img/arvinh/elm-ext-structure.png)

我們這邊將 `background.js` 與 `content.js` 當作不同的 Elm application 來撰寫，彼此之間再透過 Elm 特殊的 JavaScript Interop 來溝通（後面會講到）。

由於個別當作一個 application，就用各自的 webpack 來幫忙 compile 跟打包。

`Model.elm` 則是單純用來定義我們整個 Extension 的 **State**。

最後三個 Elm application 編億完的結果都會放入 `dist` 中，也就是我們最後的成品。

另外前面沒有提到的一個資料夾 `elm-stuff`，其功用就像 `node_modules` 一樣，當你 `elm install` 完後，Elm 會將 packages 放入其中。

而 `elm.json` 則是類似 `package.json` 加上 `package-lock.json` 的存在：

```json
{
    "type": "application",
    "source-directories": [
        "./background",
        "./content",
        "./common"
    ],
    "elm-version": "0.19.0",
    "dependencies": {
        "direct": {
            "elm/browser": "1.0.1",
            "elm/core": "1.0.0",
            "elm/html": "1.0.0",
            "elm/json": "1.1.3",
            "myrho/elm-round": "1.0.4"
        },
        "indirect": {
            "elm/time": "1.0.0",
            "elm/url": "1.0.0",
            "elm/virtual-dom": "1.0.2"
        }
    },
    "test-dependencies": {
        "direct": {},
        "indirect": {}
    }
}
```

可以看到 Elm 是用 exact versions，可以保證 reliable builds。而 dependencies 中的 direct 就是所有你能直接在 Elm code 中 import 的 package，而那些 package 所各自引用的其他 lib 則會自動被載入到 indirect 中。

值得注意的是，雖然我們整個 project 用了三個 Elm application，但我們是共用同一個 elm.json 來管理 packages，因此在 `source-directories` 的地方我們羅列三個 source 來源。

### 專案實作方式概略

這個 Extension 的功能很單純，流程如下：

使用者 Select 了某段數字後，就會由 `Content.js` 將選取的數字傳遞給 `Background.js` 去處理，而 `Background.js` 就會負責拉取匯率 API，並將使用者選取的數字進行換算，接著將結果更新到 `Model` 上，再經由 `Model` 的變動去觸發 `Content.js` 進行 `View` 的更新。

我們一步一步來看，主要會 Focus 在如何更新 Model，以及如何讓 Elm(計算邏輯) 與 JavaScript(頁面 Dom 與 Chrome API) 互相溝通。

#### Model

首先，我們先從最簡單的 Model 看起：

```elm
module Model exposing (Model)
-- This is the model in common among all of our apps
type alias Model =
    { result: Float, selectedContent: Int, exrateTWD: Float, exrateJPY: Float }
```

line 1 就是 Elm 中載入 package 的方式，而在我們的 application 中，Model 內容很單純，主要是記錄型別為 Float 的結果（result)、Int 的 selectContent、Float 的 exrateTWD 與 exrateJPY。

然後透過 webpack loader 來幫忙編譯：

```js
module.exports = {
  module: {
    rules: [
      {...},
      {
        test: /\.elm$/,
        exclude: [/elm-stuff/, /node_modules/],
        use: ['elm-hot-webpack-loader', 'elm-webpack-loader?verbose=true'],
      },
    ],
  },
}
```

### Main.elm in Background.js

在 Background 資料夾中，我們有一個 `index.js` 與 `Main.elm`，`index.js` 負責與 `content.js` 溝通和處理 chrome api 相關的 JavaScript；`Main.elm` 則是負責計算匯率以及更新 Model。

index.js 會與 Main.elm 互相溝通，Webpack 會將兩者打包成 `background.js`。

但 JavaScript 怎麼與 Elm 溝通呢？容許我從 [Beginning Elm](https://elmprogramming.com/sending-data-to-javascript.html) 中借張圖：

![How JS interact with Elm](/img/arvinh/elm-js-interpo.png)

從這張圖可以很清楚看到 Elm runtime 是怎麼跟外部溝通，又是怎麼與我們程式邏輯互動。

透過 `Command` 我們可以對 Elm runtime 下達指令去執行 side effect，像是 HTTP request 等等；Elm runtime 則藉由 `Subscription` 與 `Message` 將 side effect 結果傳遞回 application 本身。

透過 `Message`，Elm runtime 與我們的 application 可以知道要執行甚麼動作，包含 Update State（Model）或是更新 View。

而若是需要與外部 JavaScript 溝通，則有 `Ports` 提供橋樑，在 JavaScript 中，能使用類似 `postMessage` 的方式傳遞資料。

來看看 background 中的 Main.elm：

```elm
-- ... 省略 import packages
-- PORTS FROM JAVASCRIPT
port selected : (Model -> msg) -> Sub msg
-- PORTS TO JAVASCRIPT
port broadcast : Model -> Cmd msg
-- MODEL
init : Flags -> ( Model, Cmd Msg )
init flags =
    ( {
        selectedContent = flags.selectedContent,
        exrateTWD = flags.exrateTWD,
        exrateJPY = flags.exrateJPY,
        result = flags.result
     }
    , Cmd.none
    )
type Msg
    = NoOp
    | Select Model
update : Msg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        NoOp ->
            ( model, Cmd.none )
        Select data ->
            let
                nextModel =
                    { model | result = (toFloat data.selectedContent) * (data.exrateTWD / data.exrateJPY) }
            in
            ( nextModel, broadcast nextModel )
subscriptions : Model -> Sub Msg
subscriptions model =
    selected (\newModels -> Select newModels)
type alias Flags =
    {
      selectedContent: Int,
      exrateTWD: Float,
      exrateJPY: Float,
      result: Float
    }
main : Program Flags Model Msg
main =
    Platform.worker
        { init = init
        , update = update
        , subscriptions = subscriptions
        }
```

background 的 Main.elm 中，主要有幾個區塊：

* `port` keyword：定義用來給外部 JavaScript 呼叫的函數，以及 Elm 要傳遞資料給 JavaScript 時呼叫的函數

* `init` 函數：我們定義 init 函數，初始化 application 的 Model (State），`Flags` 是 Elm 中特殊的型別，外部 JavaScript 可以透過 Flags 在 Elm 的 init 階段傳遞初始 State 資料。如同上面範例中，init 函數接收一個 `Flags` 參數，並將其 assign 到新的物件當中，並且回傳，而這物件就是（也必須是）Model。

* `subscriptions`：當外部 JavaScript 呼叫我們在 port 中定義的函數時，Elm 會透過 subscriptions 來處理回應，基本上都會是呼叫一個函數，而該函數會 Trigger `update`，丟入 一個 Message 去更新 Model，以上面例子來說，JavaScript 會傳進新的 Model，而 Elm subscriptions 將新的 Model 以及我們指定的 Message 傳遞給 `update` 去真正的將 Model 更新。

* `update`：update 相對單純一點，接收 Message 與 Model 型別的物件，根據 Message 的內容去判斷要進行什麼更新。在上面例子中，我們接收到 `Select` 這個 message，代表收到 JavaScript 傳來的新 Model（也就是使用者選取的數字），而 Model 中有要轉換的金額，以及台日幣的匯率差，我們在這邊進行主要的邏輯運算，將結果產生成新的 Model，透過呼叫 `broadcast` 這個 定義在 port 中的 Command，將其回傳回 JavaScript。

* `main`：是 Main.elm 這個程式的主要進入點，這隻 Elm 程式並沒有參與到 View 的部分，因此我們使用 `Platform.worker` 函數實作，將定義好的 `init`, `update` 與 `subscriptions` 傳入。

篇幅有限實在無法將每個 API 都詳細介紹，建議參照[官網 Guide](https://guide.elm-lang.org/) 了解其詳細內容。

### index.js in Background.js

JavaScript 的部分就簡單一些，重點在於初始化 State：

```js
import { Elm } from './Main.elm';
let currState = {
  selectedContent: 0,
  exrateTWD: 0.0,
  exrateJPY: 0.0,
  result: 0.0
}
const app = Elm.Main.init({
  flags: currState
});
```

在 `index.js` 我們載入 `Main.elm`，並呼叫其 `Main.init` 函數，傳入 flags 物件來初始化 state。

接著，subscribe Elm 的 ports `broadcast`，當 Elm 傳遞訊息過來時，我們透過 chrome 的 `postMessage` 傳遞給 `content.js`。（這邊的 port 是 `chrome.runtime.onConnect` 的 port，細節請看 [source code](https://github.com/ArvinH/Elm-ChromeExt/blob/master/background/src/index.js#L15-L41) ）

```js
function broadcast(state) {
  currState = state
  for (const port of listeners) {
    port.postMessage(state)
  }
}
app.ports.broadcast.subscribe(state => {
  broadcast(state)
})
```

最後這邊我有點偷吃步，由於透過 Elm 發送 HTTP request 後的結果與我目前的 model 之間要如何整合，我在一天的 hackday 中實在沒有研究出來，所以省去那段，讓 fetch 匯率資料的部分由 JS 完成，最後再透過 `port` 的 `selected` 將 API result `send` 給 Elm：

```js
chrome.runtime.onMessage.addListener((request, sender) => {
  if (request.kind === 'selected') {
    const selectNum = request.selectedContent.replace(',', '');
    fetch('https://tw.rter.info/capi.php')
      .then(function(response) {
        return response.json()
      })
      .then(function(myJson) {
        const {
          USDJPY: { Exrate: ExrateJPY },
          USDTWD: { Exrate: ExrateTWD }
        } = myJson
        app.ports.selected.send({ result: 0.0, exrateTWD: ExrateTWD, exrateJPY: ExrateJPY, selectedContent: +selectNum})
      });
  }
})
```

### Main.elm in Content.js

處理好 Background.js，接著就是 Content.js。

Content 中的 `Main.elm` 主要負責接收到更新的 Model，然後將其更新到 DOM 上：

```elm
-- 省略 import packages
-- PORTS FROM JAVASCRIPT
port onState : (Model -> msg) -> Sub msg
init : Model -> ( Model, Cmd Msg )
init model =
    ( model
    , Cmd.none
    )
type Msg
    = NoOp
    | NewState Model
update : Msg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        NoOp ->
            ( model, Cmd.none )

        NewState newModel ->
            ( newModel, Cmd.none )
view : Model -> Html Msg
view model =
    Html.div
        [ Html.Attributes.class "Content"
        ]
        [
          Html.text ("It's "),
          Html.div
            [ Html.Attributes.class "InnerContent"
            ]
            [
                Html.div [][
                    Html.text (round 2 model.result)
                ],
                Html.div
                    [ Html.Attributes.class "units"
                    ]
                    [
                        Html.text (" TWD")
                    ]
            ]
        ]
subscriptions : Model -> Sub Msg
subscriptions model =
    onState NewState
main : Program Model Model Msg
main =
    Browser.element
        { init = init
        , update = update
        , view = view
        , subscriptions = subscriptions
        }
```

聰明的讀者看到這段 code 應該會發現，這邊的 `port`, `init` 與 `subscriptions` 基本上形式與 Background 的一樣，只是更簡單一點，當 subscription 收到 JavaScript 呼叫的 `onState` 函數時，我們傳遞 `NewState` Message 給 `update`，而 `update` 就只是單純的回傳新的 Model。

唯一差別在於 `view` 與 `main`。

在 Elm 中，view 會接受 Model 為參數，並回傳 HTML DOM，在範例中，我們產生了三層的 div，第三層有兩個 div 並行，分別呈現轉換後的匯率數字，以及單位，這邊我還用了一個 elm 的 package 來 `round` 數字到小數點後兩位。

而由於 Content 的 `Main.elm` 有跟 Browser 互動（產生 HTML），所以需要的不是 `Platform.worker` 而是 `Browser.element`，他會將 View 也綁定進去。（一樣，詳情請參照[官網 Guide](https://guide.elm-lang.org/) ）

### index.js in Content.js

Content 中的 JavaScript 就更單純了，基本上就是 `createElement` 出一個 div，並且設定 `mouseup` listener，讓使用者選取文字後，能夠 `sendMessage` 給 background.js，並且設定好 Elm DOM 要掛載的 Real Dom 位置與 style (index.css)：

```js
require('./index.css')

const mountNode = document.createElement('div');
document.body.append(mountNode);
import { Elm } from './Main.elm';
let app;
document.addEventListener('mouseup', () => {
  // 利用 window.getSelection() 抓取使用者選取的數字
  const selectedObj = window.getSelection();
  const selectedContent = selectedObj.toString();
  //...
  // 這邊忽略的 code 主要在是：
  // 計算 Elm 產生的 DOM 該插入到頁面中的哪個位置
  // 判斷何時該讓 DOM 消失
  //...
  if (selectedContent && selectedContent!== '') {
    chrome.runtime.sendMessage({ kind: 'selected', selectedContent });
  }
});
const port = chrome.runtime.connect({ name: 'broadcast' });
port.onMessage.addListener(state => {
  if (!app) {
    app = Elm.Main.init({
      node: mountNode,
      flags: state
    });
    return;
  }
  app.ports.onState.send(state);
})
```

在 `port.onMessage` 的 listener 中，我們會監聽到由 Background.js 傳遞來的初始 State，然後才 init Content 這邊的 Elm application，並將 Background.js 傳來的 state 一起傳入，若後續有更新 state，則透過 `port` 的 `onState` 來更新。

基本上這樣一來，整個 Extension 的工作就完成了，但省略了不少細節，完整程式碼與其他細節只好請大家直接看原始碼會更清楚一點： [source code](https://github.com/ArvinH/Elm-ChromeExt)

### 如何 Debug

在用 Elm 製作 Chrome extension 的過程中，Debug 的方式其實跟一般用 JS 開發一樣，都是透過 inspect 從 browser 中到 `background.js` 與 `content.js` 下中斷點，但是麻煩的是，在這邊的 `background.js` 與 `content.js` 都有一大部分是 Elm compile 出來的 JS，是沒有類似 source map 的東西能讓你直接在 Elm code 內 Debug 的，所以除錯起來真的不是很方便，之後若有研究到方法再來更新。也歡迎大家給予指導！

## 結論與心得

Elm 的確是蠻優雅的語言，但不是能在短時間內完美駕馭的...尤其是 Debug 的難度以及文件範例的相對稀少，都讓人容易退卻，但我還是蠻開心能利用這次機會親手玩玩 Elm，並且製作了一個我用得上的工具，當然還有一些需要改善的部分，但已經堪用了。希望之後還能有時間繼續研究！有興趣的讀者或許能一起加入他們的 [Community](https://elm-lang.org/community) 討論！

## 資料來源

1. [elm-lang](https://elm-lang.org)
2. [Beginning Elm](https://elmprogramming.com)
3. [elm github](https://github.com/elm)
4. [Elm 作者 ｀2012 年的演講影片](https://www.infoq.com/presentations/Elm)
5. [Elm——函数式前端框架介绍](https://shenlvmeng.github.io/blog/2017/06/19/elm-introduction/)

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化