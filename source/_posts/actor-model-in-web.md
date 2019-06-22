---
title: 46 年老技術與 Web 的新火花 - Actor Model in Web
date: 2019-06-21 13:37:30
tags:
  - web
  - web worker
  - google chrome
  - architecture
---

## 前言

在今年的 Google I/O 2019 中，[@Surma](https://twitter.com/dassurma), [@Jake](https://twitter.com/jaffathecake) 與 [@Mariko](https://twitter.com/kosamari) 推出了一款能在低階的 feature phone 上運作順暢的 [web app - PROXX](https://github.com/GoogleChromeLabs/proxx)，其中利用到了 Web worker 來分擔許多 Browser main thread 中的工作，讓畫面渲染可以更順暢，也因此讓我注意到去年在 Chrome dev summit 2018 中的一支影片 - [Architecting Web Apps - Lights, Camera, Action!](https://www.youtube.com/watch?v=Vg60lf92EkM&list=PLNYkxOF6rcIDjlCx1PcphPpmf43aKOAdF&index=17&t=0s)，讓我眼睛為之一亮，裡面講到他們利用 Actor Model 來將關注點分離，不同的 Actors 各自負責不同的工作，彼此之間用共通的介面溝通，如此一來，除了將複雜的運算邏輯放入 Web worker 外，由於架構上的特性，理論上也能很方便的抽換 View 的實作方式，像是一套 web app，可以在不重寫所有邏輯的情況下，從 Vue 實作 view，轉換到以 React 實作。

聽起來非常合理，而有趣的是，這個 Actor Model 其實早在 46 年前就被提出，也有應用在 Web 領域上過，像是用 Scala 撰寫 的 Akka，或是在電信通訊上有名的 Erlang，但是在前端上似乎是第一次被提出來。

今天透過這篇文章來了解一下這個 46 年前就存在的觀念是什麼，而又能如何與 Web 前端整合呢！

## Actor Model 介紹

Actor Model 是一個關於 concurrent computation 的 conceptual model，在 1973 年最早被提出時，只是一個概念模型，用來抽象化並行處理程序的複雜程式，直到 1985 才被延伸出一個完整的 Actor model theory，有興趣可以到 [wiki](https://en.wikipedia.org/wiki/Actor_model) 查看。

而為什麼會需要提出 Actor Model 來處理 concurrent issue 呢？

歷史上的前因後果在這個影片中的前半段解釋得蠻清楚的 [[COSCUP 2011] Programming for the Future, Introduction to the Actor Model and Akka Framework](https://www.youtube.com/watch?v=k3ja9I6bGvU)

大體上是說，由於 CPU 的製程是有其極限存在，已經無法保證 Moore's Law 會持續下去，取而代之的是用多核心來提高處理器內的電晶體數量，透過多核心的並行處理來提高運算速度，而平行處理對於撰寫程式的我們來說，就是很複雜的一個存在，因為我們有 Shared mutable state，造成 race condition、dead lock 等等的 issue 需要避免，而 Actor Model 就是因應而生的解決方案之一。

Actor Model 主要有幾個特點：

* 輕量物件: Actor 是系統內的一個基礎單位，每個 Actor 只負責自己的工作，所以可以很輕量。

* 沒有 shared state：每個 Actor 各自管理自己的 state，跑在各自的 thread 上，不共享 memory 與 state。

* 透過 message 溝通：每個 Actor 會有一個 message queue，或稱作 mailbox，接收到的訊息會在此排隊等著依序執行。

一個 Actor 的架構略如下圖：

![Actor Model](/img/arvinh/actor-model.png)

每個 Actor 都有自己的 private state，別的 Actor 沒辦法直接更動你的 state，降低了因為 shared state 產生的問題。

我們拉遠一點來看，Actor 運行在不同的 Thread 中，彼此之間透過 Message 來溝通，收到訊息後各自決定要採取什麼樣的對應動作，而更改的都只會是自己的 private state，沒有 share memory，彼此之間是互相獨立的。

![Actor Model system](/img/arvinh/actor-model-system.png)

而在操作上，每個 Actor 只被允許做下面三種 operation：

1. 創建另一個 Actor
2. 傳送 Message
3. 指示該如何處理下一個 Message

前面兩種很好理解，但第三個就蠻有趣的了，舉例來說，假設有個 Counter Actor A，一直以來都是會把接受到的 Message 數字累加到自己的 counter state 中，但今天可以有另一個 Actor B 傳遞訊息跟 Actor A 說：『hey, 你這次先不用累加數字了，但是下一個傳進來的訊息，你要乘以 2 以後再放到計數器內喔』。

![Counter example](/img/arvinh/actor-model-counter.png)

這個三個特性合再一起時，有個很大的好處：容錯系統。

一個 Actor 對於他創建的 Actor 可以擁有 supervisor 的權限，可以告訴其管控的 Actor 說：『如果你 crash 了，至少死前丟給我個訊息跟我說』，然後當 Supervisor Actor 收到 Actor 死亡訊息時，可以再傳送 "restart" Message 去重啟 Actor，這樣的能力造就了一個 Self-healing systems：

![Fault Tolerance System](/img/arvinh/actor-model-fault-tolerance.png)

看到這邊會發現，雖然 Actor model 是因為 concurrent computation 而提出的，但其特性用在**分散式系統**上也是非常合適的，每個 Actor 跑在不同的 thread 上，當然也可以是不同的遠端環境上，並透過統一個 Message 介面溝通。

## 運用到 Web 前端上

在瀏覽器的世界中，也是存在有多種 Process 與 Context（推薦閱讀 @Mariko 的這篇[瀏覽器深入淺出介紹](https://developers.google.com/web/updates/2018/09/inside-browser-part1))，像是負責渲染的 UI Process，而在 DOM 外的 web worker 則是另一個獨立的 Worker Process，就想是一個縮小版的分散式系統，Worker Process 沒辦法直接存取 UI Process 所控制的 DOM 元件，相對的 UI Process 也無法直接呼叫 Worker Process 中所存在的函式。

照著這樣的邏輯梳理下來，Web 前端真的是蠻適合套用 Actor Model 的，就像我們有個 UI Actor 運行在 UI Process，同時負責控制狀態的 State Actor 運行在 Worker Process，彼此之間透過 Message 來溝通。

透過 Actor Model 來切割 UI 與 State 的關係，只要處理得當，理論上我們能夠讓一套相同商業邏輯的 Web app，輕易地從普通的 Web 轉換成 3D 版本，或是 Web VR 版本。

更棒的是，將關注點分離成各種 Actor 後，code splitting 變得更自然更容易了，每個 Actor 都能夠在需要的時候再載入即可。

## Web 應用的範例

說了這麼多，來看點實際的例子！

既然這個概念是 Google engineer 在他們的開發大會展示的，勢必有 Demo code 可以看，這套叫做 [actor-boilerplate](https://github.com/PolymerLabs/actor-boilerplate) 的 repository 包含了 [actor-helper](https://github.com/PolymerLabs/actor-helpers) 來幫忙 cover 了 Actor Model 與 Messaging system 的實作細節，讓我們能輕易的體驗 Actor Model 運用在 Web Frontend 的感覺。

我們做個簡單的基礎代謝率(BMR)計算器來感受一下。

先看一下成果：

![bmr calculator - actor model](/img/arvinh/actor-bmr-demo.gif)

UI 上就是簡單幾個 input 欄位而已，按下按鈕後，取出使用者輸入的值進行計算，最後顯示結果。這段流程我們可以拆分成兩個 Actor 來完成，分別是控制 UI 的 **UI Actor** 與更動 State 的 **State Actor**。（仔細看上方的 gif 的話，可以看到 State Actor 其實是運行在 Worker process 上的。

接著我們先來看看 UI Actor 長什麼樣子，關鍵的程式碼在下面這幾行（順帶一提，因為 action-boilerplate 本身就是以 TypeScript 實作，因此接下來範例也是 TypeScript）：

```js
import { Actor, lookup } from "actor-helpers/src/actor/Actor.js";
export default class UiActor extends Actor<Message> {
  private state = lookup("state");
  private resultEl = document.getElementById("result") as HTMLSpanElement;
  private getInputVal = () => {
    // gender
    const genderEl = document.getElementById("gender") as HTMLSelectElement;
    const genderVal = genderEl.value;
    // height, weight, age 依此類推...
    return {
      gender: genderVal,
      height: heightVal,
      weight: weightVal,
      age: ageVal,
    }
  };
  async init() {
    const calculateButton = document.getElementById(
      "Calculate"
    ) as HTMLButtonElement;
    calculateButton.onclick = () =>
      this.state.send({
        type: StateMessageType.CALCULATE,
        value: this.getInputVal()
      });
      //...略
  }
  async onMessage(msg: Message) {
    this.resultEl.textContent = `${msg.state.result}`;
  }
}
```

首先從 `actor-helpers` 中取出 Actor 物件來繼承，實作兩個主要函式：`init()` 與 `onMessage()`：

* init()：
  負責在初始時與 DOM 元件建立關係，綁定 event handler，當事件觸發時傳遞訊息給 State Actor。
* onMessage():
  就像是接收 `postMessage` 傳送的訊息一般，`actor-helpers` 幫我們串接好訊息溝通這段，在這 `onMessage()` 中，可以接收到其他 Actor 傳送過來的訊息。

在 `init()` 中的 `this.state` 是什麼呢？為什麼他可以 send message？

那是我們利用 `actor-helpers` 提供的 `lookup` 函式，將 State Actor 綁定到創建的私有變數上，這樣就能使用 `send` 來傳送訊息。

接著我們來看看 State Actor：

```js
import { Actor, lookup } from "actor-helpers/src/actor/Actor.js";
// lots of Type definitions...
// ...略
export default class StateActor extends Actor<Message> {
  private ui = lookup("ui");
  private state: State = {
    result: 0
  };

  async onMessage(msg: Message) {
    switch (msg.type) {
      case MessageType.CALCULATE:
        const {
          weight,
          height,
          age,
          gender
        }: BMRParams = msg.value;
        // Do the math
        this.state.result = bmr;
        break;
      case MessageType.RESET:
        this.state.result = 0;
        break;
    }
    this.ui.send({
      state: this.state
    });
  }
}
```

跟 UI Actor 大同小異，繼承 `Actor` 並利用 `lookup` 綁定 UI Actor 到私有變數上，在 `onMessage()` 中，根據接收到的 Message Type 來執行對應動作，並更改自身的 state，再將 Result Message 傳回給 UI Actor。

透過 `action-helps`，實作 Actor 變得方便許多（當然也是因為這是很簡單的例子...）。

不過還沒結束，Actor 都有了以後，我們要怎麼使用呢？

我們要準備兩個特殊的檔案，`bootstrap.ts` 與 `worker.ts`：

```js
import { hookup, initializeQueues } from "actor-helpers/src/actor/Actor.js";

import UiActor from "./actors/ui.js";

async function bootstrap() {
  await initializeQueues();
  hookup("ui", new UiActor());

  const worker = new Worker("worker.js");
  // This is necessary in Safari to keep the worker alive.
  // 相關討論可以看 repo 內的 issue https://github.com/PolymerLabs/actor-boilerplate/issues/13
  setInterval(() => {
    worker.postMessage("");
  }, 3000);
}
bootstrap();

```

顧名思義，`bootstrap.ts` 負責啟動整個專案，透過 `hookup` 載入 UI Actor，並且 new 出一個 worker 來運行我們的 State Actor。

`initializeQueues` 主要是在程式執行前，清空目前瀏覽器的 Message queue，該函式的程式碼也就一行 `await messageStore.popMessages("*");`。

而 `worker.ts` 的內容更簡單：

```js

import { hookup } from "actor-helpers/src/actor/Actor.js";

import StateActor from "./actors/state.js";

hookup("state", new StateActor());
```

就是 `hookup` State Actor！

最後在你的 html 上加入 `bootstrap.ts` 即可，這樣我們就完成了一個以 Actor Model 為基礎架構的 Web app！

[完整程式瑪可從此參考](https://github.com/ArvinH/actor-boilerplate)。

## 結論

Actor Model 雖然有了 `actor-helps` 的幫忙，實作上來說已經簡易很多，但任何操作都要透過 Message 的實作方式在前端應用上恐怕還是不好被接受，可能也因為如此，`actor-boilerplate` 其實在去年 Chrome dev summit 結束後似乎就沒再更新了 XD

不過我個人是蠻喜歡這個想法的，之後有機會再來真的實作一個不同 View 但使用同樣 State 的 Web app 看看！

另外，我也發現到，Web worker 雖存在已久，但是使用上的限制，讓一般我們在開發上很少會去考慮他，但這幾年看來，像是 AMP 等等的出現，發現 Google 似乎在推廣一個概念，就是 User experience first，Developer experience second，Developer 擅長處理複雜的事情，既然如此，我們應該專注在提供使用者最佳使用體驗上，而犧牲一點開發體驗。

如果你的 Web app 有複雜運算的需求，就試試看用 Actor Model 來將複雜的邏輯丟給在 Worker process 的 Actor 處理吧！

## 資料來源

1. [Lights, Camera, Action!](https://dassur.ma/things/lights-camera-action/)
2. [actor-boilerplate](https://github.com/PolymerLabs/actor-boilerplate)
3. [[COSCUP 2011] Programming for the Future, Introduction to the Actor Model and Akka Framework](https://www.youtube.com/watch?v=k3ja9I6bGvU)
4. [wiki - Actor Model](https://en.wikipedia.org/wiki/Actor_model)
5. [Inside Browser](https://developers.google.com/web/updates/2018/09/inside-browser-part1)

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化