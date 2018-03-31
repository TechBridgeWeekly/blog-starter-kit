---
title: 淺談 React Fiber 及其對 lifecycles 造成的影響
date: 2018-03-31 10:19:29
tags: react,fiber,react lifecycle
author: huli
---

# 前言

雖然說從以前就一直聽到 React 要把內部的 reconciler 換成一個叫做 Fiber 的東西，但從沒仔細研究過，也不知道這樣子的改變會對上層造成什麼影響。

真正開始比較深入理解，是在某一次使用 Redux Form 時踩到一個相關的 bug，才知道 React 自從正式改用 Fiber 之後，其實對上層也有一些改變。

這篇的標題「淺談」不是騙你的，這篇真的很淺，我不會談到 Fiber 底層的運作（因為我也還沒認真研究過），只會用白話文跟你說 Fiber 大概是怎樣，是為了解決什麼樣子的問題而誕生。

# 萬里之行，始於 Bug

每一次能踩到 Bug 的機會，我都會好好把握。

為什麼？因為這是一次強迫你學習的機會。Bug 解不掉，你就沒辦法繼續下去，所以為了要解決 Bug，你必須去探究原因，必須知道這個問題為什麼產生，以及應該要怎麼解決。

當然，你也可以直接從 Stack Overflow 找答案然後複製貼上，覆蓋這張牌結束這回合。可是工作久了你會發現，不是所有問題都可以從那上面找到解答。

舉例來說，我一年前寫的[我遇過的最難的 Cookie 問題](https://blog.techbridge.cc/2017/03/24/difficult-problem-of-cookie/)對我來說就是一個很好的學習機會。

好，那這次我又是遇到什麼 Bug 呢？

我們公司的產品有用到redux-form，而問題是這樣的，我有兩個頁面，都用了同一個 component，叫做`FormBlock`好了。

我先去 A 頁面，再去 B 頁面，再回來 A 頁面，我的 redux-form 的 validation 就失效了，在表單 submit 的時候並不會執行 validation。

那時候搜到了幾個相關的 issue，但還是想自己查個清楚，就跑去找了 redux-form 的原始碼，研究了幾個小時終於找到出問題的地方。

在 redux-form [執行 validation](https://github.com/erikras/redux-form/blob/5c13be079476cb0d0430ca88fd3e1abbd09e674a/src/selectors/isValid.js#L37) 的時候，會先檢查 fields 是不是有被註冊過，如果沒被註冊的話，就直接回傳`true`，不會進行任何驗證，我自己加了幾個 console.log 之後，發現問題就是出在這邊，field 沒有被註冊到。

接著就來找一下是在哪邊註冊的，發現在`componentWillMount`的時候，會 dispatch 一個 action 來註冊所有的表單欄位（`REGISTER_FIELD`）。

然後在`componentWillUnmount`的時候，redux-form 會 dispatch 一個 action 叫做`DESTROY`（[相關程式碼](https://github.com/erikras/redux-form/blob/5c13be079476cb0d0430ca88fd3e1abbd09e674a/src/createReduxForm.js#L556)），把所有註冊的 field 給清掉。

到目前為止，一切看似都很合理。我在離開 B 頁面的時候，觸發`FormBlock`的`componentWillUnmount`，取消註冊所有的 field，在進入 A 頁面時，觸發`FormBlock`的`componentWillMount`，把所有的 field 重新註冊回來。

可是如果你打開 redux-devtool，會發現順序跟你想像中不太一樣：

![](/img/huli/fiber/form.png)

咦？怎麼會先註冊再刪除？而且因為刪除了，所以驗證就失效了，不會執行任何驗證的邏輯。

再仔細找了一下相關的資料，看到這篇 [Browser back button not working with react-router@4.0.0-beta.7 and react@16-alpha.4](https://github.com/facebook/react/issues/9214#issuecomment-287763538) Issue 以及 Redux 以及 React 的開發者 gaearon 在下面的回答：

> In React 15, if A is replaced by B, we unmount A, and then create and mount B:
  
> 1. A.componentWillUnmount
2. B.constructor
3. B.componentWillMount
4. B.componentDidMount
  
> In Fiber, we create B first, and only later unmount A and mount B:
  
> 1. B.constructor
2. B.componentWillMount
3. A.componentWillUnmount
4. B.componentDidMount

在 React 16 以後，由於這樣子的順序改變，導致了上面 redux-form 的 lifecycle 執行順序跟預期中不符，也間接導致了我開頭所說的那個 Bug。

到了這邊，問題產生的原因一路從 redux-form 本身追到了 React，又更細節的追到了 Fiber，看來，沒辦法再繼續逃避 Fiber 了。

先一併奉上跟 redux-form 以及執行順序相關的其他參考資料，再讓我們好好看看 Fiber。

1. [Re-mounting a Field component erases the field-level validation function](https://github.com/erikras/redux-form/issues/3566)
2. [Ordering of componentWillMount/Unmount in React 16](https://github.com/facebook/react/issues/12233)
3. [Asynchronous ComponentWillUnmount in React 16](https://github.com/facebook/react/issues/11106)

# Fiber 到底是什麼？

要瞭解一個新的東西，最快的方式就是回答以下問題：

1. 它是拿來解決什麼問題的？
2. 解決方法是什麼？

只要能了解這兩個問題，就能對這個新的東西有初步的概念，儘管你還是不知道實作細節，但至少你知道它帶來的影響及改變是什麼。

我們先來看一下一直存在於 React 的一個問題。

假設你現在有一個超級多功能的 App，有著超級多的 Component，然後你改變了最上層的 Component（假設它叫`<App />`） 的 state。

因為 state 變了，所以就會來執行這個`<App />`的 render function，然後執行`App`底下的 component 的 render function，就這樣一直往下執行下去，直到碰到最底層為止。

你如果去看 call stack，就會發現這個 call stack 超大一個：

![](/img/huli/fiber/call.png)

（圖片來源：[React Fiber現状確認](http://blog.koba04.com/post/2017/04/25/a-state-of-react-fiber/)）

這樣會造成什麼問題呢？因為你的 call stack 太深而且東西太多，再加上這個過程又是不能被中斷的，會導致 main thread 被 block 住，在這時間之內你做任何事情，瀏覽器都不會有反應。

簡單來說呢，就是因為要做的事太多，所以 main thread 就會 block 住了，這就是 React 在效能上會碰到的一個問題。

到這邊，我們已經回答第一個問題了，Fiber 就是為了解決這個問題而產生的解法。接著我們來回答第二個問題：解決方法是什麼？

> 既然問題的成因是：「要做的事情太多又不能中斷」，那我們只要發明一個「可以中斷」的機制就好啦！不要一次全部更新，而是增量更新（incremental rendering），就可以解決這個問題了！

比起原本的一次性更新，假如我們能夠把要更新的工作切成一個個小的工作，每次只執行一個小工作，那這樣 main thread 就不會被 block 住了，因為每個小工作之間都可以有空檔去做別的事情（響應使用者的點擊、繪製新的畫面等等）。

就像下面這張示意圖一樣，每次完成一點點的工作，而不是一次完成全部的：

![](/img/huli/fiber/cartoon.png)

（圖片來源：[Lin Clark - A Cartoon Intro to Fiber - React Conf 2017
](https://www.youtube.com/watch?v=ZCuYPiUIONs)）

好了，你已經知道什麼是 Fiber 了，這就是 Fiber。每一個小工作就叫做 Fiber，而 Fiber 在英文裡面是纖維的意思，所以又有人把這個機制稱作「纖程」。

或是換個角度想，原本的問題是因為程式裡面這樣子一層層執行 render function 的方法是透過 call stack，每次 call 一個 function 就把一個新的任務丟到 stack frame 去，可是這樣子的機制會導致任務無法中斷。

於是 Fiber 就實作出了 virtual stack frame，簡單來說就是自己用 js 再模擬出一個 call stack 的感覺，但好處就是自己有完全的掌控權，而不是被 js 的運行機制給綁住。

再幫大家重新整理一次，沒有 Fiber 之前，你要更新的時候都是「一次性」的更新，中間無法中斷，導致 main thread 在這期間會被 block 住。

有了 Fiber 這個機制之後，我們把一個大更新切成很多塊小的更新，每次只更新一點點，這樣子在更新的空檔 main thread 就能去做其他事情，而不會被綁住。

聽起來十分美好，問題迎刃而解，可是副作用是什麼呢？

## Fiber 所帶來的改變

把核心換成 Fiber 之後，是要付出一些代價的。在 Fiber 裡面的工作其實分成兩個階段：

1. render/reconciliation
2. commit

簡單來說呢，第一階段就是找出需要改變的部分，而第二階段是真正的把這些改變應用到 DOM 上面去。第一階段是可以被中斷，也可以被重新執行的，而第二階段跟以前一樣，必須一口氣做完。

而這兩個階段也對應到不同的生命週期：

### 第一階段
* componentWillMount
* componentWillReceiveProps
* shouldComponentUpdate
* componentWillUpdate

### 第二階段
* componentDidMount
* componentDidUpdate
* componentWillUnmount


因為第一階段是可以中斷並且之後再重新執行的，所以會導致在第一階段裡的這些函數，有可能被 call 很多次。

![](/img/huli/fiber/life.png)

（圖片來源：[Lin Clark - A Cartoon Intro to Fiber - React Conf 2017
](https://www.youtube.com/watch?v=ZCuYPiUIONs)）

所以，假設你之前習慣在`componentWillMount`裡面就呼叫 API 拿資料的話，就會導致你 call 了不只一次的 API，會浪費一些頻寬，要改變的話就要把這些 code 移到`componentDidMount`去，就只會保證被 call 一次而已。

總之呢，自從內部機制改成 Fiber 之後（從 React 16 開始，所以如果你是用 16 以上的版本，已經是 Fiber 了），React 的生命週期函數被呼叫的次數跟方式會跟以前不太一樣。

除此之外就是我開頭提的那個順序的不一樣，這點也是值得注意的一個部分。雖然看起來不是什麼大問題，但如果不知道這點的話可能會生出一些莫名其妙的 Bug。

## React 的未來

React 16.3 在昨天[正式發佈](https://reactjs.org/blog/2018/03/29/react-v-16-3.html)了，伴隨而來的是正式的 context API 以及 lifecycle 的改變。

隨著 Fiber 的正式上線，未來可以期待會有更多令人興奮的新功能。比如說在[Sneak Peek: Beyond React 16](https://reactjs.org/blog/2018/03/01/sneak-peek-beyond-react-16.html)這篇提到的`time slicing`，把整個 App 的體驗變得更順暢。

而[Update on Async Rendering](https://reactjs.org/blog/2018/03/27/update-on-async-rendering.html)這篇文章也提到了非同步渲染的進展。

自從內部的機制改成 Fiber 之後，就讓 async rendering 得以發揮最大的效能。

但為了 async rendering，是需要付出一些代價的。原本的 lifecycle API 在這種場景底下可能會有一些問題，官方有給出許多常見的例子，也包含我們上面所說到的，`componentWillMount`會被呼叫多次的問題：

（忽略原本的範例程式碼，但大意就是在`componentWillMount`裡面 call API）
>The above code is problematic for both server rendering (where the external data won’t be used) and the upcoming async rendering mode (where the request might be initiated multiple times).

>The recommended upgrade path for most use cases is to move data-fetching into componentDidMount

對於 async rendering，會引起問題的是以下三個生命週期：

1. componentWillMount
2. componentWillReceiveProps
3. componentWillUpdate

這三個 lifecycle 會在 React 17 裡面被拿掉（如果你還是想用的話可以加上`UNSAFE_`，例如說改成`UNSAFE_componentWillMount`就一樣可以用），但既然都說是 UNSAFE 了，沒有理由繼續使用下去。

舊的不去新的不來，在最新發佈的 16.3 中，引入了兩個新的 lifecycle 來解決上面的那些問題：

1. getDerivedStateFromProps
2. getSnapshotBeforeUpdate 

第一個很顯然是要來取代`componentWillReceiveProps`的，而第二個是拿來取代`componentWillUpdate`的。或其實有些場景底下，用`componentDidUpdate`也可以取代原本那兩個生命週期。

至於最前面所提到的`componentWillMount`，則建議把裡面的程式碼搬到`componentDidMount`去。

接著讓我們快速來看一下新的生命週期如何替代舊的，以下我就直接使用官方給的範例了。這個範例會偵測 props 來決定要不要改變 state，是很常見的應用場景：

``` js
// Before
class ExampleComponent extends React.Component {
  state = {
    isScrollingDown: false,
  };
  
  componentWillReceiveProps(nextProps) {
    if (this.props.currentRow !== nextProps.currentRow) {
      this.setState({
        isScrollingDown:
          nextProps.currentRow > this.props.currentRow,
      });
    }
  }
}
```

而新的生命週期`static getDerivedStateFromProps`，會在 component 被建立還有收到新的 props 的時候被呼叫，但只會傳入新的 props 跟舊的 state，因此我們可以這樣改：

``` js
// After
class ExampleComponent extends React.Component {
  // 初始化 state
  state = {
    isScrollingDown: false,
    lastRow: null,
  };
  
  static getDerivedStateFromProps(nextProps, prevState) {
    // 把新的 props 跟舊的 state 做比較
    if (nextProps.currentRow !== prevState.lastRow) {
      // 回傳新的 state
      return {
        isScrollingDown: nextProps.currentRow > prevState.lastRow,
        lastRow: nextProps.currentRow, // 同步一下 state
      };
    }
  
    // return null 代表不用改變 state
    return null;
  }
}
```

其實說穿了就只是你自己把以前`componentWillReceiveProps`會傳來的`prevProps`存到 state 裡面，改成跟 state 來比較而已。

看到這邊你可能會很疑惑：「那為什麼 getDerivedStateFromProps 不直接把 prevProps 傳進來就好？」

React 官方給的理由有兩個：

1. 因為 getDerivedStateFromProps 在初始化的時候也會被 call，所以第一次的 prevProps 會是 null，代表你每次都要做一次 null check，這樣不好
2. 不傳 prevProps 就代表 React 不用幫你記住 prevProps 了，對未來在記憶體上面的優化有幫助

總之呢，以後就不會有`componentWillReceiveProps`可以用了，你要自己把需要的`prevProps`保存在 state 裡面，並且在`getDerivedStateFromProps`裡面進行比較。

再看另外一個例子，這個例子的目的是要在新增 item 的時候維持捲軸的位置，所以必須在 update 之前保存舊的高度，在 update 之後去調整捲軸的位置：

``` js
class ScrollingList extends React.Component {
  listRef = null;
  previousScrollHeight = null;
  
  componentWillUpdate(nextProps, nextState) {
    // 有新增 item 的話，記住現在的高度
    if (this.props.list.length < nextProps.list.length) {
      this.previousScrollHeight = this.listRef.scrollHeight;
    }
  }
  
  componentDidUpdate(prevProps, prevState) {
    // 如果 previousScrollHeight 不是 null，代表有新增 item
    // 調整捲軸位置
    if (this.previousScrollHeight !== null) {
      this.listRef.scrollTop += this.listRef.scrollHeight - this.previousScrollHeight;
      this.previousScrollHeight = null;
    }
  }
  
  render() {
    return (
      <div ref={this.setListRef}>
        {/* ...contents... */}
      </div>
    );
  }
  
  setListRef = ref => {
    this.listRef = ref;
  };
}
```

那這樣子會帶來的問題是什麼呢？還記得我們前面有提過 Fiber 有兩個階段嗎？render 跟 commit。這兩個階段會有時間差，而`componentWillUpdate`是處於第一個階段，`componentDidUpdate`是屬於第二個階段。

假如使用者在這兩個階段之間做了一些事情，例如說調整視窗的尺寸，那你存的高度就不會是正確的了，而是會拿到舊的值。

解決方法就是利用新的生命週期`getSnapshotBeforeUpdate`，這個會在 DOM 被更新之前呼叫，可以保證你拿到的東西一定是最新的。

``` js
class ScrollingList extends React.Component {
  listRef = null;
  
  getSnapshotBeforeUpdate(prevProps, prevState) {
    // 如果 list 有變動，就回傳現在的捲軸高度
    // 這個回傳值會被當作 componentDidUpdate 的第三個參數
    if (prevProps.list.length < this.props.list.length) {
      return this.listRef.scrollHeight;
    }
    return null;
  }
  
  componentDidUpdate(prevProps, prevState, snapshot) {
    // snapshot 就是上面回傳的那個值
    // 如果不是 null，就利用 snapshot 來調整捲軸高度
    if (snapshot !== null) {
      this.listRef.scrollTop +=
        this.listRef.scrollHeight - snapshot;
    }
  }
  
  render() {
    return (
      <div ref={this.setListRef}>
        {/* ...contents... */}
      </div>
    );
  }
  
  setListRef = ref => {
    this.listRef = ref;
  };
}
```

總之呢，結合搭配使用 commit phase 的 lifecycle（`componentDidMount`、`componentDidUpdate`、`componentWillUnmount `）以及新引進的`getDerivedStateFromProps`與`getSnapshotBeforeUpdate `，就可以取代掉舊的那些有可能會造成問題的 lifecycle。

如果想要看更多範例的話，這篇很值得參考：[Update on Async Rendering](https://reactjs.org/blog/2018/03/27/update-on-async-rendering.html)。

## 結論

效能一直是 Web App 很注重的一個點，而需要把握的原則就只有一個：不要 block main thread。只要 main thread 可以做事，它就可以去處理其他事情，例如說響應使用者的 click 或是繪製新的畫面等等。

而 React 原本的機制會造成問題，因此將內部核心用 Fiber 改寫，把一大個不可中斷的任務切割成許多小的、可以中斷的工作，而可以切割之後也使得以後有平行化的可能，render 的速度可能又會更快一點。

但也因為這樣機制的改變，影響到原本的生命週期，一個不小心就會出狀況，而官方也發布了新的兩個生命週期來解決這個問題。

身為 React 長期的使用者，對這種大的改變雖然覺得要改 code 很煩，但長期來看其實是利多，畢竟可以做的事情又更多了，效能也會愈來愈好。

這篇總結了近期我研究 Fiber 跟關注 React 新的變化的一些心得，Fiber 底層的實作機制因為我也不是很理解，所以不敢出來班門弄斧，只希望能透過白話文讓大家理解這個機制大概是長怎樣。

如果有哪邊有講錯，還麻煩不吝指正，感謝。

參考資料：

1. [React Fiber Architecture](https://github.com/acdlite/react-fiber-architecture)
2. [What is React Fiber ?](https://giamir.com/what-is-react-fiber)
3. [React中state render到html dom的流程分析](https://github.com/xieyu/blog/blob/master/React/from-jsx-to-dom.md)
4. [完全理解React Fiber](http://www.ayqy.net/blog/dive-into-react-fiber/)
5. [[翻譯] React Fiber 現狀確認](https://medium.com/@_cybai/%E7%BF%BB%E8%AD%AF-react-fiber-%E7%8F%BE%E7%8B%80%E7%A2%BA%E8%AA%8D-fd3808072279)
6. [React v16.3.0: New lifecycles and context API](https://reactjs.org/blog/2018/03/29/react-v-16-3.html)
7. [React Docs - Scheduling](https://reactjs.org/docs/design-principles.html#scheduling)
8. [浅谈React 16中的Fiber机制](https://tech.youzan.com/react-fiber/)
9. [Lin Clark - A Cartoon Intro to Fiber - React Conf 2017](https://www.youtube.com/watch?v=ZCuYPiUIONs)

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好