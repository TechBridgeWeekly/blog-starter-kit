---
title: 進階 React Component Patterns 筆記（下）
date: 2018-07-21 23:33:27
tags:
  - react
  - es6
  - javascript
  - pattern
author: arvinh
---

# 前言

上次我們介紹了三種 Rect Component Patterns，包含 `Compound component`、`Render props component` 與 `Prop collections & getters`，而今天要繼續往下介紹剩下的五個 Patterns:

* State Initializers
* State Reducer
* Control Props
* Provider
* Higher-order component

若對前三個 Pattern 不熟悉，或是沒看過上一篇文章的可以移駕至 [進階 React Component Patterns 筆記（上）](https://blog.arvinh.info/2018/06/27/advanced-react-component-patterns-note/)

接下來的 Pattern 都會延續之前的 Demo 範例，所以建議先閱讀過上篇！

此外，每個 Pattern 的最後都放有 codesandbox 的 demo link，覺得文字太多的可以直接去看完整的 code 喔！

# State Initializers

有時候我們會希望能讓元件回復到初始狀態，或是能讓使用者自己定義初始狀態，這時就適合採用 State initializer 技巧。

首先，我們利用自定義的 `initialState` 來存放元件初始狀態，而在真正的 state 中去 reference 它：

```js
class Toggle extends React.Component {
  static defaultProps = { onToggle: () => { } };
  initialState = { on: false };
  state = this.initialState;
  // ...other function
}
```

這樣一來，要實作 `reset` 函式就相當簡單了：

```js
reset = () =>
  this.setState(this.initialState, () =>
    this.props.onReset(this.initialState)
  );
```

而要讓使用者能自定義元件初始狀態的方式，相信多數讀者都有用過，也就是讓使用者透過 props 來定義元件的 initial state：

```js
class Toggle extends React.Component {
  static defaultProps = { onToggle: () => { }, initialOn: false };
  initialState = { on: this.props.initialOn };
  state = this.initialState;
  // ...other function
}
```

由於並不是所有使用者都會自行定義初始狀態，所以別忘了在 `defaultProps` 中宣告我們自己希望的初始值喔！

[Demo link](https://codesandbox.io/embed/2wp3jr6o8j)

上面的 Demo 範例是延續[上篇](https://blog.arvinh.info/2018/06/27/advanced-react-component-patterns-note/)提到的 `Prop collections & getters` 與 `Render props`，所以這邊加入的 `reset` 按鈕要記得加入 `getStateAndHelpers` 中傳遞給 `render props` 中的 children 使用。

# State Reducer

State Reducer 是一個蠻有趣的概念，主要目的是讓使用者能夠介入元件狀態改變的行為，讓元件在每次的 `setState` 時，都能夠被使用者影響。

舉個簡單的範例，像是我們先前的 Toggle component，如果今天使用者提出個需求，想要讓這個元件只能被 toggle 三次，那我們該怎麼做呢？

你當然可以讓使用者多傳一個 props 控制次數，然後在內部更動狀態時去檢查有沒有超過那個次數：

```js
class Toggle extends React.Component {
  static defaultProps = {
    onToggle: () => {},
    onReset: () => {},
    initialOn: false
  };
  // 元件內部多一個 toggleTimes 來控制目前的 toggle 次數
  initialState = { on: this.props.initialOn, currentToggleTimes: 0 };
  state = this.initialState;
  reset = () =>
    this.setState(this.initialState, () =>
      this.props.onReset(this.initialState)
    );
  toggle = () => {
    // 每次 toggle 時判斷有沒有超過使用者定義的 toggle 次數上限
    if (this.state.currentToggleTimes >= this.props.toggleTimes) {
      console.log('toggle too much')
      return;
    }
    this.setState(
      ({ on }) => ({
        on: !on,
        currentToggleTimes:this.state.currentToggleTimes + 1 }),
      () => this.props.onToggle(this.state.on),
    );
  }
  // ...other methods
}
```

但你也知道使用者的需求總是會變動，假如他突然間也想控制 reset 的次數怎麼辦? 你的程式不就改不完？

這時我們就能採用 `State Reducer`，先看一下使用者應該會怎麼使用 `State Reducer`：
｀
```js
class App extends React.Component {
  initialState = { timesClicked: 0 }
  state = this.initialState
  toggleStateReducer = (state, changes) => {
    // state 為 Toggle 的 current state
    // changes 為該次 Toggle 動作所造成的改變
    if (this.state.timesClicked >= 4) {
      return { ...changes, on: false }
    }
    return changes
  }
  render() {
    return (
      <div className="App">
        <Toggle
          initialOn={true}
          onToggle={on => {
            this.setState(({ timesClicked }) => ({
               timesClicked: timesClicked + 1,
             }))
          }}
          onReset={initialState => this.setState(this.initialState)}
          stateReducer={this.toggleStateReducer}
        >
          {({ on, getTogglerProps, reset }) => (
            // render props
          )}
        </Toggle>
      </div>
    );
  }
}
```

我們讓使用者傳入一個 `stateReducer`，其中接受兩個參數，一個是 Toggle component 的 current state，另一個是該次 Toggle component 執行 `setState` 時，所接受的變化 `changes`，而回傳值就是 Toggle component 實際 `setState` 時所接受的 change object。

因此在這個函式中，使用者就擁有了一個機會能夠在元件真正觸發 `setState` 之前，進行一些操作，以剛剛例子來說，就能在這邊判斷使用者自己紀錄的 state(`timesClicked`) 有沒有超過某個值，如果超過了，那我們之後每次的回傳結果中，都會將 `on` 這個 state 設為 false。

那元件本身該如何讓 `stateReducer` 介入 `setState` 中呢？重點就在這段：

```js
internalSetState(changes, callback) {
    this.setState(currentState => {
      // 確認傳入的 changes 是單純的物件，或是函式
      const changesObject =
        typeof changes === 'function' ? changes(currentState) : changes
      // 呼叫使用者傳入的 stateReducer 來取得最終的 state change object
      const reducedChanges =
        this.props.stateReducer(currentState, changesObject) || {}
      // 最後只是檢查一下 changes 是否為空，避免重複 render
      return Object.keys(reducedChanges).length
        ? reducedChanges
        : null
    }, callback)
  }
```

我們需要建立一個介面與原本 `setState` 相同的 `internalSetState` 的方法，取代原本的 `setState`。

其中需要注意的有兩點，一個是原本的 `setState` 是能接受函式當第一個參數的，因此我們需要先判斷 `changes` 是否為 function，才能繼續進行其他動作。

另一個則是並非所有的 `setState` 都一定要用 `internalSetState` 取代，像是 `reset` function 我們可能不太希望使用者能介入，應該要很明確的 reset 所有狀態，因此這邊可以用原本的 `setState`。

看看 [Demo Link](https://codesandbox.io/embed/wyl152o1jw) ，並實際玩玩看會更清楚！

另外，在 Kent C. Dodds 的 workshop 中，他在 internalSetState 的實作上有提到一種他比較偏好的寫法：

```js
internalSetState(changes, callback) {
    this.setState(currentState => {
      return [changes]
        .map(c => typeof c === 'function' ? changes(currentState) : c)
        .map(c => this.props.stateReducer(currentState, c) || {})
        .map(c => Object.keys(reducedChanges).length ? c : null)[0]
  }
```

透過硬轉成 array 後，再用 map 將每個步驟 chain 起來，的確比較乾淨跟簡單，但比起原本做法沒那麼直覺就是了，尤其是最後還要取 `[0]` 出來，但參考一下也不錯！

透過 `State Reducer`，不僅使用者開心（能夠介入元件 state 的更動），開發者也不用疲於奔命一直改 code（讓使用者自己處理 reducer 實際內容），但壞處就是你需要呼叫一個 `internalSetState` 的函式，蠻可能造成 trace code 上的困擾，算是個 trade-off。

# Control Props

除非你從來沒有用 React 開發與表單相關的 component，否則你一定用過 `Control Props`，因為所謂的 `Control Props` 其實就是 `Controlled component` 的一種實作。

舉例來說，`Select`, `Input` 等 `Form` 的元件，當使用者輸入值時，其改變的是元件的內部狀態，該狀態通常綁定在 `value` 這個屬性上頭。

若在 React 中想要取得使用者輸入進表單元件的值時，你就會想要將 state 綁定在元件的 `value` 上頭，然而，一但你傳值給 `value`（也就是 `value={this.state.value}`），你就必須要自己利用 handler 去控制它的狀態改變，否則使用者再怎麼輸入，都不會改變其狀態。因為在你傳值給 `value` 的時候，這個元件就已經歸你控制了，這樣的方式可以保證該元件內部狀態是 single source of truth，不會有使用者的輸入與你的 state 不一致的狀態發生。（關於 `Controlled component` 在 React 官方網站有詳細的[介紹](https://reactjs.org/docs/forms.html)）

所以說，`Control Props` 就是想利用這樣的技巧，讓你的元件在讓使用者自行操作 input 時，能確保元件內部狀態的 single source of truth。透過這種方式，也就能夠從使用者角度來同步多個元件的內部狀態。

一樣已先前的 Toggle 元件來舉例，但這次我們用個簡化版：

假設今天使用者想同步兩個元件的狀態，他們可以透過本身的 `State` 來控制，並在 `onToggle` 時來更動 `State`：

```js
class App extends React.Component {
  state = { bothOn: false };
  handleToggle = on => {
    this.setState({ bothOn: on });
  };
  render() {
    return (
      <div className="App">
        <Toggle on={this.state.bothOn} onToggle={this.handleToggle}>
           {({ on, toggle }) => (
            <div>
              {on ? "The button is on" : "The button is off"}
              <hr />
              <button className="button1" onClick={toggle}>
                {on ? "click on" : "click off"}
              </button>
              <hr />
            </div>
          )}
        </Toggle>
        <Toggle on={this.state.bothOn} onToggle={this.handleToggle}>
          {({ on, toggle }) => (
            // same render props as above
          )}
        </Toggle>
      </div>
    );
  }
}

```

但要記得，`onToggle` 實際上是 `Toggle` 元件內部執行完 `toggle` 後才會執行的動作（告知使用者該元件"被" Toggle 了），這樣的話，元件要怎麼依照傳入的 Props 來處理內部狀態呢？

來看一下我們 Toggle 的實作：

```js
class Toggle extends React.Component {
  state = { on: false };
  isControlled(prop) {
    return this.props[prop] !== undefined;
  }
  getState() {
    return Object.entries(this.state).reduce((combinedState, [key, value]) => {
      if (this.isControlled(key)) {
        combinedState[key] = this.props[key];
      } else {
        combinedState[key] = value;
      }
      return combinedState;
    }, {});
  }
  toggle = () => {
    if (this.isControlled("on")) {
      this.props.onToggle(!this.getState().on);
    } else {
      this.setState(
        ({ on }) => ({ on: !on }),
        () => {
          this.props.onToggle(this.getState().on);
        }
      );
    }
  };
  render() {
    return this.props.children({ ...this.getState(), toggle: this.toggle });
  }
}
```

主要重點在於，每次 `toggle` 被 trigger 時，我們都會先去確認一下 `on` 這個 state 有沒有被使用者 `Controlled`（`isControlled()`），若是使用者有透過 `props`（使用者端）傳值給這個 `state`（元件內部），就代表我們得將該 `state` 的掌控交給使用者。

什麼叫『交給使用者』呢？

其實也就是要將使用者傳入的 props 與我們自己本身的 state 做 **combination**，並將結果當作元件實際的 state 來使用，如同上述程式碼中的 `getState()` 函數。之後元件所有需要操作 state 的地方都需要透過該函數來取得元件的 **Current State**。

如此一來，只要使用者有傳入 `on` 這個 props，元件內部關於 `on` 這個 state 的變化，就會像是由使用者本身操控一般（因為我們在每次取得 current state 時都會 merge props 中對應的值），也就能讓使用者同步多個 `Toggle` component 了！

`Control Props` 用文字敘述比較繁瑣難懂，可以到下面的 demo link 玩玩，試著把 `Toggle` component 的 `on` props 拿掉看看差別，拿掉 props 後，兩個元件的狀態就無法同步，但元件本身的狀態還是正常的。
[demo link](https://codesandbox.io/embed/p94nmr2p2m)

在 Kent C. Dodds 的 workshop 中，他其實還有介紹如何整合先前的 `State Reducer` 與 `Control Props`，不過我覺得過於複雜，除了很難光用文字敘述外，實際使用的機會感覺也不大，如果有興趣的讀者可以直接去 [codesandbox](https://codesandbox.io/s/github/kentcdodds/advanced-react-patterns-v2) 上看範例(file 10.js)

# Provider

Provider pattern 其實是為了解決 `Props drilling` 的問題，什麼是 `Props drilling` 呢？

舉個簡單例子：

```js
class Toggle extends React.Component {
  state = { on: false };
  toggle = () => { /*...*/ };
  render() {
    return this.props.children({ ...this.state, toggle: this.toggle });
  }
}
const Layer1 = ({toggle, ...props}) => <Layer2 toggle={toggle} />
const Layer2 = ({toggle, ...props}) => <Layer3 toggle={toggle} />
const Layer3 = ({toggle, ...props}) => <button onClick={toggle} />
class App extends React.Component {
  handleToggle = () => {};
  render() {
    return (
      <Toggle onToggle={this.handleToggle}>
        <Layer1 />
      </Toggle>
    );
  }
}
```

我知道這段 code 很奇怪，但這裡想呈現的重點是，有些時候我們可能真的想要把某個外層的 props 往下傳遞給底下的 component，這種情況下可能得一層一層將 props 往下帶，即便中間經過的 component 都不需要用到該 props。

要解決這樣的問題，可以利用 React 的 [`Context API`](https://medium.com/dailyjs/reacts-%EF%B8%8F-new-context-api-70c9fe01596b)。

雖然在 React 16 以前，`Context API` 在官方文件是一直處於一種不推薦使用的狀態，但大概因為太多人需要吧（像是 `redux` 等 state management 其實都有用到），現在有了新的實作，讓我們終於可以放心使用 `Context API` 了，因此這邊要介紹的 `Provider pattern`，其實就是利用 React 最新的 `Context API` 來解決 `Props drilling` 問題！

早在[上篇](https://blog.arvinh.info/2018/06/27/advanced-react-component-patterns-note/)中介紹的 `Compound component` 我們就有用到 Provider pattern 了，而現在就讓我們用剛剛那個離奇的例子來做點修正吧：

```js
const ToggleContext = React.createContext();
class Toggle extends React.Component {
  static Consumer = ToggleContext.Consumer;
  toggle = () => this.setState(({ on }) => ({ on: !on }));
  state = { on: false, toggle: this.toggle };
  render() {
    const { children, ...rest } = this.props;
    const ui = typeof children === "function" ? children(this.state) : children;
    return (
      <ToggleContext.Provider value={this.state} {...rest}>
        {ui}
      </ToggleContext.Provider>
    );
  }
}
```

利用 React 16 後出現的 `React.createContext()`，創造一個 `ToggleContext`，並將其提供的 `Consumer` 當作 static 變數放在 `Toggle` 中。

接著在 render function 中我們使用 `Context API` 提供的另一個 component `Provider`，將傳入 `Toggle` 的 render props 包裹住，並且將 `Toggle` 本身的 `state` 或 `function` 傳到 `value` 這個 props 中。如此一來，`Toggle` 底下的所有 children 之後只要將自己用 `Toggle.Consumer` 包住就可以自由存取 `Toggle` 傳下來的 `value`：

```js
const Layer1 = () => <Layer2 />;
const Layer2 = () => <Layer3 />;
const Layer3 = () => (
  <Toggle.Consumer>
    {({ on, toggle }) => (
      <Fragment>
        <div>{on ? "The button is on" : "The button is off"}</div>
        <button className="button1" onClick={toggle}>
          {on ? "click on" : "click off"}
        </button>
      </Fragment>
    )}
  </Toggle.Consumer>
);

class App extends React.Component {
  render() {
    return (
      <div className="App">
        <Toggle>
          <Layer1 />
        </Toggle>
      </div>
    );
  }
}
```

由上面的程式碼可以看到，`Toggle` component 的 `state` 與 `toggle` function 都會被當成 props 傳給被 `Toggle.Consumer` 包裹著的 children。

包在第三層的 `<Layer3 />` 就可以直接拿到想要的 `on` 與 `toggle`，再也不用從 `Layer1` 傳到 `Layer2` 再傳到 `Layer3` 了！

[Demo Link](https://codesandbox.io/embed/m3p2p38z5j)

# Higher-order component

最後一個 Pattern 我想是大家最熟悉，也是我認為最需要懂得融會貫通的 `Higher-order component`，通常簡稱 `HOC`。旨在解決 [Cross-Cutting Concerns](https://en.wikipedia.org/wiki/Cross-cutting_concern)，說白一點就是讓你將一些可共用的邏輯抽取出來，讓其他元件透過 `HOC` 的包裝後，能獲得該共用功能，之後修改新增時不會因為邏輯跟元件綁太緊而出現問題。

雖然很重要，但這個 Pattern 相對簡單，React 官網其實就有[非常詳細的介紹](https://reactjs.org/docs/higher-order-components.html)。這邊就簡單介紹就好，先來個範例吧：

```js
const Layer1 = () => <Layer2 />;
const Layer2 = () => <Layer3 />;
const Layer3 = withToggle(({contextProps: { on, toggle }}) => (
  <Fragment>
    <div>{on ? "The button is on" : "The button is off"}</div>
    <button className="button1" onClick={toggle}>
      {on ? "click on" : "click off"}
    </button>
  </Fragment>
));
const Layer4 = withToggle(({contextProps: { on, toggle }}) => (
  <Fragment>
    <div>
      <button className="button2" onClick={toggle}>
        {on ? "click on" : "click off"}
      </button>
    </div>
    <div>{on ? "The button2 is on" : "The button2 is off"}</div>
  </Fragment>
));
class App extends React.Component {
  render() {
    return (
      <div className="App">
        <Toggle>
          <Layer1 />
          <Layer4 />
        </Toggle>
      </div>
    );
  }
}
```

這個範例延續前一個 `Provider pattern`，我們將 `Toggle.Consumer` 抽出來，包裝成一個 `HOC` `withToggle`，這樣一來，我們可以輕鬆製造出多個擁有 `Toggle` component 功能與狀態的元件，像是這邊的 `Layer3` 與 `Layer4`，他們只需要 care 自己的 UI 邏輯即可，剩下與 `Toggle` 相關的狀態操作都交由 `withToggle` 這個 HOC 幫忙處理。

而 `withToggle` 長這樣：

```js
function withToggle(Component) {
  function Wrapper(props, ref) {
    return (
      <Toggle.Consumer>
        {toggleContext => (
          <Component contextProps={toggleContext} {...props} ref={ref} />
        )}
      </Toggle.Consumer>
    );
  }
  Wrapper.displayName = `withToggle(${Component.displayName ||
    Component.name})`;
  return hoistNonReactStatics(React.forwardRef(Wrapper), Component);
}
```

是不是很簡單呢！

`HOC` 負責主要的共用邏輯，在這邊就是 `Toggle.Consumer` 這段，然後將傳入的 `Component` 塞入，可能是放在 `render` 或是像這邊是傳入 `Consumer` 的 children。

特別要注意的有三點，一個是 `displayName`，由於 `HOC` 會回傳一個新的 Component，這時如果你沒有明確定義一個 `displayName` 的話，在 Dev tool 裡你就只能看到一個 `Unknown` 的元件，會造成開發上的困擾，所以記得要指定一下 `displayName`，通常會用 `HOC` 自己的名稱加上原有 Component 的 `displayName`。

另一個要注意的點是 `forwardRef`，在 React 中，`ref` 與 `props` 的處理方式不相同，`ref` 並不會如同 props 一般往下傳遞，若你想要取得被 `HOC` 包裹過的 component 的 `ref`，那在你的 `HOC` 中，必須使用 `React.forwardRef` 將其 forward 下去，詳細介紹可以看[官網說明](https://reactjs.org/docs/forwarding-refs.html)。

最後，假如你原先的 component 有一些 `static method`，透過 `HOC` 包裝後，你可能會發現那些 `static method` 都取不到了！

你必須要在 `HOC` 中自行複製一份到 `HOC` 上頭，像這樣（取自 [React 官網](https://reactjs.org/docs/higher-order-components.html#static-methods-must-be-copied-over))：

```js
function enhance(WrappedComponent) {
  class Enhance extends React.Component {/*...*/}
  // Must know exactly which method(s) to copy :(
  Enhance.staticMethod = WrappedComponent.staticMethod;
  return Enhance;
}
```

但這樣太麻煩了，我們可以直接利用 `hoistNonReactStatics` 這套 lib 來幫忙，這樣就萬無一失了！

```js
import hoistNonReactStatic from 'hoist-non-react-statics';
function enhance(WrappedComponent) {
  class Enhance extends React.Component {/*...*/}
  hoistNonReactStatic(Enhance, WrappedComponent);
  return Enhance;
}
```

[Demo Link](https://codesandbox.io/embed/q3wmv6okqw)

# 結論

介紹了這麼多種 Pattern，其實我覺得 HOC、Render Props 與 Compound Component 是最需要好好掌握並且多加運用的，其他如 State Reducer、Prop Collections and Getters 則是平常在進行 Code Review 時，可以好好拿出來思考一下是否能夠採用，為你的專案加分。
無論如何，經過這樣的學習與紀錄，至少讓自己平日開發時，能主動多思考一些優化的方向與可能性，總體是蠻有收穫的！

最後提供大家 Kent C. Dodds 在 workshop 後自己寫的一篇文章，[Mixing Component Patterns](https://blog.kentcdodds.com/mixing-component-patterns-4328f1e26bb5)，裡頭他將這些 pattern 結合在一起使用，有興趣的讀者可以去看看到底這麼多 Pattern 要怎麼融合使用。

謝謝真的有看到這邊的各位，這些筆記斷斷續續的紀錄，一不小心就篇幅很多...若發現中間有敘述不順或是錯誤的地方，歡迎大家告知！

## 資料來源

1. [Advanced React Patterns workshop](https://frontendmasters.com/courses/advanced-react-patterns/)
2. [Advanced React Patterns V2 codesandbox](https://codesandbox.io/s/github/kentcdodds/advanced-react-patterns-v2)
3. [Advanced React Patterns Blog](https://blog.kentcdodds.com/advanced-react-component-patterns-56af2b74bc5f)
4. [Answers to common questions about render props](https://blog.kentcdodds.com/answers-to-common-questions-about-render-props-a9f84bb12d5d)
5. [Do more with less using render props](https://hackernoon.com/do-more-with-less-using-render-props-de5bcdfbe74c)
6. [React new context api](https://medium.com/dailyjs/reacts-%EF%B8%8F-new-context-api-70c9fe01596b)
7. [Mixing Component Patterns](https://blog.kentcdodds.com/mixing-component-patterns-4328f1e26bb5)

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化