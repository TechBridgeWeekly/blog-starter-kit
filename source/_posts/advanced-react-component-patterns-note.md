---
title: 進階 React Component Patterns 筆記（上）
date: 2018-06-27 23:33:27
tags:
  - react
  - es6
  - javascript
  - pattern
author: arvinh
---

# 前言

前不久在 Frontend masters 看到一部影片 [Advanced React Patterns](https://frontendmasters.com/courses/advanced-react-patterns/)，是 Paypal 的工程師 Kent C. Dodds 在某場 workshop 介紹他實作 React component 時所使用的進階 React Patterns（目前似乎要付費觀看，而他在 [egghead.io](https://egghead.io/courses/advanced-react-component-patterns) 上也有一系列教學，同樣要收費就是了...），內容非常豐富扎實，若是想提供一些可高度客製化的共用元件，使用這些 Pattern 絕對是有絕佳的幫助！如果可以，我強烈建議大家自己去聽聽，不用把時間花在閱讀我的文字上 :p

自從看完那影片後，平常開發都會不自覺得去思考這些 pattern，看看如果應用到我目前的專案中，能否帶來益處。

當然有些早已有在使用，而有些則苦無伸展之處，為了加深自己對這些 pattern 的印象與理解，今天就來筆記一下這系列影片的內容，也希望能對來到這邊的讀者有所幫助。

## 為何要有 Pattern

在開始介紹之前，我其實想討論一下為什麼需要有 Pattern，或是說，我們真的需要 Pattern 嗎？

會有這樣的疑問其實是在過往經驗中，還蠻常發現採用 Design pattern 的程式碼如果需要修改時，對於不熟悉該 Pattern，或是原作者雖然採用某種 Pattern，但沒有堅持到底，在一些地方為了方便而改採取別種作法；更慘的情況是，在錯誤的情境下，採用了不適合的 pattern，這些情況都容易造成維護上的極大困擾。

所謂的 Pattern，代表的應該是『被分門別類過，通過反覆使用與測試的考驗，且多數人知曉的程式設計經驗總結』，如果說，你今天採用的 Pattern 是團隊內的人都能接受並理解的，然後也都同意該情境下非常適合使用那種 Pattern，非他莫屬，直接打趴其他簡單做法的優缺點，那麼使用 Pattern 是絕對有好處的，除了測試的穩定性、程式碼的重用性外，在開發共用 component 或套件上，更有助於一般元件使用者，像是可以 export 出更加簡潔、方便的 API 等等。

若不是上述情況，有道是『最好的 Pattern，就是沒有 Pattern』，讓邏輯單純簡單、不影響效能、方便接手的人進行維護，可能比起刻意套用 Pattern 來得有效益。所以在開發前，最好花點時間釐清自己的狀況。

不過，你也得先了解有哪些 Patterns，才能在開發時有參考依據麻！所以不管你現在用不用得到，多學還是好的！

# React Component Patterns

主要想介紹的 React Component Pattern 有幾種：

* Compound comopnent
* Render props comopnent
* Prop Collections and Getters
* State Initializers
* State Reducer
* Control Props
* Provider
* Higher-order component

但礙於篇幅關係，打算拆成幾篇文章來說明，今天是上篇，會介紹到 `Compound component`、`Render props component` 與 `Prop Collections and Getters`。

## Compound component

假設今天你要撰寫一個 `Toggle` Component，並且分別在 state `On` 或 `Off` 時，能夠顯示不同的文字描述，你會怎麼做？

一個很直覺的做法是，利用 props 來傳遞不同 state 時所需要顯示的文字（只是呈現 idea，並非有功能的 code）：

```js
class Toggle extends React.Component {
  state = { on: false }
  render() {
    const { whenOn, whenOff } = this.props;
    return this.state.on ? whenOn : whenOff;
  }
}
<Toggle
  whenOn="Toggle is on"
  whenOff="Toggle is off"
/>
```

這樣做當然可行，但有幾個明顯的壞處：

* 順序無法調動：沒辦法讓 `Toggle` component 的使用者自由得調整 `whenOn` 與 `whenOff` 的顯示順序，除非你另外加入其他 props 來控制。
* 利用 Props 來傳遞 UI 元件的話，容易造成使用者困擾，只能盡量透過 props 的命名來區別 UI 元件與其他狀態控制的 pros，舉例來說，如果今天 `Toggle` 還有一個 props 是來控制 toggle on 時的 callback：
    ```js
      <Toggle
        whenOn="Toggle is on"
        whenOff="Toggle is off"｀
        whenOnCallback={() => console.log('yeee, I am on')}
      />
    ```
    看起來就不是一個非常好的 API 設計，對吧？

要解決這種情況，最適合的方式就是採用 `Compound component`！

所謂的 **Compound component** 就是讓你的 UI 元件透過 `this.props.children` 的方式傳入給 `parent component`，利用 `React.Children.map()` 來 render 所有傳入的 `this.props.children`，並且透過 `React.cloneElement` 將 parent 的 `state` 傳入每個 children 的 `props`，讓 parent 與 children 之間會 **隱含著狀態的共享**，對於元件使用者來說，他只需要傳入想要的 `children component`，不用知道 parent 與 children 之間如何溝通，當然也能隨意調整順序，這樣的 API 設計，對於元件使用者就非常的友善。

在這樣的原則下，不難發現，Compound component 必須要 **同時結合使用 parent component 與 children component 才有意義**。

以剛剛的 `<Toggle>` 為例子，若改以 Compound component 的話：

```js
class Toggle extends React.Component {
  static On = ({ on, children }) => on && children;
  static Off = ({ on, children }) => !on && children;
  static Button = ({ on, toggle }) => <button onClick={toggle} />
  static defaultProps = { onToggle: () => { } }
  state = { on: false }
  toggle = () =>
    this.setState(
      ({ on }) => ({ on: !on }),
      () => this.props.onToggle(this.state.on),
    )
  render() {
    const children = React.Children.map(
      this.props.children,
      child =>
        React.cloneElement(child, {
          on: this.state.on,
          toggle: this.toggle,
        }),
    )
    return <div>{children}</div>
  }
}
<Toggle
  onToggle={on => console.log('toggle', on)}
>
  <Toggle.On>The button is on</Toggle.On>
  <Toggle.Off>The button is off</Toggle.Off>
  <Toggle.Button />
</Toggle>
```

[Demo link](https://codesandbox.io/embed/6zz376vrzz)

我們透過 `Toggle` Class 的 static properties 來定義 children component，這樣的好處是我們能夠從名稱上就保持 Parent 與 Children Component 之間的關聯性。

透過 Compound Component，不僅能讓使用者自訂元件順序，又能將 UI component 與其他 callback props 做個清楚的切割。對於使用者來說，完全不需要管 `Toggle` 的 state 變化時，`Toggle.On` 要怎麼變動，他們之間的 state 與 props 都由 Parent component 處理。

另外，在 Kent C. Dodds 的 workshop 中，有人提問為何不直接用 `this.props.children.map` 就好，而要用 `React.Children.map`？

原因在於，在目前 React 的實作中，當你的 children 只有一個時，`this.props.children` 不會是一個 array，透過 `React.Children.map` 能寫得更簡潔，少些判斷。

看到這邊，不知道有沒有想到，其實我們在撰寫表單時常用的 `<select>` 與 `<option>` 就是一個 Compound component 的例子：

```html
<select>
  <option>Opt 1<option>
  <option>Opt 2<option>
</select>
```

## 更有彈性的 Compound Component

在上面的例子中，其實只要使用者更動其中一個 children，整個 Compound component 就會壞掉：

```js
<Toggle
  onToggle={on => console.log('toggle', on)}
>
  <Toggle.On>The button is on</Toggle.On>
  <Toggle.Off>The button is off</Toggle.Off>
  <div>
    <Toggle.Button />
  </div>
</Toggle>
```

因為 `React.Children.map` 只會 for-loop 到第一層的 children，也就是說，`React.cloneElement` 現在不會複製 `<Toggle.Button>` 而是複製 `<div>` 了。

不過好在現在有了 React 16 的 Context API，要解決這個問題簡單了不少，主要就是將 `state` 的共享改由 `Context` 來完成：

```js
const ToggleContext = React.createContext({
  on: false,
  toggle: () => {}
});
class Toggle extends React.Component {
  static On = ({ children }) => (
    <ToggleContext.Consumer>
      {contextValue => contextValue.on && children}
    </ToggleContext.Consumer>
  );
  static Off = ({ children }) => (
    <ToggleContext.Consumer>
      {contextValue => !contextValue.on && children}
    </ToggleContext.Consumer>
  );
  static Button = () => (
    <ToggleContext.Consumer>
      {contextValue => <button onClick={contextValue.toggle}> Toggle </button>}
    </ToggleContext.Consumer>
  );
  static defaultProps = { onToggle: () => {} };
  state = { on: false };
  toggle = () =>
    this.setState(
      ({ on }) => ({ on: !on }),
      () => this.props.onToggle(this.state.on)
    );
  render() {
    // 由於不用傳遞 props 給 children，也就不用 React.Children.map 了，直接使用 this.props.children 即可
    return (
      <ToggleContext.Provider
        value={{
          on: this.state.on,
          toggle: this.toggle
        }}
      >
        {this.props.children}
      </ToggleContext.Provider>
    );
  }
}
<Toggle onToggle={on => console.log("toggle", on)}>
  <Toggle.On>The button is on</Toggle.On>
  <Toggle.Off>The button is off</Toggle.Off>
  <div>
    <Toggle.Button />
  </div>
</Toggle>
```

[Demo Link](https://codesandbox.io/embed/mzv766qqmx)

如此一來，使用者想要在我們的 Componet 裡面如何 wrap 各個 Child Component 都可以了！

**Note:** 在 workshop 中，Kent C. Dodds 有提到關於在 render 時， `Provider` 每次都會接收到新 object，造成多餘 re-render 的問題，因此可以做點小改良：
將要傳給 `Provider` 的 value 改以 `this.state` 傳入，這樣每次在 render `Toggle` 時，才不會讓所有 `Toggle` 的 children component 也 re-render。
只是要特別注意你原本要傳入 context 的 value 內，若有一些 function 如 `this.toggle`，記得確保你的 state 在宣告時能取得到。

```js
//...
toggle = () =>
  this.setState(
    ({ on }) => ({ on: !on }),
    () => this.props.onToggle(this.state.on)
  );
state = { on: false, toggle: this.toggle }; // 將 state 移至 toggle function 之下，以確保 refer 得到
render() {
  // 由於不用傳遞 props 給 children，也就不用 React.Children.map 了，直接使用 this.props.children 即可
  return (
    <ToggleContext.Provider
      value={this.state}
    >
      {this.props.children}
    </ToggleContext.Provider>
  );
}
//...
```

不過當你並不想把所有 `state` 都放入 `context` 時，可能就得想另一種方法了。

# Render props comopnent

Render props 相對於 Compound Component 來說應該是比較有名的 Pattern，而他的概念與實作方式也相對簡單，網路上也早有針對 Render props 的相關討論，像[這個](https://blog.kentcdodds.com/answers-to-common-questions-about-render-props-a9f84bb12d5d)或[這個](https://hackernoon.com/do-more-with-less-using-render-props-de5bcdfbe74c)。

從名稱就可以猜出一二，所謂 Render props 就是將 render function 當作 props 傳入，讓原本的 render function 的控制權，從內部元件本身移轉至使用該元件的使用者身上，這種方式讓使用元件的使用者可以更方便的操作 `｀state`，而實作方式上比較常看到的有兩種：

* 將 render 當作 props 傳入：
    ```js
      class Toggle extends React.Component {
        static defaultProps = { onToggle: () => { } }
        state = { on: false }
        toggle = () =>
          this.setState(
            ({ on }) => ({ on: !on }),
            () => this.props.onToggle(this.state.on),
          )
        render() {
          return this.props.renderToggle({on: this.state.on, toggle: this.toggle});
        }
      }
      <Toggle
        onToggle={on => console.log('toggle', on)}
        renderToggle={({on, toggle}) => (
          <div>
            {on ? 'The button is on' : 'The button is off'}
            <hr />
            <buttononClick={toggle}>
              {on ? 'click on' : 'click off'}
            </button>
          </div>
        )}
      />
    ```

* 用 `this.props.children` 來呼叫 render props：

    ```js
      class Toggle extends React.Component {
          static defaultProps = { onToggle: () => { } }
          state = { on: false }
          toggle = () =>
            this.setState(
              ({ on }) => ({ on: !on }),
              () => this.props.onToggle(this.state.on),
            )
          render() {
            return this.props.children({on: this.state.on, toggle: this.toggle});
          }
        }
        <Toggle
          onToggle={on => console.log('toggle', on)}
        >
          {({on, toggle}) => (
            <div>
              {on ? 'The button is on' : 'The button is off'}
              <hr />
              <button onClick={toggle}>
                {on ? 'click on' : 'click off'}
              </button>
            </div>
          )}
        </Toggle>
    ```

[Demo Link](https://codesandbox.io/embed/6zx6q92qw)

以這兩種方法來說，我的想法與 Kent C. Dodds 相同，以 `this.props.children` 來呼叫 render props 在使用上比較有優勢，除了可以明確知道 `Toggle` 元件的起始點外，還可以避免 當 renderProps 的內容很多時，容易 miss 掉一些 `Toggle` 的重要 props 的問題，例如下面的範例：

<iframe src="https://codesandbox.io/embed/jv9xjz5mr3?view=editor" style="width:100%; height:500px; border:0; border-radius: 4px; overflow:hidden;" sandbox="allow-modals allow-forms allow-popups allow-scripts allow-same-origin"></iframe>

# Prop Collections and Getters

在 Kent C. Dodds 的 workshop 中所提到的 `Prop Collections` 與 `Prop Getters` 其實是要搭配運用的。

我們將上面的例子稍微改變一下：

```js
<Toggle onToggle={on => console.log("toggle", on)}>
  {({ on, toggle }) => (
    <div>
      {on ? "The button is on" : "The button is off"}
      <hr />
      <button class="button1" onClick={toggle} aria-pressed={on}>
        {on ? "click on" : "click off"}
      </button>
      <hr />
      <button class="button2" onClick={toggle} aria-pressed={on} aria-label="custom-button2">
        {on ? "click on" : "click off"}
      </button>
    </div>
  )}
</Toggle>
```

現在我們有兩個 button 可以同時更改 Toggle 的狀態，而稍微注意一下可以發現兩個 button 其實接收的 props 有共通的部份，目前的寫法很多餘，也不好看。這時候我們就可以創造一個 `Prop collections` 來負責提供 Common 的 props：

```js
class Toggle extends React.Component {
  //... Same as before
  getStateAndHelpers() {
    return {
      on: this.state.on,
      togglerProps: { // collection for common Props
        "aria-pressed": this.state.on,
        onClick: this.toggle
      }
    };
  }
  render() {
    return this.props.children(this.getStateAndHelpers());
  }
}
```

而剛剛的範例就能修改為：

```js
<Toggle onToggle={on => console.log("toggle", on)}>
  {({ on, togglerProps }) => (
    <div>
      {on ? "The button is on" : "The button is off"}
      <hr />
      <button class="button1" {...togglerProps}>
        {on ? "click on" : "click off"}
      </button>
      <hr />
      <button
        class="button2"
        {...togglerProps}
        aria-label="custom-button2"
      >
        {on ? "click on" : "click off"}
      </button>
    </div>
  )}
</Toggle>
```

但這樣做你可能會發現一個缺點，就是當別人不小心 overwrite 你的 common props 時，你無能為力。這其實也是 `render props` 的一個小缺點，例如：

```js
<button
  class="button1"
  {...togglerProps}
  onClick={() => 'overwrite'} // 蓋過我們原先在 onClick 的操作 (this.toggle)
>
  {on ? "click on" : "click off"}
</button>
```

## 這時候 `Prop Getters` 就可以出動了！

在原本的 Prop collections（`togglerProps`）中，我們改以呼叫一個 `Prop Getter` 的方式取得 common props：

```js
class Toggle extends React.Component {
  //... Same as before
  getTogglerProps = ({ onClick, ...props } = {}) => ({
    'aria-pressed': this.state.on,
    onClick: () => {
      this.toggle();
      onClick && onClick();
    },
    ...props,
  })

  getStateAndHelpers() {
    return {
      on: this.state.on,
      getTogglerProps: this.getTogglerProps
    };
  }
  render() {
    return this.props.children(this.getStateAndHelpers());
  }
}
```

如此一來，使用者所提供的 `onClick` 就不會蓋過我們元件原始的 `onClick` 行為，也就是 `this.toggle`：

```js
<button
  class="button2"
  {...getTogglerProps({
    onClick: () => console.log('overwrite')
  })}
  aria-label="custom-button2"
>
  {on ? "click on" : "click off"}
</button>
```

如果想要 overwrite 更多的 method，在 `Prop Getter` 中也要做相對應的判斷與修改：

```js
getTogglerProps = ({ onClick, otherMethod, ...props } = {}) => ({
  'aria-pressed': this.state.on,
  onClick: () => {
    this.toggle();
    onClick && onClick();
  },
  otherMethod: () => {
    this.originOtherMethod();
    otherMethod && otherMethod();
  }
  ...props,
})
```

如果嫌每次都要判斷傳入的 props 是否存在很麻煩，可以學 Kent C. Dodds 寫一個 handy funtion：

```js
const callAll = (...fns) => (...args) =>
  fns.forEach(fn => fn && fn(...args));
getTogglerProps = ({ onClick, otherMethod, ...props } = {}) => ({
  'aria-pressed': this.state.on,
  onClick: callAll(onClick, this.toggle),
})
```

[Demo Link](https://codesandbox.io/embed/wywk4w1z6w)

# 結論

今天統整了三個 React component patterns: `Compound Component`、`Render Props` 與 `Prop Collections & Getters`，讓我自己對這幾個 pattern 更熟悉了一些，不過還是要盡量運用在自己的專案當中，才能確切感受其好處，並更得心應手。也希望這些整理對來到這邊的讀者有所幫助，剩下的幾個 Pattern 會陸續補上，決不食言！

最後，再次呼籲大家去看看 Kent C. Dodds 的影片 [Advanced React Patterns](https://frontendmasters.com/courses/advanced-react-patterns/)，我比較推薦看 workshop 的版本，因為還可以聽到現場其他人對他的提問，以及他的回答，都很有幫助！

## 資料來源

1. [Advanced React Patterns workshop](https://frontendmasters.com/courses/advanced-react-patterns/)
2. [Advanced React Patterns V2 codesandbox](https://codesandbox.io/s/github/kentcdodds/advanced-react-patterns-v2)
3. [Advanced React Patterns Blog](https://blog.kentcdodds.com/advanced-react-component-patterns-56af2b74bc5f)
4. [什麼是設計模式](https://juejin.im/post/59b78dfe5188257e7e115cae)
5. [Answers to common questions about render props](https://blog.kentcdodds.com/answers-to-common-questions-about-render-props-a9f84bb12d5d)
6. [Do more with less using render props](https://hackernoon.com/do-more-with-less-using-render-props-de5bcdfbe74c)

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化