---
title: React 性能優化大挑戰：一次理解 Immutable data 跟 shouldComponentUpdate
date: 2018-01-05 18:48:27
tags: react
author: huli
---

前陣子正在重構公司的專案，試了一些東西之後發現自己對於 React 的渲染機制其實不太了解，不太知道 render 什麼時候會被觸發。而後來我發現不只我這樣，其實還有滿多人對這整個機制不太熟悉，因此決定寫這篇來分享自己的心得。

其實不知道怎麼優化倒還好，更慘的事情是你自以為在優化，其實卻在拖慢效能，而根本的原因就是對 React 的整個機制還不夠熟。被「優化」過的 component 反而還變慢了！這個就嚴重了。

因此，這篇文章會涵蓋到下面幾個主題：

1. Component 跟 PureComponent 的差異
2. shouldComponentUpdate 的作用
3. React 的渲染機制
4. 為什麼要用 Immutable data structures

為了判別你到底對以上這些理解多少，我們馬上進行幾個小測驗！有些有陷阱，請睜大眼睛看清楚啦！

# React 小測驗

## 第一題

以下程式碼是個很簡單的網頁，就一個按鈕跟一個叫做`Content`的元件而已，而按鈕按下去之後會改變`App`這個 component 的 state。

``` javascript
class Content extends React.Component {
  render () {
    console.log('render content!');
    return <div>Content</div>
  }
}
  
class App extends React.Component {
  handleClick = () => {
    this.setState({
      a: 1
    })
  }
  render() {
    console.log('render App!');
    return (
      <div>
        <button onClick={this.handleClick}>setState</button>
        <Content />
      </div>
    );
  }
}
  
ReactDOM.render(
  <App />,
  document.getElementById('container')
);
```

請問：當你按下按鈕之後，console 會輸出什麼？

A. 什麼都沒有（App 跟 Content 的 render function 都沒被執行到）
B. 只有 `render App!`（只有 App 的 render function 被執行到）
C. `render App!` 以及 `render content!`（兩者的 render function 都被執行到）

## 第二題

以下程式碼也很簡單，分成三個元件：App、Table 跟 Row，由 App 傳遞 list 給 Table，Table 再用 map 把每一個 Row 都渲染出來。

``` js
class Row extends Component {
  render () {
    const {item, style} = this.props;
    return (
      <tr style={style}>
        <td>{item.id}</td>
      </tr>
    )
  }
}
  
class Table extends Component {
  render() {
    const {list} = this.props;
    const itemStyle = {
      color: 'red'
    }
    return (
      <table>
          {list.map(item => <Row key={item.id} item={item} style={itemStyle} />)}
      </table>
    )
  }
}
  
class App extends Component {
  state = {
    list: Array(10000).fill(0).map((val, index) => ({id: index}))
  }
  
  handleClick = () => {
    this.setState({
      otherState: 1
    })
  }
  
  render() {
    const {list} = this.state;
    return (
      <div>
        <button onClick={this.handleClick}>change state!</button>
        <Table list={list} />
      </div>
    );
  }
}
```

而這段程式碼的問題就在於按下按鈕之後，`App`的 render function 被觸發，然後`Table`的 render function 也被觸發，所以重新渲染了一次整個列表。

可是呢，我們點擊按鈕之後，`list`根本沒變，其實是不需要重新渲染的，所以聰明的小明把 Table 從 Component 變成 PureComponent，只要 state 跟 props 沒變就不會重新渲染，變成下面這樣：

``` js
class Table extends PureComponent {
  render() {
    const {list} = this.props;
    const itemStyle = {
      color: 'red'
    }
    return (
      <table>
          {list.map(item => <Row key={item.id} item={item} style={itemStyle} />)}
      </table>
    )
  }
}
  
// 不知道什麼是 PureComponent 的朋友，可以想成他自己幫你加了下面的 function
shouldComponentUpdate (nextProps, nextState) {
  return !shallowEqual(this.props, nextProps) || !shallowEqual(this.state, nextState)
}
```

把 Table 從 Component 換成 PureComponent 之後，如果我們再做一次同樣的操作，也就是按下`change state`按鈕改變 App 的 state，這時候會提升效率嗎？

A. 會，在這情況下 PureComponent 會比 Component 有效率
B. 不會，兩者差不多
C. 不會，在這情況下 Component 會比 PureComponent 有效率

# 第三題

接著讓我來看一個跟上一題很像的例子，只是這次換成按按鈕以後會改變 list：

``` js
class Row extends Component {
  render () {
    const {item, style} = this.props;
    return (
      <tr style={style}>
        <td>{item.id}</td>
      </tr>
    )
  }
}
  
class Table extends PureComponent {
  render() {
    const {list} = this.props;
    const itemStyle = {
      color: 'red'
    }
    return (
      <table>
          {list.map(item => <Row key={item.id} item={item} style={itemStyle} />)}
      </table>
    )
  }
}
  
class App extends Component {
  state = {
    list: Array(10000).fill(0).map((val, index) => ({id: index}))
  }
  
  handleClick = () => {
    this.setState({
      list: [...this.state.list, 1234567] // 增加一個元素
    })
  }
  
  render() {
    const {list} = this.state;
    return (
      <div>
        <button onClick={this.handleClick}>change state!</button>
        <Table list={list} />
      </div>
    );
  }
}
```

這時候 Table 的 PureComponent 優化已經沒有用了，因為 list 已經變了，所以會觸發 render function。要繼續優化的話，比較常用的手段是把 Row 變成 PureComponent，這樣就可以確保相同的 Row 不會再次渲染。

``` js
class Row extends PureComponent {
  render () {
    const {item, style} = this.props;
    return (
      <tr style={style}>
        <td>{item.id}</td>
      </tr>
    )
  }
}
  
class Table extends PureComponent {
  render() {
    const {list} = this.props;
    const itemStyle = {
      color: 'red'
    }
    return (
      <table>
          {list.map(item => <Row key={item.id} item={item} style={itemStyle} />)}
      </table>
    )
  }
}
```

請問：把 Row 從 Component 換成 PureComponent 之後，如果我們再做一次同樣的操作，也就是按下`change state`按鈕改變 list，這時候會提升效率嗎？

A. 會，在這情況下 PureComponent 會比 Component 有效率
B. 不會，兩者差不多
C. 不會，在這情況下 Component 會比 PureComponent 有效率

# React 的 render 機制

在公布答案之前，先幫大家簡單複習一下 React 是如何把你的畫面渲染出來的。

首先，大家都知道你在`render`這個 function 裡面可以回傳你想渲染的東西，例如說：

``` js
class Content extends React.Component {
  render () {
    return <div>Content</div>
  }
}
```

要注意的是這邊 return 的東西不會直接就放到 DOM 上面去，而是會先經過一層 virtual DOM。其實你可以簡單把這個 virtual DOM 想成 JavaScript 的物件，例如說上面 Content render 出來的結果可能是：

``` js
{
  tagName: 'div',
  children: 'Content'
}
```

最後一步則是 React 進行 virtual DOM diff，把上次的跟這次的做比較，並且把變動的部分更新到真的 DOM 上面去。

簡單來說呢，就是在 React Component 以及 DOM 之間新增了一層 virtual DOM，先把你要渲染的東西轉成 virtual DOM，再把需要更新的東西 update 到真的 DOM 上面去。

如此一來，就能夠減少觸碰到真的 DOM 的次數並且提升性能。

舉個例子，假設我們實作一個非常簡單的，按一個按鈕之後就會改變 state 的小範例：

``` js
class Content extends React.Component {
  render () {
    return <div>{this.props.text}</div>
  }
}
  
class App extends React.Component {
  state = {
    text: 'hello'
  }
  handleClick = () => {
    this.setState({
      text: 'world'
    })
  }
  render() {
    return (
      <div>
        <button onClick={this.handleClick}>setState</button>
        <Content text={this.state.text} />
      </div>
    );
  }
}
```

在程式剛開始執行時，渲染的順序是這樣的：

1. 呼叫 App 的 render
2. 呼叫 Content 的 render
3. 拿到 virtual DOM
4. 跟上次的 virtual DOM 做比較
5. 把改變的地方應用到真的 DOM

這時候的 virtual DOM 整體應該會長得像這樣：

``` js
{
  tagName: 'div',
  children: [
    {
      tagName: 'button',
      children: 'setState'
    }, {
      tagName: 'div',
      children: 'hello'
    }
  ]
}
```

當你按下按鈕，改變 state 了以後，執行順序都跟剛剛一樣：

1. 呼叫 App 的 render
2. 呼叫 Content 的 render
3. 拿到 virtual DOM

這時候拿到的 virtual DOM 應該會長得像這樣：

``` js
{
  tagName: 'div',
  children: [
    {
      tagName: 'button',
      children: 'setState'
    }, {
      tagName: 'div',
      children: 'world' // 只有這邊變了
    }
  ]
}
```

而 React 的 virtual DOM diff 演算法，就會發現只有一個地方改變，然後把那邊的文字替換掉，其他部分都不會動到。

其實[官方文件](https://reactjs.org/docs/reconciliation.html#motivation)把這一段寫得很好：

> When you use React, at a single point in time you can think of the render() function as creating a tree of React elements. On the next state or props update, that render() function will return a different tree of React elements. React then needs to figure out how to efficiently update the UI to match the most recent tree.

大意就是你可以想像成 render function 會回傳一個 React elements 的 tree，然後 React 會把這次的 tree 跟上次的做比較，並且找出如何有效率地把這差異 update 到 UI 上面去。

所以說呢，如果你要成功更新畫面，你必須經過兩個步驟：

1. render function
2. virtual DOM diff

因此，要優化效能的話你有兩個方向，那就是：

1. 不要觸發 render function
2. 保持 virtual DOM 的一致

我們先從後者開始吧！

# 提升 React 效能：保持 virtual DOM 的一致

因為有了 virtual DOM 這一層的守護，通常你不必太擔心 React 的效能。

像是我們開頭問答的第一題：

``` js
class Content extends React.Component {
  render () {
    console.log('render content!');
    return <div>Content</div>
  }
}
  
class App extends React.Component {
  handleClick = () => {
    this.setState({
      a: 1
    })
  }
  render() {
    console.log('render App!');
    return (
      <div>
        <button onClick={this.handleClick}>setState</button>
        <Content />
      </div>
    );
  }
}
  
ReactDOM.render(
  <App />,
  document.getElementById('container')
);
```

你每次按下按鈕之後，由於 App 的 state 改變了，所以會先觸發 App 的 render function，而因為裡面有回傳`<Content />`，所以也會觸發 Content 的 render function。

因此你每按一次按鈕，這兩個 component 的 render function 就會個別被呼叫一次。所以答案是`C. render App! 以及 render content!（兩者的 render function 都被執行到）`

可是儘管如此，真的 DOM 不會有任何變化。因為在 virtual DOM diff 的時候，React 會發現你這次跟上次的 virtual DOM 長得一模一樣（因為沒有東西改變嘛），就不會對 DOM 做任何操作。

如果能盡量維持 virtual DOM 的結構相似的話，可以減少一些不必要的操作，在這點上其實可以做的優化還很多，可以參考[官方文件](https://reactjs.org/docs/reconciliation.html)，裡面寫的很詳細。

# 提升 React 效能：不要觸發 render function

雖然不必太過擔心，但是 virtual DOM diff 也是需要執行時間的。雖然說速度很快，但再快也比不上完全不呼叫來的快，你說是吧。

對於這種「我們已經明確知道不該有變化」的情形，我們連 render 都不該呼叫，因為沒必要嘛，再怎麼呼叫都是一樣的結果。如果 render 沒有被呼叫的話，連 virtual DOM diff 都不需要執行，又提升了一些性能。

你應該有聽過`shouldComponentUpdate`這個 function，就是來做這件事的。如果你在這個 function 中回傳 false，就不會重新呼叫 render function。

``` js
class Content extends React.Component {
  shouldComponentUpdate () {
    return false;
  }
  render () {
    console.log('render content!');
    return <div>Content</div>
  }
}
  
class App extends React.Component {
  handleClick = () => {
    this.setState({
      a: 1
    })
  }
  render() {
    console.log('render App!');
    return (
      <div>
        <button onClick={this.handleClick}>setState</button>
        <Content />
      </div>
    );
  }
}
```

加上去之後，你會發現無論你按多次按鈕，Content 的 render function 都不會被觸發。

但是這個東西請小心使用，一個不注意你就會碰到 state 跟 UI 搭不上的情形，例如說 state 明明變成 world，可是 UI 顯示的還是 Hello：

``` js
class Content extends React.Component {
  shouldComponentUpdate(){
    return false;
  }
  
  render () {
    return <div>{this.props.text}</div>
  }
}
  
class App extends React.Component {
  state = {
    text: 'hello'
  }
  handleClick = () => {
    this.setState({
      text: 'world'
    })
  }
  render() {
    return (
      <div>
        <button onClick={this.handleClick}>setState</button>
        <Content text={this.state.text} />
      </div>
    );
  }
}
```

在上面的例子中，按下按鈕之後 state 確實變成`world`，但是因為 Content 的`shouldComponentUpdate`永遠都回傳 false，所以不會再次觸發 render，就看不到對應的新的 state 的畫面了。

不過這有點極端，因為通常不會永遠都回傳 false，除非你真的確定這個 component 完全不需要 re-render。

比起這個，有一個更合理的判斷基準是：

> 如果每一個 props 跟 state 都沒有變，那就回傳 false

``` js
class Content extends React.Component {
  shouldComponentUpdate(nextProps, nextState){
    return !shallowEqual(this.props, nextProps) || !shallowEqual(this.state, nextState);
  }
  
  render () {
    return <div>{this.props.text}</div>
  }
}
```

假設`this.props`是：

``` js
{
  text: 'hello'
}
```

而`nextProps`是：

``` js
{
  text: 'world'
}
```

那在比較的時候就會發現`props.text`變了，就可以順理成章的呼叫 render function。還有另外一點是這邊用`shallowEqual`來比較前後的差異，而不是用`deepEqual`。

這是出於效能上的考量。別忘了，你要執行這樣的比較也是會吃資源的，尤其是在你的 object 很深很深的時候，要比較的東西可就多了，因此我們會傾向用`shallowEqual`，只要比較一層即可。

另外，前面有提到`PureComponent`這個東西，其實就是 React 提供的另外一種元件，差別就是在於它自動幫你加上上面那一段的比較。如果你想看原始碼的話，[在這邊](https://github.com/facebook/react/blob/1637b43e27c40c73f9489603145f9bb1d0ece618/packages/react-reconciler/src/ReactFiberClassComponent.js#L194)：

``` js
if (type.prototype && type.prototype.isPureReactComponent) {
  return (
    !shallowEqual(oldProps, newProps) || !shallowEqual(oldState, newState)
  );
}
```

講到這邊，就可以來公佈第二題的解答了，答案是：`A. 會，在這情況下 PureComponent 會比 Component 有效率`，因為繼承了 PureComponent 之後，只要 props 跟 state 沒變，就不會執行 render function，也不會執行 virtual DOM diff，節省了許多開銷。

# shallowEqual 與 Immutable data structures

你剛開始在學 React 的時候，可能會被告誡說如果要更改資料，不能夠這樣寫：

``` js
// 不能這樣
const newObject = this.state.obj
newObject.id = 2;
this.setState({
  obj: newObject
})
  
// 也不能這樣
const arr = this.state.arr;
arr.push(123);
this.setState({
  list: arr
})
```

而是應該要這樣：

``` js
this.setState({
  obj: {
    ...this.state.obj,
    id: 2
  }
})
  
this.setState({
  list: [...this.state.arr, 123]
})
```

但你知道為什麼嗎？

這個就跟我們上面講到的東西有關了。如同上面所述，其實使用`PureComponent`是一件很正常的事情，因為 state 跟 props 如果沒變的話，本來就不該觸發 render function。

而剛剛也提過`PureComponent`會幫你`shallowEqual` state 跟 props，決定要不要呼叫 render function。

在這種情況下，如果你用了一開始講的那種寫法，就會產生問題，例如說：

``` js
const newObject = this.state.obj
newObject.id = 2;
this.setState({
  obj: newObject
})
```

在上面的程式碼中，其實`this.state.obj`跟`newObject`還是指向同一個物件，指向同一塊記憶體，所以當我們在做`shallowEqual`的時候，就會判斷出這兩個東西是相等的，就不會執行 render function 了。

在這時候，我們就需要 Immutable data，Immutable 翻成中文就是永遠不變的，意思就是：「當一個資料被創建之後，就永遠不會變了」。那如果我需要更改資料的話怎麼辦呢？你就只能創一個新的。

``` js
const obj = {
  id: 1,
  text: 'hello'
}
  
obj.text = 'world' // 這樣不行，因為你改變了 obj 這個物件
  
// 你必須要像這樣創造一個新的物件
const newObj = {
  ...obj,
  text: 'world'
}
```

有了 Immutable 的概念之後，`shallowEqual`就不會出錯了，因為如果我們有新的資料，就可以保證它是一個新的 object，這也是為什麼我們在用`setState`的時候總是要產生一個新的物件，而不是直接對現有的做操作。

``` js
// 沒有 Immutable 的概念前 
const props = {
  id: 1,
  list: [1, 2, 3]
}
  
const list = props.list;
list.push(4)
nextProps = {
  ...props,
  list
}
  
props.list === nextProps.list // true
  
// 有了 Immutable 的概念後
const props = {
  id: 1,
  list: [1, 2, 3]
}
  
const nextProps = {
  ...props,
  list: [...props.list, 4]
}
  
props.list === nextProps.list // false
```

這邊還有一個要注意的地方，那就是 spread operator 只會複製第一層的資料而已，它並不是 deep clone：

``` js
const test = {
  a: 1,
  nest: {
    title: 'hello'
  }
}
  
const copy = {...test}
  
copy.nest === test.nest // true
```

所以當你的 state 是比較複雜的結構時，要改變資料就會變得比較麻煩一點，因為你必須要對每一層都做差不多的事情，避免直接去改到你要改的物件：

``` js
// 沒有 Immutable 的概念前 
const props = {
  title: '123',
  list: [
    {
      id: 1,
      name: 'hello'
    }, {
      id: 2,
      name: 'world'
    }
  ]
}
  
const list = props.list;
list[1].name = 'world2'; // 直接改
nextProps = {
  ...props,
  list
}
  
props.list === nextProps.list // true
props.list[1] === nextProps.list[1] // true
  
// 有了 Immutable 的概念後
const props = {
  title: '123',
  list: [
    {
      id: 1,
      name: 'hello'
    }, {
      id: 2,
      name: 'world'
    }
  ]
}
  
// 要注意這邊只是 shallow copy 而已
// list[0] === props.list[0] => true
const list = [...props.list.slice(0, 1)]
const data = props.list[1];
  
const nextProps = {
  ...props,
  list: [...list, {
    ...data, // 再做一次 spread oprator
    name: 'world2'
  }]
}
  
props.list === nextProps.list // false
props.list[0] === nextProps.list[0] // true
props.list[1] === nextProps.list[1] // false
```

若你的 state 結構很多層，那就會變得非常非常難改，這時候你有三個選擇：

1. 避免這麼多層的 state，盡量壓平（可參考[normalizr](https://github.com/paularmstrong/normalizr)）
2. 找一個會幫你做 Immutable 的 library，例如說 Facebook 的 [Immutable.js](https://facebook.github.io/immutable-js/)
3. 直接用 deep clone 把資料全部複製下來，之後你愛怎麼改就怎麼改（不推薦）

註：感謝網友 KanYueh Chen 的指正，讓我補上上面這一段。

# PureComponent 的陷阱

當我們遵守 Immutable 的規則之後，理所當然的就會想把所有的 Component 都設成 PureComponent，因為 PureComponent 的預設很合理嘛，資料沒變的話就不呼叫 render function，可以節省很多不必要的比較。

那讓我們回頭來看開場小測驗的最後一題：

``` js
class Row extends PureComponent {
  render () {
    const {item, style} = this.props;
    return (
      <tr style={style}>
        <td>{item.id}</td>
      </tr>
    )
  }
}
  
class Table extends PureComponent {
  render() {
    const {list} = this.props;
    const itemStyle = {
      color: 'red'
    }
    return (
      <table>
          {list.map(item => <Row key={item.id} item={item} style={itemStyle} />)}
      </table>
    )
  }
}
```

我們把`Row`變成了 PureComponent，所以只要 state 跟 props 沒變，就不會 re-render，所以答案應該要是`A. 會，在這情況下 PureComponent 會比 Component 有效率`？

錯，如果你把程式碼看更清楚一點，你會發現答案其實是`C. 不會，在這情況下 Component 會比 PureComponent 有效率`。

你的前提是對的，「只要 state 跟 props 沒變，就不會 re-render，PureComponent 就會比 Component 更有效率」。但其實還有另外一句話也是對的：「如果你的 state 或 props 『永遠都會變』，那 PureComponent 並不會比較快」。

所以這兩種的使用時機差異在於：state 跟 props 到底常常會變還是不會變？

上述的例子中，陷阱在於`itemStyle`這個 props，我們每次 render 的時候都創建了一個新的物件，所以對 Row 來說，儘管 props.item 是一樣的，但是 props.style 卻是「每次都不一樣」。

如果你已經知道每次都會不一樣，那 PureComponent 這時候就無用武之地了，而且還更糟。為什麼？因為它幫你做了`shallowEqual`。

別忘記了，`shallowEqual`也是需要執行時間的。

已經知道 props 的比較每次都失敗的話，那不如不要比還會來的比較快，所以在這個情形下，Component 會比 PureComponent 有效率，因為不用做`shallowEqual`。

這就是我開頭提到的需要特別注意的部分。不要以為你把每個 Component 都換成 PureComponent 就天下太平，App 變超快，效能提升好幾倍。不去注意這些細節的話，就有可能把效能越弄越糟。

最後再強調一次，如果你已經預期到某個 component 的 props 或是 state 會「很頻繁變動」，那你根本不用換成 PureComponent，因為你實作之後反而會變得更慢。

# 總結

在研究這些效能相關的問題時，我最推薦這篇：[React, Inline Functions, and Performance](https://cdb.reacttraining.com/react-inline-functions-and-performance-bdff784f5578)，解開了很多我心中的疑惑以及帶給我很多新的想法。

例如說文末提到的 PureComponent 有時候反而會變慢，也是從這篇文章看來的，真心推薦大家抽空去看看。

前陣子跟同事一起把一個專案打掉重做，原本的共識是盡量用 PureComponent，直到我看到這篇文並且仔細思考了一下，發現如果你不知道背後的原理，還是不要輕易使用比較好。因此我就提議改成全部用 Component，等我們碰到效能問題要來優化時再慢慢調整。

最後附上一句我很喜歡的話，從[React 巢狀 Component 效能優化](https://blog.wuct.me/react-%E5%B7%A2%E7%8B%80-component-%E6%95%88%E8%83%BD%E5%84%AA%E5%8C%96-b01d8a0d3eff)這篇看來的（這篇也是在講最後提到的 PureComponent 的問題）：

> 雖然你知道可以優化，但不代表你應該優化。

參考資料：
[High Performance React: 3 New Tools to Speed Up Your Apps](https://medium.freecodecamp.org/make-react-fast-again-tools-and-techniques-for-speeding-up-your-react-app-7ad39d3c1b82)
[reactjs - Reconciliation](https://reactjs.org/docs/reconciliation.html#motivation)
[reactjs- Optimizing Performance](https://reactjs.org/docs/optimizing-performance.html)
[React is Slow, React is Fast: Optimizing React Apps in Practice](https://marmelab.com/blog/2017/02/06/react-is-slow-react-is-fast.html)
[Efficient React Components: A Guide to Optimizing React Performance](https://www.toptal.com/react/optimizing-react-performance)

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好