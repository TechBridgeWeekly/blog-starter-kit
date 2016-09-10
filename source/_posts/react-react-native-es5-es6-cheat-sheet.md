---
title: 一看就懂的 React ES5、ES6+ 常見用法對照表
date: 2016-04-04 23:26:00
tags: React, React Native, ES5, ES6, ES7, JavaScript, ECMAScript2015, Webpack
author: kdchang
---

![一看就懂的 React ES5、ES6+ 常見用法對照表](/img/kdchang/react-es6.jpg)

## 前言
[React](https://facebook.github.io/react/) 是 Facebook 推出的開源 [JavaScript](https://en.wikipedia.org/wiki/JavaScript) Library。自從 React 正式開源後，React 生態系開始蓬勃發展。事實上，透過學習 React 生態系（ecosystem）的過程中，可以讓我們順便學習現代化 Web 開發的重要觀念（例如：ES6、[Webpack](https://github.com/webpack/webpack)、[Babel](https://babeljs.io/)、模組化等），成為更好的開發者。雖然 ES6（ECMAScript2015）、ES7 是未來趨勢（本文將 ES6、ES7 稱為 ES6+），然而目前在網路上有許多的學習資源仍是以 ES5 為主，導致讀者在學習上遇到一些坑洞和迷惑（本文假設讀者對於 React 已經有些基本認識，若你對於 React 尚不熟悉，建議先行[閱讀官方文件](https://facebook.github.io/react/)和[本篇入門教學](https://scotch.io/tutorials/learning-react-getting-started-and-concepts)）。因此本文希望透過整理在 React 中 ES5、ES6+ 常見用法對照表，讓讀者們可以在實現功能時（尤其在 [React Native](https://facebook.github.io/react-native/)）可以更清楚兩者的差異，無痛轉移到 ES6+。 

## 大綱
1. Modules
2. Classes
3. Method definition
4. Property initializers
5. State
6. Arrow functions
7. Dynamic property names & template strings
8. Destructuring & spread attributes
9. Mixins
10. Default Parameters

## 1. Modules
隨著 Web 技術的進展，模組化開發已經成為一個重要課題。關於 JavaScript 模組化我們這邊不詳述，建議讀者參考 [這份投影片](http://huangxuan.me/js-module-7day/#/) 和 [這篇文章](http://justineo.github.io/singles/writing-modular-js/)。

ES5 若使用 CommonJS 標準，一般使用 `require()` 用法引入模組：

```js
var React = require('react');
var MyComponent = require('./MyComponent');
```

輸出則是使用 `module.exports`：

```js
module.exports = MyComponent;
```


ES6+ `import` 用法：

```js
import React from 'react';
import MyComponent from './MyComponent';
```

輸出則是使用 `export default`：

```js
export default class MyComponent extends React.Compoent {
	
}
```


## 2. Classes
在 React 中元件（Component）是組成視覺頁面的基礎。在 ES5 中我們使用 `React.createClass()` 來建立 Component，而在 ES6+ 則是用 [Classes](https://babeljs.io/docs/learn-es2015/#classes) 繼承 `React.Component` 來建立 Component。若是有寫過 Java 等物件導向語言（OOP）的讀者應該對於這種寫法比較不陌生，不過要注意的是 JavaScript 仍是原型繼承類型的物件導向程式語言，只是使用 `Classes` 讓物件導向使用上更加直觀。對於選擇 `class` 使用上還有疑惑的讀者建議可以閱讀 [React.createClass versus extends React.Component](https://toddmotto.com/react-create-class-versus-component/) 這篇文章。

ES5 `React.createClass()` 用法：

```js 
var Photo = React.createClass({
  render: function() {
    return (
    	<div>
    		<img alt={this.props.description} src={this.props.src} />
    	</div>
    	);
  }
});
ReactDOM.render(<Photo />, document.getElementById('main'));
```

ES6+ `class` 用法：

```js 
class Photo extends React.Component {
  render() {
    return <img alt={this.props.description} src={this.props.src} />;
  }
}
ReactDOM.render(<Photo />, document.getElementById('main'));
```

在 ES5 我們會在 `componentWillMount ` 生命週期定義希望在 `render` 前執行，且只會執行一次的任務：
 
```js
var Photo = React.createClass({
  componentWillMount: function() {}
});
```

在 ES6+ 則是定義在 `constructor ` 建構子中：

```js
class Photo extends React.Component {
  constructor(props) {
    super(props);
    // 原本在 componentWillMount 操作的動作可以放在這
  }
}
```

## 3. Method definition
在 ES6 中我們使用 `Method` 可以忽略 `function` 和 `,`，使用上更為簡潔！ES5 `React.createClass()` 用法：

```js
var Photo = React.createClass({
  handleClick: function(e) {},
  render: function() {}
});
```

ES6+ class 用法：

```js
class Photo extends React.Component {
  handleClick(e) {}
  render() {}
}
```


## 4. Property initializers
Component 屬性值是資料傳遞重要的元素，在 ES5 中我們使用 `propTypes ` 和  `getDefaultProps ` 來定義屬性（props）的預設值和型別：

```js
var Todo = React.createClass({
  getDefaultProps: function() {
    return {
      checked: false,
      maxLength: 10,
    };
  },
  propTypes: {
    checked: React.PropTypes.bool.isRequired,
    maxLength: React.PropTypes.number.isRequired
  },
  render: fucntion() {
  	return();
  }
});
```

在 ES6+ 中我們則是參考 [ES7 property initializers](https://github.com/jeffmo/es-class-fields-and-static-properties) 使用 `class` 中的靜態屬性（static properties）來定義：

```js
class Todo extends React.Component {
  static defaultProps = {
    checked: false,
    maxLength: 10,
  }; // 注意有分號
  static propTypes = {
    checked: React.PropTypes.bool.isRequired,
    maxLength: React.PropTypes.number.isRequired
  };
  render() {
  	return();
  }
}
```

ES6+ 另外一種寫法，可以留意一下，主要是看各團隊喜好和規範，選擇合適的方式：

```js
class Todo extends React.Component {
    render() {
        return (
            <View />
        );
    }
}
Todo.defaultProps = {
    checked: false,
    maxLength: 10,
};
Todo.propTypes = {
    checked: React.PropTypes.bool.isRequired,
    maxLength: React.PropTypes.number.isRequired,
};
```


## 5. State
在 React 中 `Props` 和 `State` 是資料流傳遞的重要元素，不同的是 `state` 可更動，可以去執行一些運算。在 ES5 中我們使用 `getInitialState ` 去初始化 `state`： 
 
```js
var Todo = React.createClass({
    getInitialState: function() {
        return {
            maxLength: this.props.maxLength,
        };
    },
});
```

在 ES6+ 中我們初始化 `state` 有兩種寫法：

```js
class Todo extends React.Component {
    state = {
        maxLength: this.props.maxLength,
    }
}
```

另外一種寫法，使用在建構式初始化。比較推薦使用這種方式，方便做一些運算：

```js
class Todo extends React.Component {
    constructor(props){
        super(props);
        this.state = {
            maxLength: this.props.maxLength,
        };
    }
}
```


## 6. Arrow functions

在講 `Arrow functions` 之前，我們先聊聊在 React 中 `this` 和它所代表的 `context`。在 ES5 中，我們使用 `React.createClass()` 來建立 Component，而在 `React.createClass()` 下，預設幫你綁定好 `method` 的 `this`，你毋須自行綁定。所以你可以看到像是下面的例子，`callback function` handleButtonClick 中的  `this` 是指到 component 的實例（instance），而非觸發事件的物件：

```js
var TodoBtn = React.createClass({
    handleButtonClick: function(e) {
        // 此 this 指到 component 的實例（instance），而非 button
        this.setState({showOptionsModal: true});
    },
    render: function(){
        return (
            <div>
                <Button onClick={this.handleButtonClick}>{this.props.label}</Button>
            </div>
        )
    },
});
```

然而自動綁定這種方式反而會讓人容易誤解，所以在 ES6+ 推薦使用 `bind ` 綁定 `this` 或使用 `Arrow functions`（它會绑定當前 `scope` 的 `this context`）兩種方式，你可以參考下面例子：

```js
class TodoBtn extends React.Component
{
    handleButtonClick(e){
        // 確認綁定 this 指到 component instance
        this.setState({toggle: true});
    }
    render(){
        // 這邊可以用 this.handleButtonClick.bind(this) 手動綁定或是 Arrow functions () => {} 用法
        return (
            <div>
                <Button onClick={this.handleButtonClick.bind(this)} onClick={(e)=> {this.handleButtonClick(e)} }>{this.props.label}</Button>
            </div>
        )
    },
}
```

`Arrow functions` 雖然一開始看起來有點怪異，但其實觀念很簡單：一個簡化的函數。函數基本上就是參數（不一定要有參數）、表達式、回傳值（也可能是回傳 undefined）：

```
// Arrow functions 的一些例子
()=>7
e=>e+2
()=>{
    alert('XD');
}
(a,b)=>a+b
e=>{
    if (e == 2){
        return 2;
    }
    return 100/e;
}
```

不過要注意的是無論是 `bind` 或是 `Arrow functions`，每次執行回傳都是指到一個新的函數，若需要再調用到這個函數，請記得先把它存起來：

錯誤用法：

```js
class TodoBtn extends React.Component{
    componentWillMount(){
        Btn.addEventListener('click', this.handleButtonClick.bind(this));
    }
    componentDidmount(){
        Btn.removeEventListener('click', this.handleButtonClick.bind(this));
    }
    onAppPaused(event){
    }
}
```

正確用法：

```js
class TodoBtn extends React.Component{
    constructor(props){
        super(props);
        this.handleButtonClick = this.handleButtonClick.bind(this);
    }
    componentWillMount(){
        Btn.addEventListener('click', this.handleButtonClick);
    }
    componentDidmount(){
        Btn.removeEventListener('click', this.handleButtonClick);
    }
}
```

更多 Arrows and Lexical This 特性可以[參考這個文件](https://babeljs.io/docs/learn-es2015/#arrows)。


## 7. Dynamic property names & template strings
以前在 ES5 我們要動態設定屬性名稱時，往往需要多寫幾行程式碼才能達到目標：

```js
var Todo = React.createClass({
  onChange: function(inputName, e) {
    var stateToSet = {};
    stateToSet[inputName + 'Value'] = e.target.value;
    this.setState(stateToSet);
  },
});
```

但在 ES6+中，透過 [enhancements to object literals](https://babeljs.io/blog/2015/06/07/react-on-es6-plus) 和 [template strings](https://babeljs.io/docs/learn-es2015/#template-strings) 可以輕鬆完成動態設定屬性名稱的任務：

```
class Todo extends React.Component {
  onChange(inputName, e) {
    this.setState({
      [`${inputName}Value`]: e.target.value,
    });
  }
}
```

Template Strings 是一種語法糖（syntactic sugar），方便我們組織字串（這邊也用上 `let`、`const` 變數和常數宣告的方式，和 `var` 的 `function scope` 不同的是它們是屬於 `block scope`，亦即生存域存在於 `{}` 間）：

```js
// Interpolate variable bindings
const name = "Bob", let = "today";
`Hello ${name}, how are you ${time}?` \\ Hello Bob, how are you today?
```


## 8. Destructuring & spread attributes
在 React 的 Component 中，父元件利用 `props` 來傳遞資料到子元件是常見作法，然而我們有時會希望只傳遞部分資料，此時 ES6+ 中的 [Destructuring](https://babeljs.io/docs/learn-es2015/#destructuring) 和 [JSX 的 Spread Attributes
](https://facebook.github.io/react/docs/jsx-spread.html) ，`...` Spread Attributes 主要是用來迭代物件：

```js
class Todo extends React.Component {
  render() {
    var {
      className,
      ...others,  // ...others 包含 this.props 除了 className 外所有值。this.props = {value: 'true', title: 'header', className: 'content'}
    } = this.props;
    return (
      <div className={className}>
        <TodoList {...others} />
        <button onClick={this.handleLoadMoreClick}>Load more</button>
      </div>
    );
  }
}
```

但使用上要注意的是若是有重複的屬性值則以後來覆蓋，下面的例子中若 `...this.props`，有 `className`，則被後來的 `main` 所覆蓋：

```js
<div {...this.props} className="main">
  …
</div>
```

而 `Destructuring` 也可以用在簡化 `Module` 的引入上，這邊我們先用 ES5 中引入方式來看：

```js
var React = require('react-native');
var Component = React.component;
 
class HelloWorld extends Component {
  render() {
    return (
      <View>
        <Text>Hello, world!</Text>
      </View>
    );
  }
}
 
export default HelloWorld;
```

以下 ES5 寫法：

```
var React = require('react-native');
var View = React.View;
```

在 ES6+ 則可以直接使用 `Destructuring` 這種簡化方式來引入模組中的元件：

```
// 這邊等於上面的寫法
var { View } = require('react-native');
```

更進一步可以使用 `import` 語法：

```js
import React, {
	View,
	Component,
	Text,
} from 'react-native';

class HelloWorld extends Component {
  render() {
    return (
      <View>
        <Text>Hello, world!</Text>
      </View>
    );
  }
}
 
export default HelloWorld;
```


## 9. Mixins
在 ES5 中，我們可以使用 `Mixins` 的方式去讓不同的 Component 共用相似的功能，重用我們的程式碼：

```js
var PureRenderMixin = require('react-addons-pure-render-mixin');
React.createClass({
  mixins: [PureRenderMixin],

  render: function() {
    return <div className={this.props.className}>foo</div>;
  }
});
```

但由於官方不打算在 ES6+ 中繼續推行 `Mixins`，若還是希望使用，可以參考看看[第三方套件](https://www.npmjs.com/package/es6-class-mixin)或是[這個文件的用法](https://gist.github.com/sebmarkbage/ef0bf1f338a7182b6775)。

## 10. Default Parameters
以前 ES5 我們函數要使用預設值需要這樣使用：

```js
var link = function (height, color) {  
    var height = height || 50;  
    var color = color || 'red';  
}  
```

現在 ES6+ 的函數可以支援預設值，讓程式碼更為簡潔：

```js
var link = function(height = 50, color = 'red') {  
  ...  
} 
```

## 總結
以上就是 React ES5、ES6+常見用法對照表，能看到這邊的你應該已經對於 React ES5、ES6 使用上有些認識，先給自己一些掌聲吧！確實從 ES6 開始，JavaScript 和以前我們看到的 JavaScript 有些不同，增加了許多新的特性，有些讀者甚至會很懷疑說這真的是 JavaScript 嗎？ES6 的用法對於初學者來說可能會需要寫一點時間吸收，下面我幫大家準備了延伸閱讀，方便大家進一步參考學習。接下來我們也會有一系列從零開始學 React 的文章來帶大家由淺入深學習 ReactJS 生態系，運用 JavaScript 打造跨平台應用程式。筆者才書學淺，若有疏漏歡迎大家一起交流討論：）

## 延伸閱讀
1. [React/React Native 的ES5 ES6写法对照表](http://bbs.reactnative.cn/topic/15/react-react-native-%E7%9A%84es5-es6%E5%86%99%E6%B3%95%E5%AF%B9%E7%85%A7%E8%A1%A8)
2. [React on ES6+](https://babeljs.io/blog/2015/06/07/react-on-es6-plus)
3. [react native 中es6语法解析](http://www.ghugo.com/react-native-es6/) 
4. [Learn ES2015](https://babeljs.io/docs/learn-es2015/)
5. [ECMAScript 6入门](http://es6.ruanyifeng.com/)
6. [React官方網站](https://facebook.github.io/react/index.html)
7. [React INTRO TO REACT.JS](http://fraserxu.me/intro-to-react/)
8. [React.createClass versus extends React.Component](https://toddmotto.com/react-create-class-versus-component/)
9. [react-native-coding-style](https://github.com/lzbSun/react-native-coding-style)
10. [Airbnb React/JSX Style Guide](https://github.com/airbnb/javascript/tree/master/react)
11. [從零開始學 React（ReactJS 101）](https://www.gitbook.com/book/kdchang/react101/details)

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校，目前專注在 Mobile 和 IoT 應用開發。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 