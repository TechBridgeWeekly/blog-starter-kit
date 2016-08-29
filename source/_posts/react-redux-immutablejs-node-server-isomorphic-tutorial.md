---
title: 一看就懂的 React Server Rendering（Isomorphic JavaScript）入門教學
date: 2016-08-27 23:26:00
tags: React, Redux, Server Rendering, Isomorphic, JavaScript, Node, Express, Universal JavaScript, Isomorphic JavaScript
author: kdchang
---

![React Redux Sever Rendering（Isomorphic）入門](/img/kdchang/isomorphic-javascript.png "React Redux Sever Rendering（Isomorphic）入門")

## 前言
由於可能有些讀者沒聽過 [Isomorphic JavaScript](http://isomorphic.net/) 。因此在進到開發 React Redux Sever Rendering 應用程式的主題之前我們先來聊聊 Isomorphic JavaScript 這個議題。

根據 [Isomorphic JavaScript](http://isomorphic.net/) 這個網站的說明：

>Isomorphic JavaScript apps are JavaScript applications that can run both client-side and server-side.
The backend and frontend share the same code. 

Isomorphic JavaScript 係指瀏覽器端和伺服器端共用 JavaScript 的程式碼。

另外，除了 Isomorphic JavaScript 外，讀者或許也有聽過 Universal JavaScript 這個用詞。那什麼是 Universal JavaScript 呢？它和 Isomorphic JavaScript 是指一樣的意思嗎？針對這個議題網路上有些開發者提出了自己的觀點： [Universal JavaScript](https://medium.com/@mjackson/universal-javascript-4761051b7ae9#.67xsay73m)、[Isomorphism vs Universal JavaScript](https://medium.com/@ghengeveld/isomorphism-vs-universal-javascript-4b47fb481beb#.qvggcp3v8)。其中 Isomorphism vs Universal JavaScript 這篇文章的作者 Gert Hengeveld 指出 `Isomorphic JavaScript` 主要是指前後端共用 JavaScript 的開發方式，而 `Universal JavaScript` 是指 JavaScript 程式碼可以在不同環境下運行，這當然包含瀏覽器端和伺服器端，甚至其他環境。也就是說 `Universal JavaScript` 在意義上可以涵蓋的比 `Isomorphic JavaScript` 更廣泛一些，然而在 Github 或是許多技術討論上通常會把兩者視為同一件事情，這部份也請讀者留意。

## Isomorphic JavaScript 的好處
在開始真正撰寫 Isomorphic JavaScript 前我們在進一步探討使用 Isomorphic JavaScript 有哪些好處？在談好處之前，我們先看看最早 Web 開發是如何處理頁面渲染和 state 管理，還有遇到哪些挑戰。

最早的時候我們談論 Web 很單純，都是由 Server 端進行模版的處理，你可以想成 template 是一個函數，我們傳送資料進去，template 最後產生一張 HTML 給瀏覽器顯示。例如：Node 使用的（[EJS](http://ejs.co/)、[Jade](http://jade-lang.com/)）、Python/Django 的 [Template](https://docs.djangoproject.com/el/1.10/ref/templates/) 或替代方案 [Jinja](https://github.com/pallets/jinja)、PHP 的 [Smarty](http://www.smarty.net/)、[Laravel](https://laravel.com/) 使用的 [Blade](https://laravel.com/docs/5.0/templates)，甚至是 Ruby on Rails 用的 [ERB](http://guides.rubyonrails.org/layouts_and_rendering.html)。都是由後端去 render 所有資料和頁面，前端處理相對單純。

然而隨著前端工程的軟體工程化和使用者體驗的要求，開始出現各式前端框架的百花齊放，例如：[Backbone.js](http://backbonejs.org/)、[Ember.js](http://emberjs.com/) 和 [Angular.js](https://angularjs.org/) 等前端 MVC (Model-View-Controller) 或 MVVM (Model-View-ViewModel) 框架，將頁面於前端渲染的不刷頁單頁式應用程式（Single Page App）也因此開始流行。

後端除了提供初始的 HTML 外，還提供 API Server 讓前端框架可以取得資料用於前端 template。複雜的邏輯由 ViewModel/Presenter 來處理，前端 template 只處理簡單的是否顯示或是元素迭代的狀況，如下圖所示：

![React Redux Sever Rendering（Isomorphic）入門](/img/kdchang/client-mvc.png "React Redux Sever Rendering（Isomorphic）入門")

然而前端渲染 template 雖然有它的好處但也遇到一些問題包括效能、SEO 等議題。此時我們就開始思考 Isomorphic JavaScript 的可能性：為什麼我們不能前後端都使用 JavaScript 甚至是 React？

![React Redux Sever Rendering（Isomorphic）入門](/img/kdchang/isomorphic-api.png "React Redux Sever Rendering（Isomorphic）入門")

事實上，React 的優勢就在於它可以很優雅地實現 Server Side Rendering 達到 Isomorphic JavaScript 的效果。在 `react-dom/server` 中有兩個方法 `renderToString` 和 `renderToStaticMarkup` 可以在 server 端渲染你的 components。其主要都是將 React Component 在 Server 端轉成 DOM String，也可以將 props 往下傳，然而事件處理會失效，要到 client-side 的 React 接收到後才會把它加上去（但要注意 server-side 和 client-side 的 checksum 要一致不然會出現錯誤），這樣一來可以提高渲染速度和 SEO 效果。`renderToString` 和 `renderToStaticMarkup` 最大的差異在於 `renderToStaticMarkup` 會少加一些 React 內部使用的 DOM 屬性，例如：`data-react-id`，因此可以節省一些資源。

使用 `renderToString` 進行 Server 端渲染：

```javascript
import ReactDOMServer from 'react-dom/server';

ReactDOMServer.renderToString(<HelloButton name="Mark" />);
```

渲染出來的效果：

```html
<button data-reactid=".7" data-react-checksum="762752829">
  Hello, Mark
</button>
```

總的來說使用 Isomorphic JavaScript 會有以下的好處：

1. 有助於 SEO
2. Rendering 速度較快，效能較佳
3. 放棄蹩腳的 Template 語法擁抱 Component 元件化思考，便於維護
4. 盡量前後端共用程式碼節省開發時間

不過要注意的是如果有使用 Redux 在 Server Side Rendering 中，其流程相對複雜，不過大致流程如下：
由後端預先載入需要的 initialState，由於 Server 渲染必須全部都轉成 string，所以先將 state 先 dehydration（脫水），等到 client 端再 rehydration（覆水），重建 store 往下傳到前端的 React Component。

而要把資料從伺服器端傳遞到客戶端，我們需要：

1. 把取得初始 state 當做參數並對每個請求建立一個全新的 Redux store 實體
2. 選擇性地 dispatch 一些 action 
3. 把 state 從 store 取出來
4. 把 state 一起傳到客戶端

接下來我們就開始動手實作一個簡單的 React Server Side Rendering Counter 應用程式。

## 專案成果截圖

![React Redux Sever Rendering（Isomorphic）入門](/img/kdchang/react-server-rendering-demo.png "React Redux Sever Rendering（Isomorphic）入門")

## 環境安裝與設定
1. 安裝 Node 和 NPM

2. 安裝所需套件

	```
	$ npm install --save react react-dom redux react-redux react-router immutable redux-immutable redux-actions redux-thunk babel-polyfill babel-register body-parser express morgan qs
	```

	```
	$ npm install --save-dev babel-core babel-eslint babel-loader babel-preset-es2015 babel-preset-react babel-preset-stage-1 eslint eslint-config-airbnb eslint-loader eslint-plugin-import eslint-plugin-jsx-a11y eslint-plugin-react html-webpack-plugin webpack webpack-dev-server redux-logger
	```

接下來我們先設定一下開發文檔。

1. 設定 Babel 的設定檔： `.babelrc`

	```javascript
	{
		"presets": [
	  	"es2015",
	  	"react",
	 	],
		"plugins": []
	}
	```

2. 設定 ESLint 的設定檔和規則： `.eslintrc`

	```javascript
	{
	  "extends": "airbnb",
	  "rules": {
	    "react/jsx-filename-extension": [1, { "extensions": [".js", ".jsx"] }],
	  },
	  "env" :{
	    "browser": true,
	  }
	}
	```

3. 設定 Webpack 設定檔： `webpack.config.js`

	```javascript
	// 讓你可以動態插入 bundle 好的 .js 檔到 .index.html
	const HtmlWebpackPlugin = require('html-webpack-plugin');

	const HTMLWebpackPluginConfig = new HtmlWebpackPlugin({
	  template: `${__dirname}/src/index.html`,
	  filename: 'index.html',
	  inject: 'body',
	});
	
	// entry 為進入點，output 為進行完 eslint、babel loader 轉譯後的檔案位置
	module.exports = {
	  entry: [
	    './src/index.js',
	  ],
	  output: {
	    path: `${__dirname}/dist`,
	    filename: 'index_bundle.js',
	  },
	  module: {
	    preLoaders: [
	      {
	        test: /\.jsx$|\.js$/,
	        loader: 'eslint-loader',
	        include: `${__dirname}/src`,
	        exclude: /bundle\.js$/
	      }
	    ],
	    loaders: [{
	      test: /\.js$/,
	      exclude: /node_modules/,
	      loader: 'babel-loader',
	      query: {
	        presets: ['es2015', 'react'],
	      },
	    }],
	  },
	  // 啟動開發測試用 server 設定（不能用在 production）
	  devServer: {
	    inline: true,
	    port: 8008,
	  },
	  plugins: [HTMLWebpackPluginConfig],
	};
	```

太好了！這樣我們就完成了開發環境的設定可以開始動手實作 `React Server Side Rendering Counter` 應用程式了！	

先看一下我們整個專案的資料結構，我們把整個專案分成三個主要的資料夾（`client`、`server`，還有共用程式碼的 `common`）：

![React Redux Sever Rendering（Isomorphic）入門](/img/kdchang/react-server-rendering-folder.png "React Redux Sever Rendering（Isomorphic）入門")

## 動手實作

首先，我們先定義了 `client` 的 `index.js`：

```javascript
// 引用 babel-polyfill 避免瀏覽器不支援部分 ES6 用法
import 'babel-polyfill';
import React from 'react';
import ReactDOM from 'react-dom';
import { Provider } from 'react-redux';
import CounterContainer from '../common/containers/CounterContainer';
import configureStore from '../common/store/configureStore'
import { fromJS } from 'immutable';

// 從 server 取得傳進來的 initialState。由於從字串轉回物件，又稱為 rehydration（覆水） 
const initialState = window.__PRELOADED_STATE__;

// 由於我們使用 ImmutableJS，所以需要把在 server-side dehydration（脫水）又在前端 rehydration（覆水）的 initialState 轉成 ImmutableJS 資料型態，並傳進 configureStore 建立 store
const store = configureStore(fromJS(initialState));

// 接下來就跟一般的 React App 一樣，把 store 透過 Provider 往下傳到 Component 中
ReactDOM.render(
  <Provider store={store}>
    <CounterContainer />
  </Provider>,
  document.getElementById('app')
);

```

由於 Node 端要到新版對於 ES6 支援較好，所以先用 `babel-register` 在 `src/server/index.js` 去即時轉譯 `server.js`，但目前不建議在 `production` 環境使用。

```javascript
// use babel-register to precompile ES6 syntax
require('babel-register');
require('./server');
```

接著是我們 `server` 端，也是這個範例最重要的一個部分。首先我們用 `express` 建立了一個 port 為 3000 的 server，並使用 webpack 去執行 `client` 的程式碼。這個範例中我們使用了 `handleRender` 當 request 進來時（直接拜訪頁面或重新整理）就會執行 fetchCounter() 進行處理：

```javascript
import Express from 'express';
import qs from 'qs';

import webpack from 'webpack';
import webpackDevMiddleware from 'webpack-dev-middleware';
import webpackHotMiddleware from 'webpack-hot-middleware';
import webpackConfig from '../webpack.config';

import React from 'react';
import { renderToString } from 'react-dom/server';
import { Provider } from 'react-redux';
import { fromJS } from 'immutable';

import configureStore from '../common/store/configureStore';
import CounterContainer from '../common/containers/CounterContainer';

import { fetchCounter } from '../common/api/counter';

const app = new Express();
const port = 3000;

function handleRender(req, res) {
  // 模仿實際非同步 api 處理情形
  fetchCounter(apiResult => {
  // 讀取 api 提供的資料（這邊我們 api 是用 setTimeout 進行模仿非同步狀況），若網址參數有值擇取值，若無則使用 api 提供的隨機值，若都沒有則取 0
    const params = qs.parse(req.query);
    const counter = parseInt(params.counter, 10) || apiResult || 0;
    // 將 initialState 轉成 immutable 和符合 state 設計的格式 
    const initialState = fromJS({
      counterReducers: {
        count: counter,
      }
    });
    // 建立一個 redux store
    const store = configureStore(initialState);
    // 使用 renderToString 將 component 轉為 string
    const html = renderToString(
      <Provider store={store}>
        <CounterContainer />
      </Provider>
    );
    // 從建立的 redux store 中取得 initialState
    const finalState = store.getState();
    // 將 HTML 和 initialState 傳到 client-side
    res.send(renderFullPage(html, finalState));
  })
}

// HTML Markup，同時也把 preloadedState 轉成字串（stringify）傳到 client-side，又稱為 dehydration（脫水）
function renderFullPage(html, preloadedState) {
  return `
    <!doctype html>
    <html>
      <head>
        <title>Redux Universal Example</title>
      </head>
      <body>
        <div id="app">${html}</div>
        <script>
          window.__PRELOADED_STATE__ = ${JSON.stringify(preloadedState).replace(/</g, '\\x3c')}
        </script>
        <script src="/static/bundle.js"></script>
      </body>
    </html>
    `
}

// 使用 middleware 於 webpack 去進行 hot module reloading 
const compiler = webpack(webpackConfig);
app.use(webpackDevMiddleware(compiler, { noInfo: true, publicPath: webpackConfig.output.publicPath }));
app.use(webpackHotMiddleware(compiler));
// 每次 server 接到 request 都會呼叫 handleRender
app.use(handleRender);

// 監聽 server 狀況
app.listen(port, (error) => {
  if (error) {
    console.error(error)
  } else {
    console.info(`==> 🌎  Listening on port ${port}. Open up http://localhost:${port}/ in your browser.`)
  }
});
```

處理完 Server 的部份接下來我們來處理 actions 的部份，在這個範例中 actions 相對簡單，主要就是新增和減少兩個行為，以下為 `src/actions/counterActions.js`：

```javascript
import { createAction } from 'redux-actions';
import {
  INCREMENT_COUNT,
  DECREMENT_COUNT,
} from '../constants/actionTypes';

export const incrementCount = createAction(INCREMENT_COUNT);
export const decrementCount = createAction(DECREMENT_COUNT);
```

以下為輸出常數 `src/constants/actionTypes.js`：

```javascript
export const INCREMENT_COUNT = 'INCREMENT_COUNT';  
export const DECREMENT_COUNT = 'DECREMENT_COUNT';  
```

在這個範例中我們使用 `setTimeout()` 來模擬非同步的產生資料讓 server 端在每次接收 request 時讀取隨機產生的值。實務上，我們會開 API 讓 Server 讀取初始要匯入的 initialState。

```javascript
function getRandomInt(min, max) {
  return Math.floor(Math.random() * (max - min)) + min
}

export function fetchCounter(callback) {
  setTimeout(() => {
    callback(getRandomInt(1, 100))
  }, 500)
}
```

談完 actions 我們來看我們的 reducers，在這個範例中 reducers 也是相對簡單的，主要就是針對新增和減少兩個行為去 set 值，以下是 `src/reducers/counterReducers.js`：

```javascript
import { fromJS } from 'immutable';
import { handleActions } from 'redux-actions';
import { CounterState } from '../constants/models';

import {
  INCREMENT_COUNT,
  DECREMENT_COUNT,
} from '../constants/actionTypes';

const counterReducers = handleActions({
  INCREMENT_COUNT: (state) => (
    state.set(
      'count',
      state.get('count') + 1
    )
  ),
  DECREMENT_COUNT: (state) => (
    state.set(
      'count',
      state.get('count') - 1
    )
  ),
}, CounterState);

export default counterReducers;
```

準備好了 `rootReducer` 就可以使用 `createStore` 來創建我們 store，值得注意的是由於 `configureStore` 需要被 client-side 和 server-side 使用，所以把它輸出成 function 方便傳入 initialState 使用。以下是 `src/store/configureStore.js`：

```javascript
import { createStore, applyMiddleware } from 'redux';
import thunk from 'redux-thunk';
import createLogger from 'redux-logger';
import rootReducer from '../reducers';

export default function configureStore(preloadedState) {
  const store = createStore(
    rootReducer,
    preloadedState,
    applyMiddleware(createLogger({ stateTransformer: state => state.toJS() }), thunk)
  )
  return store
}
```

最後來到了 `components` 和 `containers` 的時間，這次我們的 Component 主要有兩個按鈕讓使用者可以新增和減少數字並顯示目前數字。以下是 `src/components/Counter/Counter.js`：

```javascript
import React, { Component, PropTypes } from 'react'

const Counter = ({
  count,
  onIncrement,
  onDecrement,
}) => (
  <p>
    Clicked: {count} times
    {' '}
    <button onClick={onIncrement}>
      +
    </button>
    {' '}
    <button onClick={onDecrement}>
      -
    </button>
    {' '}
  </p>
);

// 注意要檢查 propTypes 和給定預設值
Counter.propTypes = {
  count: PropTypes.number.isRequired,
  onIncrement: PropTypes.func.isRequired,
  onDecrement: PropTypes.func.isRequired
}

Counter.defaultProps = {
  count: 0,
  onIncrement: () => {},
  onDecrement: () => {}
}

export default Counter;
```

最後把取出的 `count ` 和事件處理方法用 connect 傳到 `Counter` 就大功告成了！以下是 `src/containers/CounterContainer/CounterContainer.js`：

```javascript
import 'babel-polyfill';
import { connect } from 'react-redux';
import Counter from '../../components/Counter';

import {
  incrementCount,
  decrementCount,
} from '../../actions';

export default connect(
  (state) => ({
    count: state.get('counterReducers').get('count'),
  }),
  (dispatch) => ({ 
    onIncrement: () => (
      dispatch(incrementCount())
    ),
    onDecrement: () => (
      dispatch(decrementCount())
    ),
  })
)(Counter);
```

若一切順利，在終端機打上 `$ npm start`，你將可以在瀏覽器的 `http://localhost:3000` 看到自己的成果！

![React Redux Sever Rendering（Isomorphic）入門](/img/kdchang/react-server-rendering-demo.png "React Redux Sever Rendering（Isomorphic）入門")

## 總結
本章闡述了 Web 頁面瀏覽的進程和 Isomorphic JavaScript 的優勢，並介紹了如何使用 React Redux 進行 Server Side Rendering 的應用程式設計。若想參考更進一步資料可以參考筆者撰寫的 React 入門教學書 [《從零開始學 ReactJS》](https://github.com/kdchang/reactjs101)，相關範例程式碼可以[參考這裡](https://github.com/kdchang/reactjs101/tree/master/Ch10/react-redux-server-rendering)，若有任何問題或建議歡迎一起學習討論：）

## 延伸閱讀
1. [DavidWells/isomorphic-react-example](https://github.com/DavidWells/isomorphic-react-example)
2. [RickWong/react-isomorphic-starterkit](https://github.com/RickWong/react-isomorphic-starterkit)
3. [Server-rendered React components in Rails](https://www.bensmithett.com/server-rendered-react-components-in-rails/)
4. [Our First Node.js App: Backbone on the Client and Server](http://nerds.airbnb.com/weve-launched-our-first-nodejs-app-to-product/)
5. [Going Isomorphic with React](https://bensmithett.github.io/going-isomorphic-with-react/#/)
6. [A service for server-side rendering your JavaScript views](https://github.com/airbnb/hypernova)
7. [Isomorphic JavaScript: The Future of Web Apps](http://nerds.airbnb.com/isomorphic-javascript-future-web-apps/)
8. [React Router Server Rendering](https://github.com/reactjs/react-router-tutorial/tree/master/lessons/13-server-rendering)

（image via [airbnb](http://nerds.airbnb.com/wp-content/uploads/2013/11/Screen-Shot-2013-11-06-at-5.21.00-PM.png)）

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校，目前專注在 Mobile 和 IoT 應用開發。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
