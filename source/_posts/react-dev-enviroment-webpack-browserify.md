---
title: 一看就懂的 React 開發環境建置與 Webpack 入門教學
date: 2016-07-30 23:26:00
tags: React, React Native, ES5, ES6, ES7, JavaScript, ECMAScript2015, Webpack, Browserify, CommonJS, UMD, ES6 Module, CMD
author: kdchang
---

![一看就懂的 React 開發環境建置與 Webpack 入門教學](/img/kdchang/react-webpack-browserify.png "一看就懂的 React 開發環境建置與 Webpack 入門教學")

# 前言
俗話說工欲善其事，必先利其器。寫程式也是一樣，搭建好開發環境後可以讓自己在後續開發上更加順利。因此本篇接下來將討論 React 開發環境的三種主要方式：CDN-based、[browserify](https://webpack.github.io/) 和 [webpack](https://webpack.github.io/)（這邊我們就先不討論 [TypeScript](https://www.typescriptlang.org/) 的開發方式），讓讀者閱讀完本章後可以開始 React 開發之旅！

# JavaScript 模組化
隨著網站開發的複雜度提昇，許多現代化的網站已不是單純的網站而已，更像是個富有互動性的網頁應用程式（Web App）。為了應付現代化網頁應用程式開發的需求，解決一些像是全域變數污染、低維護性等問題，JavaScript 在模組化上也有長足的發展。過去一段時間讀者們或許聽過像是 `Webpack`、`Browserify`、`module bundlers`、`AMD`、`CommonJS`、`UMD`、`ES6 Module` 等有關 JavaScript 模組化開發的專有名詞或工具。若是讀者對於 JavaScript 模組化開發尚不熟悉的話推薦可以參考[這篇文章](http://huangxuan.me/2015/07/09/js-module-7day/) 和 [這篇文章](https://medium.freecodecamp.com/javascript-modules-a-beginner-s-guide-783f7d7a5fcc#.oa2n5s5zt)，當作入門。筆者之後也會有相關文章針對 JavaScript 模組化議題做討論。因為限於篇幅，這邊我們會專注在 `React` 開發環境的三種主要方式介紹。

總的來說，使用模組化開發 JavaScript 應用程式主要有以下三種好處：

1. 提昇維護性（Maintainability）
2. 命名空間（Namespacing）
3. 提供可重用性（Reusability）

而在 React 應用程式開發上更推薦使用像是 `Webpack` 這樣的 `module bundlers` 來組織我們的應用程式，但對於一般讀者來說 `Webpack` 強大而完整的功能相對複雜。為了讓讀者先熟悉 `React` 核心觀念（我們假設讀者已經有使用 `JavaScript` 或 `jQuery` 的基本經驗），我們將從使用 `CDN` 引入 `<script>` 的方式開始介紹：

# CDN-based
![一看就懂的 React 開發環境建置與 Webpack 入門教學](/img/kdchang/react.png "一看就懂的 React 開發環境建置與 Webpack 入門教學")
使用 CDN-based 的開發方式缺點是較難維護我們的程式碼（當引入函式庫一多就會有很多 `<script/>`）且會容易遇到版本相容性問題，不太適合開發大型應用程式，但因為簡單易懂，適合教學上使用。

以下是 React [官方首頁的範例](https://facebook.github.io/react/index.html)，以下使用 `React v15.2.1`：
0. 理解 `React` 是 `Component` 導向的應用程式設計
1. 引入 `react.js`、`react-dom.js`（react 0.14 後將 react-dom 從 react 核心分離，更符合 react 跨平台抽象化的定位）以及 `babel-core-browser` 版 script（可以想成 `babel` 是翻譯機，翻譯瀏覽器看不懂的 `JSX` 或 `ES6+` 語法成為瀏覽器看的懂得的 `JavaScript`）。事實上，JSX 並非一種全新的語言，而是一種語法糖（Syntatic Sugar），一種語法類似 XML 的 ECMAScript 語法擴充。若讀者不熟悉的話可以參考筆者另一篇文章：[一看就懂的 JSX 簡明入門教學指南](http://blog.techbridge.cc/2016/04/21/react-jsx-introduction/)
2. 在 `<body>` 撰寫 React Component 要插入（mount）的地方：`<div id="example"></div>`
3. 透過 `babel` 進行語言翻譯 `React JSX` 語法，`babel` 會將其轉為瀏覽器看的懂得 `JavaScript`。其代表意義是：`ReactDOM.render(欲 render 的 Component 或 HTML 元素, 欲插入的位置)`。所以我們可以在瀏覽器上打開我們的 `hello.html`，就可以看到 `Hello, world!` 。That's it，我們第一個 `React` 應用程式就算完成了！

```html hello.html
<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8" />
    <title>Hello React!</title>
    <!-- 以下引入 react.js, react-dom.js（react 0.14 後將 react-dom 從 react 核心分離，更符合 react 跨平台抽象化的定位）以及 babel-core browser 版 -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/react/15.2.1/react.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/react/15.2.1/react-dom.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/babel-core/5.8.34/browser.min.js"></script>
  </head>
  <body>
    <!-- 這邊的 id="example" 的 <div> 為 React Component 要插入的地方 -->
    <div id="example"></div>
    <!-- 以下就是包在 babel（透過進行語言翻譯）中的 React JSX 語法，babel 會將其轉為瀏覽器看的懂得 JavaScript -->
    <script type="text/babel">
      ReactDOM.render(
        <h1>Hello, world!</h1>,
        document.getElementById('example')
      );
    </script>
  </body>
</html>
```

在瀏覽器瀏覽最後成果：

![一看就懂的 React 開發環境建置與 Webpack 入門教學](/img/kdchang/hello-world.png "一看就懂的 React 開發環境建置與 Webpack 入門教學")

# Browserify + Gulp + Babelify
![一看就懂的 React 開發環境建置與 Webpack 入門教學](/img/kdchang/react-browserify-gulp.png "一看就懂的 React 開發環境建置與 Webpack 入門教學")
在進入第二種方法前，首先先介紹一下會用到 [Browserify](http://browserify.org/)、[Gulp](http://gulpjs.com/)、[Babelify](https://github.com/babel/babelify) 三種前端開發常會用到的工具：

[Browserify](http://browserify.org/)
- 如同官網上說明的：`Browserify lets you require('modules') in the browser by bundling up all of your dependencies.
`，Browserify 是一個可以讓你在瀏覽器端也能使用像 Node 用的 [CommonJS](https://en.wikipedia.org/wiki/CommonJS) 規範一樣，用輸出（export）和引用（require）來管理模組。此外，也能使用許多在 NPM 中的模組

[Gulp](http://gulpjs.com/)
- `Gulp` 是一個前端任務工具自動化管理工具。隨著前端工程的發展（Task Runner），我們在開發前端應用程式時有許多工作是必須重複進行，例如：打包文件、uglify、將 LESS 轉譯成一般的 CSS 的檔案，轉譯 ES6 語法等工作。若是使用一般手動的方式，往往會造成效率的低下，所以透過像是 [Grunt](http://gruntjs.com/)、Gulp 這類的 Task Runner 不但可以提昇效率，也可以更方便管理這些任務。由於 Gulp 是透過 pipeline 方式來處理檔案，在使用上比起 Grunt 的方式直觀許多，所以這邊我們主要討論的是 Gulp

[Babelify](https://github.com/babel/babelify)
- `Babelify` 是一個使用 Browserify 進行 Babel 轉換的外掛，你可以想成是一個翻譯機，可以將 React 中的 `JSX` 或 `ES6` 語法轉成瀏覽器相容的 `ES5` 語法

初步了解了三種工具的概念後，接下來我們就開始我們的環境設置：
1. 若是電腦中尚未安裝 [Node](https://zh.wikipedia.org/zh-tw/Node.js)（Node.js 是一個開放原始碼、跨平台的、可用於伺服器端和網路應用的 Google V8 引擎執行執行環境）和 NPM（Node 套件管理器 Node Package Manager。是一個以 JavaScript 編寫的軟體套件管理系統，預設環境為 Node.js，從 Node.js 0.6.3 版本開始，npm 被自動附帶在安裝包中）的話，請先 [上官網安裝](https://nodejs.org/en/)

2. 用 `npm` 安裝 `browserify`

3. 用 `npm` 安裝 `gulp`、`gulp-concat`、`gulp-html-replace`、`gulp-streamify`、`gulp-uglify`、`watchify`、`vinyl-source-stream` 開發環境用的套件（development dependencies）

	```
	// 使用 npm install --save-dev 會將安裝的套件名稱和版本存放到 package.json 的 devDependencies 欄位中
	$ npm install --save-dev gulp gulp-concat gulp-html-replace gulp-streamify gulp-uglify watchify vinyl-source-stream  
	```

3. 安裝 `babelify`、`babel-preset-es2015`、`babel-preset-react`，轉譯 `ES6` 和 `JSX` 開發環境用的套件，並於根目錄底下設定 `.babelrc`，設定轉譯規則（presets：es2015、react）和使用的外掛

	```
	// 使用 npm install --save-dev 會將安裝的套件名稱和版本存放到 package.json 的 devDependencies 欄位中
	$ npm install --save-dev babelify babel-preset-es2015 babel-preset-react
	```

	```js .babelrc
	{
		"presets": [
		  "es2015",
		  "react",
		],
		"plugins": []
	}
	```

4. 安裝 react 和 react-dom

	```
	$ npm install --save react react-dom
	```

6. 撰寫 Component

	```js ./app/index.js
	import React from 'react';
	import ReactDOM from 'react-dom';

	class App extends React.Component {
	  constructor(props) {
	    super(props);
	    this.state = {
	    };
	  }
	  render() {
	    return (
	      <div>
	        <h1>Hello, World!</h1>
	      </div>
	    );
	  }
	}

	ReactDOM.render(<App />, document.getElementById('app'));
	```

	```html ./index.html
	<!DOCTYPE html>
	<html lang="en">
	<head>
		<meta charset="UTF-8">
		<title>Hello React!</title>
	</head>
	<body>
		<div id="app"></div>
		<!-- build:js -->
		<script src="./dist/src/bundle.js"></script>
		<!-- endbuild -->
	</body>
	</html>
	```

7. 設定 `gulpfile.js`

	```js gulpfile.js
	// 引入所有需要的檔案
	const gulp = require('gulp');
	const uglify = require('gulp-uglify');
	const htmlreplace = require('gulp-html-replace');
	const source = require('vinyl-source-stream');
	const browserify = require('browserify');
	const watchify = require('watchify');
	const babel = require('babelify');
	const streamify = require('gulp-streamify');
	// 檔案位置參數
	const path = {
	  HTML: 'index.html',
	  MINIFIED_OUT: 'bundle.min.js',
	  OUT: 'bundle.js',
	  DEST: 'dist',
	  DEST_BUILD: 'dist/build',
	  DEST_SRC: 'dist/src',
	  ENTRY_POINT: './app/index.js'
	};
	// 複製 html 到 dist 資料夾中
	gulp.task('copy', function(){
	  gulp.src(path.HTML)
	    .pipe(gulp.dest(path.DEST));
	});
	// 監聽檔案是否有變化，若有變化則重新編譯一次
	gulp.task('watch', function() {
	  gulp.watch(path.HTML, ['copy']);
	var watcher  = watchify(browserify({
	    entries: [path.ENTRY_POINT],
	    transform: [babel],
	    debug: true,
	  }));
	return watcher.on('update', function () {
	    watcher.bundle()
	      .pipe(source(path.OUT))
	      .pipe(gulp.dest(path.DEST_SRC))
	      console.log('Updated');
	  })
	    .bundle()
	    .pipe(source(path.OUT))
	    .pipe(gulp.dest(path.DEST_SRC));
	});
	// 執行 build production 的流程（包括 uglify、轉譯等）
	gulp.task('copy', function(){
	  browserify({
	    entries: [path.ENTRY_POINT],
	    transform: [babel],
	  })
	    .bundle()
	    .pipe(source(path.MINIFIED_OUT))
	    .pipe(streamify(uglify(path.MINIFIED_OUT)))
	    .pipe(gulp.dest(path.DEST_BUILD));
	});
	// 將 script 引用換成 production 的檔案
	gulp.task('replaceHTML', function(){
	  gulp.src(path.HTML)
	    .pipe(htmlreplace({
	      'js': 'build/' + path.MINIFIED_OUT
	    }))
	    .pipe(gulp.dest(path.DEST));
	});
	// 設定 NODE_ENV 為 production
	gulp.task('apply-prod-environment', function() {
	    process.env.NODE_ENV = 'production';
	});

	// 若直接執行 gulp 會執行 gulp default 的任務：watch、copy。若跑 gulp production，則會執行 build、replaceHTML、apply-prod-environment
	gulp.task('production', ['build', 'replaceHTML', 'apply-prod-environment']);
	gulp.task('default', ['watch', 'copy']);
	```

8. 成果展示
	到目前為止我們的資料夾的結構應該會是這樣：

	![一看就懂的 React 開發環境建置與 Webpack 入門教學](/img/kdchang/browserify-folder-pregulp.png "一看就懂的 React 開發環境建置與 Webpack 入門教學")

	接下來我們透過在終端機（terminal）下 `gulp` 指令來處理我們設定好的任務：

	```
	// 當只有輸入 gulp 沒有輸入任務名稱時，gulp 會自動執行 default 的任務，我們這邊會執行 `watch` 和 `copy` 的任務，前者會監聽 `./app/index.js` 是否有改變，有的話則更新。後者則是會把 `index.html` 複製到 `./dist/index.html`
	$ gulp
	```

	當執行完 `gulp` 後，我們可以發現多了一個 `dist` 資料夾

	![一看就懂的 React 開發環境建置與 Webpack 入門教學](/img/kdchang/browserify-folder-possgulp.png "一看就懂的 React 開發環境建置與 Webpack 入門教學")

	如果我們是要進行 `production` 的應用程式開發的話，我們可以執行： 

	```
	// 當輸入 gulp production 時，gulp 會執行 production 的任務，我們這邊會執行 `replaceHTML`、`build` 和 `apply-prod-environment` 的任務，`build` 任務會進行轉譯和 `uglify`。`replaceHTML` 會取代 `index.html` 註解中的 `<script>` 引入檔案，變成引入壓縮和 `uglify` 後的 `./dist/build/bundle.min.js`。`apply-prod-environment` 則是會更改 `NODE_ENV` 變數，讓環境設定改為 `production`，有興趣的讀者可以參考[React 官網說明](https://facebook.github.io/react/downloads.html)
	$ gulp production
	```

	此時我們可以在瀏覽器上打開我們的 `./dist/hello.html`，就可以看到 `Hello, world!` 了！


# Webpack
![一看就懂的 React 開發環境建置與 Webpack 入門教學](/img/kdchang/webpack-module-bundler.png "一看就懂的 React 開發環境建置與 Webpack 入門教學")

[Webpack](https://webpack.github.io/) 是一個模組打包工具（module bundler），以下列出 Webpack 的幾項主要功能：

- 將 CSS、圖片與其他資源打包
- 打包之前預處理（Less、CoffeeScript、JSX、ES6 等）的檔案
- 依 entry 文件不同，把 .js 分拆為多個 .js 檔案
- 整合豐富的 loader 可以使用

接下來我們一樣透過 Hello World 實例來介紹如何用 Webpack 設置 React 開發環境：

1. 安裝 Node 和 NPM

2. 安裝 Webpack（可以 global 或 local project 安裝）、webpack loader、webpack-dev-server

	Webpack 中的 loader 類似於 browserify 內的 transforms，但 Webpack 在使用上比較多元，除了 JavaScript loader 外也有 CSS Style 和圖片的 loader。此外，`webpack-dev-server` 則可以啟動開發用 server 方便使用

	```
	$ npm install --save-dev babel-core babel-eslint babel-loader babel-preset-es2015 babel-preset-react html-webpack-plugin webpack webpack-dev-server
	```

3. 設定 `webpack.config.js`
	事實上，`webpack.config.js` 有點類似於前述的 `gulpfile.js` 功用，主要是設定 `webpack` 的相關設定

	```js ./webpack.config.js
	// 這邊使用 HtmlWebpackPlugin，將 bundle 好得 <script> 插入到 body  
	const HtmlWebpackPlugin = require('html-webpack-plugin');

	const HTMLWebpackPluginConfig = new HtmlWebpackPlugin({
	  template: `${__dirname}/app/index.html`,
	  filename: 'index.html',
	  inject: 'body',
	});
	
	// 檔案起始點從 entry 進入，也可以是多個檔案。output 是放入產生出來的結果的相關參數。loaders 則是放欲使用的 loaders，在這邊是使用 babel-loader 將所有 .js（這邊用到正則式）相關檔案轉譯成瀏覽器可以閱讀的 JavaScript。devServer 則是 webpack-dev-server 設定。plugins 放置所使用的外掛
	module.exports = {
	  entry: [
	    './app/index.js',
	  ],
	  output: {
	    path: `${__dirname}/dist`,
	    filename: 'index_bundle.js',
	  },
	  module: {
	    loaders: [
	      {
	        test: /\.js$/,
	        exclude: /node_modules/,
	        loader: 'babel-loader',
	        query: {
	          presets: ['es2015', 'react', 'stage-0'],
	        },
	      },
	    ],
	  },
	  devServer: {
	    inline: true,
	    port: 8008,
	  },
	  plugins: [HTMLWebpackPluginConfig],
	};
	```

4. 設定 `.babelrc`

	```js .babelrc
	{
	  "presets": [
	    "es2015",
	    "react",
	  ],
	  "plugins": []
	}
	```

5. 安裝 react 和 react-dom

	```
	$ npm install --save react react-dom
	```

6. 撰寫 Component

	```html ./app/index.html
	<!DOCTYPE html>
	<html lang="en">
	<head>
		<meta charset="UTF-8">
		<title>React Setup</title>
		<link rel="stylesheet" type="text/css" href="//maxcdn.bootstrapcdn.com/bootstrap/3.3.6/css/bootstrap.min.css">
	</head>
	<body>
		<div id="app"></div>
	</body>
	</html>
	```

	```js ./app/index.js
	import React from 'react';
	import ReactDOM from 'react-dom';

	class App extends React.Component {
	  constructor(props) {
	    super(props);
	    this.state = {
	    };
	  }
	  render() {
	    return (
	      <div>
	        <h1>Hello, World!</h1>
	      </div>
	    );
	  }
	}

	ReactDOM.render(<App />, document.getElementById('app'));
	```

7. 在終端機使用 `webpack`，成果展示

- webpack：會在開發模式下開始一次性的建置
- webpack -p：會建置 production 的程式碼 
- webpack --watch：會監聽程式碼的修改，當儲存時有異動時會更新檔案
- webpack -d：加入 source maps 檔案
- webpack --progress --colors：加上處理進度與顏色

如果不想每次都打一長串的指令碼的話可以使用 `package.json` 中的 `scripts` 設定

```js .package.json
"scripts": {
  "dev": "webpack-dev-server --devtool eval --progress --colors --content-base build"
}
```

然後在終端機執行：

```
$ npm run dev
```

當我們此時我們可以打開瀏覽器輸入 `http://localhost:8008` ，就可以看到 `Hello, world!` 了！

# 總結
以上就是 React 開發環境的三種主要方式：CDN-based、[browserify](https://webpack.github.io/) 和 [webpack](https://webpack.github.io/)。一般來說在開發大型應用程式，使用 React + Gulp + Browserify 或用 React 搭配 Webpack 都是合適的作法，主要是依據團隊開發的習慣和約定。不過，若你不想在環境設定上花太多時間的話，不妨參考 Facebook 開發社群推出的 [create-react-app](https://github.com/facebookincubator/create-react-app)，可以快速上手，使用 Webpack、[Babel](https://babeljs.io/)、[ESLint](http://eslint.org/) 開發 React 應用程式！

# 延伸閱讀
1. [JavaScript 模块化七日谈](http://huangxuan.me/2015/07/09/js-module-7day/)
2. [前端模块化开发那点历史](https://github.com/seajs/seajs/issues/588)
3. [JavaScript Modules: A Beginner’s Guide](https://medium.freecodecamp.com/javascript-modules-a-beginner-s-guide-783f7d7a5fcc#.oa2n5s5zt)
4. [深入了解 Webpack Plugins](https://rhadow.github.io/2015/05/30/webpack-loaders-and-plugins/)
5. [Babel 入门教程](http://www.ruanyifeng.com/blog/2016/01/babel.html)
6. [WEBPACK入門教學筆記](http://blog.kkbruce.net/2015/10/webpack.html#.V41hfpN96Ho)
7. [Setting up React for ES6 with Webpack and Babel](https://twilioinc.wpengine.com/2015/08/setting-up-react-for-es6-with-webpack-and-babel-2.html)
8. [【webpack】的基本工作流程](https://medium.com/html-test/webpack-%E7%9A%84%E5%9F%BA%E6%9C%AC%E5%B7%A5%E4%BD%9C%E6%B5%81%E7%A8%8B-585f2bc952b9#.auq2o72rr)
9. [我的REACT开发之路1:REACT的环境搭建](http://hao.jser.com/archive/10507/)
10. [利用browserify和gulp来构建react应用](http://www.ifeenan.com/javascript/2014-08-20-%E5%88%A9%E7%94%A8Browserify%E5%92%8CGulp%E6%9D%A5%E6%9E%84%E5%BB%BAReact%E5%BA%94%E7%94%A8/)
11. [gulp 學習筆記](https://www.gitbook.com/book/kejyuntw/gulp-learning-notes/details)
12. [Webpack vs Browserify - what's best for React?](https://www.reddit.com/r/reactjs/comments/30at04/webpack_vs_browserify_whats_best_for_react/)
13. [Browserify vs Webpack](https://medium.com/@housecor/browserify-vs-webpack-b3d7ca08a0a9#.jma7qyb2e)
14. [Setting up environment for React, SASS, ES2015, Babel with Webpack](https://medium.com/@srinisoundar/setting-up-environment-for-react-sass-es2015-babel-with-webpack-2f77445129#.7u0gtngil)
15. [Javascript Tutorial: How to set up Gulp for Developing ES2015 React v0.14+ Applications with Babelify & Browserify](http://jpsierens.com/tutorial-gulp-javascript-2015-react/)
16. [React.js Tutorial Pt 2: Building React Applications with Gulp and Browserify.](https://tylermcginnis.com/react-js-tutorial-pt-2-building-react-applications-with-gulp-and-browserify-5489228dde99#.1xb4ilepl)
17. [Building modular javascript applications in ES6 with React, Webpack and Babel](https://medium.com/@yamalight/building-modular-javascript-applications-in-es6-with-react-webpack-and-babel-538189cd485f#.bqq4rkjq9)
18. [Make your own React production version with webpack](http://dev.topheman.com/make-your-react-production-minified-version-with-webpack/)
19. [How to set React to production mode when using Gulp](http://stackoverflow.com/questions/32526281/how-to-set-react-to-production-mode-when-using-gulp)
20. [Browserify 使用指南](http://zhaoda.net/2015/10/16/browserify-guide/)
21. [使用 Webpack 建立 React 專案開發環境](https://rhadow.github.io/2015/04/02/webpack-workflow/)
22. [如何使用 Webpack 模組整合工具](https://rhadow.github.io/2015/03/23/webpackIntro/)
23. [參透 webpack](http://andyyou.logdown.com/posts/370326)
24. [WEBPACK DEV SERVER](http://www.jianshu.com/p/941bfaf13be1)
25. [Understanding ES6 Modules](https://www.sitepoint.com/understanding-es6-modules/)

(image via [srinisoundar](https://cdn-images-1.medium.com/max/477/1*qhI4E_g3TDOK0uu1VAJlCQ.png)、[sitepoint](https://d2sis3lil8ndrq.cloudfront.net/screencasts/46e215cd-2eb3-4cf0-b699-713977a2b644.png)、[keyholesoftware](https://keyholesoftware.com/wp-content/uploads/Browserify-5.png)、[survivejs](http://survivejs.com/webpack/images/webpack.png))

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校，目前專注在 Mobile 和 IoT 應用開發。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
