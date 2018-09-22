---
title: 透過製作 Babel-plugin 初訪 AST
date: 2018-09-22 16:22:47
tags:
  - es6
  - javascript
  - babel
  - ast
  - pattern
author: arvinh
---

# 前言

最近公司同事組了 Design pattern 的讀書會，剛好這週研讀到 `Visitor Pattern` 時，同事介紹說我們每天在用的工具 Babel 就是採用 `Vistor Pattern`，透過 `visitor` 拜訪 [`AST (Abstract Syntax Tree)`](https://zh.wikipedia.org/wiki/%E6%8A%BD%E8%B1%A1%E8%AA%9E%E6%B3%95%E6%A8%B9)，進而將程式碼進行對應轉換。

這除了勾起我前陣子想要了解 `AST` 的想法外，也再度提醒我對於 Babel Plugin 背後的實作方式不太了解，只知道像是 ESLint, UglifyJS 或是 Webpack，他們的核心都是用到了 `AST` 來實現程式碼的檢查、分析與操作，但並不曉得實際上要如何使用。

此外，我先前其實都沒有想過要自己寫 Babel Plugin，總覺得 Babel 的 Plugin 就是要很一般化，大家都能使用的，但其實不然啊，自己的專案或甚至是公司團隊內的專案，只要能有助於大家的工作效率，就算只給內部使用又何妨？明確定義好使用的情境跟 Style 即可。

因此這次就決定花點時間來學習如何實作 Babel Plugin，並透過實作 Babel Plugin 的過程，一同了解 AST 的概念。

## 一直提到 AST，到底什麼是 AST？

AST 的全文是 Abstract Syntax Tree，中文大多翻作抽象語法樹，主要是將我們 **人類** 所寫的程式語法，轉換成 **程式** 比較容易閱讀的語法結構，並以樹的資料結構來儲存。

直接來個例子，先讓大家看看 **給人類看的 code** 與 **給程式讀的 code** 之間的差異：

![Human code vs Program code(AST)](/img/arvinh/humancode-programcode.png)

左邊的程式經過 Parser 的轉換後，就會產出右圖中的 AST。

在遠古的 Nestcape 時代，對於 Javascript 的 AST 有許多種實作，而後來為了解決一致性的問題，[ESTree](https://github.com/estree) 這個組織定義了現今通用的 [AST specification](https://github.com/estree/estree)。

像是我們現在常用的 Babel 所使用的 AST 就是基於 ESTree 的規範延伸修改的。

### 那 Babel 這些工具到底拿 AST 來做什麼呢？

稍稍回想一下，Babel 或是 ESLint 在我們平日的開發中扮演了什麼角色？

情境大概是：

>你用 ES6 寫了一段 arrow function，而 Babel 會幫忙轉換成 ES5 的匿名函數，同時 ESLint 可能會顯示 Warning 告知你的括號前面必須要留有空格。

這段描述當中，Babel 其實將你的 code 透過三個 stage 來處理：[**parse**, **transform**, **generate**.](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md#stages-of-babel)

而其中，AST 其實就出現了兩次：

<!-- 放置圖片 code to AST and Modefied AST to code -->
![The role AST plays in babel](/img/arvinh/what-ast-play-in-babel.png)

* **Parse stage**

  你寫的 js code 會經由 js parser 轉換為 AST，當然其中會透過 [Lexical Analysis](https://en.wikipedia.org/wiki/Lexical_analysis) 與 [Syntactic Analysis](https://en.wikipedia.org/wiki/Parsing)。

* **Transform stage**

  而這時 Babel plugin 與 ESLint plugin 就可以介入你的程式進行改造，**幫忙轉換成 ES5 的匿名函數**以及**審視你的程式，決定是否要提示 Warning** 的這兩個動作就是在這個階段發生的，我們將 Parser 解析後的 AST 改造成我們想要的樣子。

* **Generate stage**

  最後，改造完成的 AST 會再被 generator 轉換為一般的 code 輸出。

常用的 js parser 有很多種，像是 babylon, acorn, esprima 等等，而所謂的 Babel plugin 或是 ESLint plugin，則可以看作是一種 Transformer。

### 窺探一下 AST 的內容

在實作 Plugin 前，先了解一下 AST 的長相對之後會比較有幫助。

開始之前先介紹兩個常用網站：

* [AST Exporler](http://astexplorer.net/)

  超棒的網站，在上面可以邊看原始程式碼根據不同 Parser 所建構出來的 AST，也能切換不同 Transformer 來實作 Plugin 內容。

* [JAVASCRIPT AST VISUALIZER](http://resources.jointjs.com/demos/javascript-ast)

  主要是將 AST 視覺化，對於理解整個程式碼的 AST 結構能有比較清楚 Overview。

不過我個人是覺得 [AST Exporler](http://astexplorer.net/) 就足夠了， [VISUALIZER](http://resources.jointjs.com/demos/javascript-ast) 雖然能看到圖像化的樹狀結構，但操作起來不是很順暢，也無法直接在上面撰寫 Transformer。所以下面都會以 `AST Exporler` 為使用工具來介紹。

現在我們以一個比最開始稍稍複雜一點的簡單範例來解析，在這邊我們採用與 Babel 相同的 Parser - [Babylon7](https://github.com/babel/babel/tree/master/packages/babel-parser)：

我們的原始程式碼如下，定義 `a` 與 `b` 兩個變數，以及一個 `add` function：

```js
const a = 2
const b = 3
function add(a,b) {
  return a+b
}
```

一步一步來看，`const a = 2` 的 AST 會長這樣：

![AST Explorer example 1](/img/arvinh/ast-explorer-sample-1.png)

你的程式碼在 AST 當中，會被拆解成各種 node 來存放表示，而每一個 node 都有自己的 type，各種 type 有其特定的屬性參數，以上圖為例：

`const` 在 AST 中就是ㄧ個 `type` 為 `VariableDeclaration` 的 node，這個 node 必須包含兩種屬性， `declarations` 與 `kind`：

`kind` 很好理解，意指我們的 `VariableDeclaration` 可以有三種類型（`var`, `let`, `const`）， 而 `declarations` 看起來就是存放你所宣告的變數，像 `a = 2`，但他是 `Array of VariableDeclarator`，我們不是只有宣告一個變數嗎？

其實是要應付這種寫法：`const a = 2, b = 3`，需要能接收多個 `VariableDeclarator`。

而 `VariableDeclarator` 這種 type 的 node 代表的就是 `a=2` 這種宣告式，其中包含兩個屬性 `id` 與 `init`，我們將其展開看看：

![AST Explorer example 2](/img/arvinh/ast-explorer-sample-2.png)

`id` 屬性所接受的是 `identifier` 這個 type 的 node，代表該 `VariableDeclarator` 的 identity。該 node 只需要一個 `name` 屬性，也就是他的名稱，在這邊的例子中當然就是 `a` 囉。而 `init` 則是這個 `VariableDeclarator` 的初始值，因為我們程式中是初始 `a` 為 `2`，所以這個 `init` 屬性就會連到一個 `NumericLiteral` type 的 node，擁有 `value` 為 `2` 的屬性。

此外，眼尖的讀者想必都有發現到，每個 node 都有 `start`, `end` 與 `loc` 這三個屬性，這是用來代表該 node 在程式的第幾行第幾列。

看到這邊應該對於 AST 的結構會有點感覺了，基本上就是 Parser 會先將你的程式切成多種 tokens，接著依照類別來區分是哪種 node，最後從程式結構上一一把 node 串接，形成一顆完整的 Abstract Syntax Tree。如果覺得剛剛用 AST Explorer 轉化的格式不夠清楚，這邊補充一下同樣程式用 VISUALIZER 出來的 AST 長相，應該會對 AST 的結構更有感受：

![ast visualizer](/img/arvinh/ast-visualizer.png)

範例程式中我們還有使用到 `function`，他長出的 AST 與宣告變數在 node type 上有不少差異，但大體結構就如同上面介紹的一般，這邊就不再贅述，相信看到這邊的讀者也有能力自己去 AST Explorer 玩玩看了！寫些簡單的程式，看看他們經由 Parser 轉換後，會產生怎樣的 AST，每種語法所對應的 node type 又是什麼。

## 懂了 AST，來實際應用一下，開始製作 Babel Plugin 吧！

出發，總要有個方向。

要做 Plugin 也得要先決定要做什麼。

剛好在工作上常常會用到 `React-intl` 這個套件來幫忙處理 i18n，他的使用方法還算簡單，當一些基本設定做好以後，你只要用其提供的一個特殊元件 `<FormattedMessage />`，放入對應文字的 id 即可，像是： `<FormattedMessage id="#words-need-i18n" />` 。

雖然實際上已經非常簡單了，但我還想更懶一點，能不能直接輸入 `{'#words-need-i18n'}` 就好呢？

像是能把：

```jsx
<div>
  {'#words-need-i18n'}
</div>
```

轉換成：

```jsx
<div>
  <FormattedMessage id="words-need-i18n" />
</div>
```

可以！自己寫 Plugin 就可以！

### 撰寫 Babel Plugin 的起手式

先到 [AST Exporler](http://astexplorer.net/) 開啟一個新的頁面，接著在上方列表選取好 Parser (Babylon7) 與 Transformer (Babel7)，並在旁邊的程式區塊內寫上一點範例程式，就是你預期能被 Babel 認得並轉換的"新"程式碼，以我的例子就是：

```jsx
<div>
  {'#words-need-i18n'}
</div>
```

![Babel plugin step 1](/img/arvinh/step-1-babel-plugin.gif)

接著看看你的 AST 長什麼樣子，想想你該怎麼修改他：

![Babel plugin step 2](/img/arvinh/step-2-babel-plugin.png)

出現沒看過的 type 了！沒關係，在剛開始撰寫 Plugin 的過程中一定會遇到許多沒看過的 type 或是不清楚他的屬性型別，好在 babel 有一份非常詳細的 [handbook](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md) 與 [docs](https://babeljs.io/docs/en/babel-types#identifier) 可以查閱，只要到上面 `ctrl + f` 一下，應該都能在上頭找到你所需要的知識。

準備好測試程式碼、了解測試程式碼的 AST、也有了工具書可以查，就能毫無懸念的開始撰寫 Babel plugin 了。

### Babel plugin 之 Transform、Visitors、Traversal

還記得我在最一開始說過，Babel 大量使用到了 Visitor pattern 嗎？就是用在這邊！

之所以 Babel 能夠輕易解析你的程式碼，並且進行各種修改操作，依賴的就是各種 visitors 在 AST 上進行 [traverse](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md#babel-traverse)，當遇到對應的 node 時，visitor 就會做出相對的操作，進而將輸入的程式碼 [transform](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md#babel-generator) 成預期的結果，這就是整個 Babel plugin 的實作核心。

### Babel plugin 的基礎結構

若你剛剛有照著我的說明，在 [AST Exporler](http://astexplorer.net/) 上方列表選取好 Parser 與 Transformer，那左下角應該會出現類似下方的程式區塊：

```js
export default function (babel) {
  const { types: t } = babel;
  return {
    name: "ast-transform", // not required
    visitor: {
      Identifier(path, state) {
        path.node.name = path.node.name.split('').reverse().join('');
      }
    }
  };
}
```

這是 [AST Exporler](http://astexplorer.net/) 上的預設 template，基本的 Babel plugin 也就是長這樣。

你會接受一個 `babel` 物件，其中我們會需要的是 `babel.types`，在之後新增修改 node 時會不斷地用到，因此最好存成一個變數，省去 chain lookup。

而基本上你要做的就是回傳一個 `visitor object`，其中定義以 node type 為名的 funciton，接收兩種參數：`path` 與 `state`：

* path
  path 代表的是在 traverse AST 過程中，連接 node 之間的邊。所以你可以用 `path.node` 取得目前的節點，也可以用 `path.parent` 取得父節點。
  此外，path 還能透過 `path.traverse` 來在原有的 visitor 內進行 nested visiting，這對於想要**讓 visitor 在某個特定 visitor 執行後再執行**時很有幫助，可以參考 [handbook 範例](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md#state)。
  
* state
  state 的用法在 handbook 上也沒有說得很明確，我的理解是一個貫串整個 traverse 過程的 global state，你可以在任意階段修改 state。其中也包含你想讓使用 plugin 的使用者傳入的 [options 設定](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md#plugin-options)。

以上述 template 為例，你的 visitor 在遇到 type 為 `Identifier` 的節點時，就會執行 `path.node.name = path.node.name.split('').reverse().join('');`，也就是將該 `Identifier` node 的名稱給顛倒過來。

此外，Visitor 在 traverse AST 的過程中，會在節點上進進出出，所以其實我們是可以定義 `enter` 與 `exit` 的函式來進行操作的：

```js
const MyVisitor = {
  Identifier: {
    enter() {
      console.log("Entered!");
    },
    exit() {
      console.log("Exited!");
    }
  }
};
```

[handbook 中有更多詳細介紹](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md#visitors)

### 動手撰寫自己的 Visitor

知道基礎架構後，就可以開始撰寫 Visitor 了！

但你可能會有點沒頭緒該怎麼開始，因為你不知道要從哪個 node type 的 visitor function 開始寫。這就是 [AST Exporler](http://astexplorer.net/) 的好處了，你只要將游標停放在編譯前的程式碼的任意位置上，右邊的 AST 樹就會自動 Focus 到對應的節點上頭（前提是要記得選取上方的 `AutoFocus`，預設會是啟用的）：

![Find the node we need](/img/arvinh/ast-explorer-find-node.gif)

以範例來說，我們要轉換的是 `{'#words-need-i18n'}`，將游標指上去後發現他是一個 type 為 `StringLiteral` 的 node，這就是我們要撰寫的 visitor function！

```js
export default function (babel) {
  const { types: t } = babel;
  
  return {
    name: "i18n-transform", // not required
    visitor: {
      StringLiteral(path) {
        // some logic in here
      }
    }
  };
}
```

接下來把需要的邏輯填寫上去：

```js
visitor: {
  StringLiteral(path) {
    if (path.parent.type !== 'JSXExpressionContainer') return
    if (path.node.value.startsWith('#')) {
      path.parentPath.replaceWith(
          t.JSXElement(
              t.JSXOpeningElement(
                  t.JSXIdentifier('FormattedMessage'),
                    [t.JSXAttribute(
                      t.JSXIdentifier('id'),
                        t.StringLiteral(path.node.value.replace('#',''))
                    )],
                    true
                ),
                null,
                [],
                true
            )
        )
    }
  }
}
```

<!-- 解釋前四行  然後畫個 path 的圖 -->
在前面有介紹到，`path` 代表連接著目前被拜訪到的節點，所以我們能用 `path.node.value.startsWith('#')` 來檢查目前節點的值是否為我們想要的（開頭為 hashtag）。但由於 `StringLiteral` 感覺得出來是到處都會出現的 node type，所以我們需要設立一些條件：當 `path.parent.type !== 'JSXExpressionContainer'` 時，我們就 bypass 這次的 visit 操作。所謂的 `JSXExpressionContainer` 就是在 jsx 中的 `{ }`。

當條件都成立時，也就代表我們的 visitor 成功找到我們想轉換的程式碼 `{'#words-need-i18n'}`。這時聰明如你，一定會想說那就把 `path.node` 替換掉就好了吧！

但是，`path.node` 目前指到的是 `StringLiteral`，也就是 `#words-need-i18n`，外面還有一層 `JSXExpressionContainer`，我們不能直接使用 `path.node.replaceWith` 來替換程式碼，我們要連同上一層都一起換掉，因此這邊需要使用 `path.parentPath.replaceWith`。以圖像表示的話大概像這樣：

![需要找到 parent path 來直接從上層替換程式碼節點](/img/arvinh/ast-explorer-path-replace.png)

<!-- 放個 gif 說明如何找出要 create 什麼 node, 以及去哪裡找合法參數 （提及 log 訊息不一定看得出來）-->
接著，`replaceWith` 接收你要替換的節點當作參數，而這時我們最一開始宣告的 `const { types: t } = babel;` 就派上用場啦！

`babel.types` 可以幫我們創建出各種 type 的 node，也提供許多 type checking 的 function，像是剛剛我們檢查 `path.parent.type` 的地方其實應該更改為 `!t.isJSXExpressionContainer(path.parent)`。更多關於 `babel.types` 的操作可以參考 [handbook 上的說明](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md#toc-transformation-operations)

但到這邊問題又來了，我們怎麼知道要創建什麼 node 呢？

我一開始也不知道，但抬頭想一想，AST Explorer 不是就會幫我們轉換 AST 嗎？那就把我們預期的結果程式碼也貼上去，不就知道他的 AST 長相了嗎？

大膽將 `<FormattedMessage id={'#words-need-i18n'}>` 貼上去後，就能得到其 AST 結構：

![預期結果的 AST 結構](/img/arvinh/ast-explorer-final-ast-format.png)

接著呢，就到 **[babel 官網的 docs](https://babeljs.io/docs/en/babel-types#identifier)  裡面去查詢每一種需要創建的 type，各自要填入哪些參數**，舉例來說，`<FormattedMessage>` 會是一個被 `JSXOpeningElement` 包起來，從 [docs](https://babeljs.io/docs/en/babel-types#jsxopeningelement) 能看到非常詳細的說明：

![JSXOpeningElement](/img/arvinh/ast-explorer-docs.png)

我們要填入三個參數: `name`, `attributes` 與 `selfClosing`，第一與第三個很好理解，中間的 `attributes` 裡面包含了其他的 type，這時你就需要再繼續往下查，直到完成所有需求為止。

照著這樣的思路，一一將 AST 內的結構轉換到你的 plugin 程式碼內，就大功告成啦！

![最終結果](/img/arvinh/ast-explorer-final-result.png)

## 結論

當然這只是個很簡陋的實作，不過也算是把 plugin 的製作概念呈現了一遍，並稍加了解了 AST 的重要性與實用性。

在實作你所想要的 plugin 功能時，勢必會遇到複雜得多的狀況，建議大家還是直接在 handbook 上查看最直接，上面有更詳細的 API 介紹與教學，包含 [babel-template](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md#babel-template) 、 babel-types 的 [builder](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md#builders) 與 [Validators](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md#validators)，更有 [Best Practices](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md#best-practices) 可供參考。也可以看看高手們的[作品](https://github.com/kentcdodds/babel-plugin-preval)

最後，不知道有沒有讀者會覺得整個 Plugin 的邏輯操作與大家熟悉的 jQuery 有點類似？都是選取到某個元件後，就針對該元件進行更動：`$('StringLiteral').textContent = '<FormattedMessage id={'xxx'}>'`

我在查資料的過程中發現一篇[非常有趣又很清楚的介紹](https://www.henryzoo.com/babel-plugin-slides/assets/player/KeynoteDHTMLPlayer.html#48)，其中就是以 jQuery 來做類比（節錄其 slides 中的一段）：

`Babel:Javascript :: jQuery:DOM`

**jQuery 幫助你改變 DOM，而 Babel 幫助你轉化 Javascirpt**

實在是很貼切的比喻不是嗎？

## 資料來源

1. [Writing custom Babel and ESLint plugins](http://slides.com/kentcdodds/a-beginners-guide-to-asts)
2. [How writing babel-plugin is like wrigint jQuery](https://www.henryzoo.com/babel-plugin-slides/assets/player/KeynoteDHTMLPlayer.html#2)
3. [Babel Plugin Handbook](https://github.com/jamiebuilds/babel-handbook/blob/master/translations/en/plugin-handbook.md)
4. [AST Exporler](http://astexplorer.net/)
5. [JAVASCRIPT AST VISUALIZER](http://resources.jointjs.com/demos/javascript-ast)
6. [AST specification](https://github.com/estree/estree)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化