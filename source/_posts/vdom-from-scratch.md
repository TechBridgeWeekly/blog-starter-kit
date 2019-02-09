---
title: 從頭打造一個簡單的 Virtual DOM
date: 2019-02-04 11:19:30
tags:
  - virtual dom
  - react
  - javascript
  - web
---

# 前言

過年除舊佈新，剛好趁這個機會來複習一下已經是老觀念的 Virtual DOM。很多人在講到 React 的時候都一定會提到 Virtual DOM，而問到 Virtual DOM 的好處時，就會說到實際 DOM 的操作成本很貴，所以透過 Virtual DOM 可以降低成本。

你在除夕餐桌上這樣講可能沒問題，面試只講這樣應該不太好。

畢竟你最後還是會操作實體 DOM 啊，這樣說明太簡化了。

Virtual DOM 的由來可以從 MVC 和 MVVM 的架構追溯起，主要都是為了解決前端頁面呈現、資料更動、使用者操作這三種狀態交互作用產生的複雜性，MVC 提供了一個解法，MVVM 提出的 View Model 有了優化的方案，還有 data 與 view 雙向綁定的方式等等，而 React 提出了另一種思路，但那不是我今天的重點，有興趣且還不知道這些名詞是什麼的讀者可以去搜尋看看，有很多文章在說明這些資訊與歷史。

React 的 Virtual DOM 是因應其數據與 UI 更新繪製的特殊思路而提出的效能解決方案。

React 希望在資料更新時，能夠直接重新渲染頁面，不用主動去探究是數據的哪部份發生變化，要對應去更新頁面哪一部分的 DOM。但頁面重新渲染的成本可是更高，所以才需要 Virtual DOM 作為緩衝，透過資料更新後，重新繪製 Virtual DOM，與實體 DOM 進行 Diff，最後再把差異部分 Patch 上去，這不僅修正了重新渲染的成本問題，也降低了 data 與 view 交互更新的複雜度，提高了 developer 的開發體驗。

說了這麼多，其實今天就只是單純想自己手刻一個 Virtual DOM 來理解一下該怎麼實現這樣的功能，畢竟知道了概念，總覺得手刻應該不難。
手刻 Virtual DOM 其實也沒什麼太大意義，但很多時候就是 for fun，然後做個記錄。

主要參考至 [@ycmjason](https://twitter.com/ycmjason) 的 [talk](https://youtu.be/85gJMUEcnkc) 與 [blog](https://dev.to/ycmjason/building-a-simple-virtual-dom-from-scratch-3d05#mount-node-target)，非常推薦欣賞，講者的熱情完全掩蓋掉音訊不佳的缺點，又很清楚地介紹了 VDOM 實作。

## 所以，Virtual DOM 到底長什麼樣子？

Virtual DOM 就只是個 javascript plain object，並且模仿 Actual DOM 的結構（但當然簡化很多）：

```js
const vElement = {
  tagName: 'div',
  attrs: {
    id: 'v-element',
  },
  children: []
};
```

一個基本的 VDOM，我們只需要元素名稱（tagName）、元素屬性（attrs）與其 Children list（既然是虛擬 DOM，這個 plain object 裡面的屬性其實隨便你取名，只要對應得到實際 DOM 即可）。

根據這個想法，我們可以模仿現存的 VDOM lib，提供一個 `createElement` 的 function：

```js createElement.js
export default (tagName, { attrs = {}, children = [] }) => {
  const vElement = Object.create(null);

  Object.assign(vElement, {
    tagName,
    attrs,
    children,
  });

  return vElement;
};
```

> Note: 利用 `Object.create(null)` 與 `Object.assign` 的方式產生物件，可以避免直接採用 Object literals 的方式會繼承到 object prototype 的屬性。

使用方式如下：

```js main.js
import createElement from './createElement';

const vRootApp = createElement('div', {
  attrs: {
    id: 'root',
  },
  children: [
    createElement('img', {
      attrs: {
        src: 'http://placekitten.com/200/300',
      },
    }),
  ],
});
console.log(vRootApp);
```

結果：

![vdom-createElement](/img/arvinh/vdom-createElement.png)

## 從 Virtual DOM 到 Real DOM

有了 Virtual DOM，我們還需要一個 `render` 函數來將其繪製到頁面上。方法很簡單，我們只需要 `document.createElement`、`setAttribute` 與 `appendChild` 三個 web api 即可完成：

```js render.js
const renderElem = ({ tagName, attrs, children }) => {
  const elem = document.createElement(tagName);
  for (const [k, v] of Object.entries(attrs)) {
    elem.setAttribute(k, v);
  }
  for (const child of children) {
    elem.appendChild(renderElem(child));
  }
  return elem;
};
export default renderElem;
```

根據 `tagName` 使用 `document.createElement` 來建立實際的 DOM 物件，並且將 `attrs` 一個一個 `setAttribute` 到實際的 DOM 元素上；最後再將 `children` 遞迴丟入 `renderElem` 函數中，將所有小孩的實際 DOM object 都建立好並 `appendChild` 到上層的實際 DOM 物件上，最後將完整的 real DOM object 回傳出去。

以概念來說基本上這樣就完成了，但可以讓他在完整一點，提供 `textNode` 的支援，利用 `document.createTextNode` 來產生純 string 的元素，稍微修改 `render.js` 如下：

```js render.js
const renderElem = ({ tagName, attrs, children }) => {
  const elem = document.createElement(tagName);
  for (const [k, v] of Object.entries(attrs)) {
    elem.setAttribute(k, v);
  }
  for (const child of children) {
    elem.appendChild(render(child));
  }
  return elem;
};
const render = (vNode) => {
  if (typeof vNode === 'string') {
    return document.createTextNode(vNode);
  }
  return renderElem(vNode);
};
export default render;
```

從 `render` 函數回傳的基本上就會是一顆完整的 Virtual DOM Tree 了，舉個例子來看：

```js
import createElement from './createElement';
import render from './render';

const vRootApp = createElement('div', {
  attrs: {
    id: 'root',
  },
  children: [
    'Hello VDOM',
    createElement('img', {
      attrs: {
        src: 'http://placekitten.com/200/300',
      },
    }),
  ],
});

const rootApp = render(vRootApp);
```

結果如下，Virtual DOM 就是個 Javascript plain object，而經由 `render` 函數回傳的即是包含實際 DOM 屬性的 Real DOM：

![Render to real dom](/img/arvinh/vDom-render-actualDOM.png)

### 掛到頁面上吧！

透過 `render` 我們有了實體 DOM，但這樣還沒辦法在頁面上顯示，需要有個類似 `ReactDOM.render` 的方法來幫助我們實現：

```js mount.js
export default (element, targetNode) => {
  targetNode.appendChild(element);
};
```

```js main.js
import createElement from './createElement';
import render from './render';
import mount from './mount';

const vRootApp = createElement('div', {
  attrs: {
    id: 'root',
  },
  children: [
    'Hello VDOM',
    createElement('img', {
      attrs: {
        src: 'http://placekitten.com/200/300',
      },
    }),
  ],
});

const rootApp = render(vRootApp);
mount(rootApp, document.getElementById('rootApp'));
```

很簡單，就把我們產生的 Real DOM `appendChild` 到 targetNode 下就好。

或是也能用 `targetNode.replaceWith(element);` 的方式直接取代掉 targetNode。（不過要注意一下 IE 是無法使用的喔！）

![Mounted DOM](/img/arvinh/mount-to-html.png)

# Diff Virtual DOM - Reconciliation

知道怎麼產生 Virtual DOM 並繪製到頁面上後，也是時候進入重頭戲了！

如前言所說，Virtual DOM 作為我們操作 Real DOM 的一層緩衝，我們比較經過狀態變化後產生的新舊 Virtual DOM 來找出實際需要更新的 Real DOM 位置，如此一來，儘管每次都重新 Render，實際更新的 DOM 也不會是全部，可以大幅改善直接重新渲染的效能問題。

而 tree diff 的演算法其實很複雜，如果用 [Tree Edit Distance](https://grfia.dlsi.ua.es/ml/algorithms/references/editsurvey_bille.pdf) 的方式遞迴檢查每個節點，複雜度將可達到 O(n^3)，是非常驚人的數字，幾乎無法在短時間處理完，因此 React 所提出的 reconciliation 制定了一些策略，來將複雜度從 O(n^3) 降至 O(n)。[React 官方文檔其實說明得很清楚](https://reactjs.org/docs/reconciliation.html)。

主要有兩個假設：

1. 只需要比較同一層的節點，同一層內的元素若擁有不同的 type，往下長出的樹就會不同。
2. 同樣 type 的元件，開發者可以使用 `key` 這個 props 來決定其子樹是否需要重新 render。

如假設一提及，我們只比較新舊兩棵 Virtual DOM Tree 中，同個父節點下的所有子節點，若發現某個節點不存在了，那就整個子樹都會刪除不去進一步比較。

![只比同 level 的 node](/img/arvinh/vdom-level-comparison.png)

這樣做的意思就是說，如果今天發生了一些跨層級的操作，像是整顆子樹被搬移到另一個節點上，對 React 來說，會是刪掉原有的子樹，然後重新在新的位置建立一模一樣的子樹出來：

![刪掉原子樹，在新位置重建](/img/arvinh/vdom-cross-level-modify.png)

> Note: 實際上 React 在這兩個假設下，還做了許多更細節的事情（component diff、element diff），可以先去參考這篇很久之前的[文章](https://zhuanlan.zhihu.com/p/20346379)，再去閱讀 [React fiber 的介紹](https://github.com/acdlite/react-fiber-architecture)。

基於這兩個假設我們可以開始實作簡單版的 Virtual DOM Diffing 演算法，基本上有四個 cases 處理：

1. newTreeRoot 為 undefined，也就是某個節點被刪除了。
2. 兩個 Node 都是純字串。
3. 一個 Node 為純字串，一個 Node 為 Virtual Element。
4. 新舊 TreeRoot 的 TagName 不同。

根據這四種 cases 我們個別處理，並且回傳一個 `patch` 函數，供之後來將 diff 完的結果 attach 到 Real DOM 上 （Note: r 開頭的都代表 Real DOM，v 開頭為 Virtual DOM）：

```js diff.js
import render from './render';
const diff = (oldVTreeRoot, newVTreeRoot) => {
  // 假設 oldVTreeRoot 一定都存在，只有 newVTreeRoot 有機會被刪除，也就是 undefined
  if (newVTreeRoot === undefined) {
    // 回傳 patch 函數，會接收 Real DOM，這邊 r 開頭的都代表 Real DOM，v 開頭為 Virtual DOM
    return rNode => {
      // 因為新的 Virtual DOM Tree 是空的，所以回傳的 Patch 函式就是直接把 Real DOM 刪除。
      rNode.remove();
      return undefined;
    }
  }
  if (typeof oldVTreeRoot === 'string' ||
    typeof newVTreeRoot === 'string') {
    if (oldVTreeRoot !== newVTreeRoot) {
      // 這邊包含兩種 cases：
      // Case 1：新舊 Virtual DOM Tree 其中一個為 string，一個為 Virtual Node，所以當然會 !==
      // Case 2：是兩者都為 string，但 !==
      // 我們直接根據新的 Virtual Tree render 新的 Real Tree，並 replace 掉原本的 Real Tree
      return rNode => {
          // 回傳 patch 函數
          const rNewNode = render(newVTreeRoot);
          rNode.replaceWith(rNewNode);
          return rNewNode;
       };
    } else {
      // 若都為 string 且值相同，那就不用改。
      return rNode => rNode; // 回傳 patch 函數
    }
  }
  if (oldVTreeRoot.tagName !== newVTreeRoot.tagName) {
    // 根據優化 Tree diffing 演算法的假設一，只要 tagName 不同，我們就直接重新 render。
    return rNode => {
      // 回傳 patch 函數
      const rNewNode = render(newVTreeRoot);
      rNode.replaceWith(rNewNode);
      return rNewNode;
    };
  }
  // ...
};
export default diff;
```

聰明的你看到這邊就會發問了：tag name 相同的 case 沒有處理到啊？

沒錯，如果新舊兩棵 Virtual Tree 的 tag name 都一樣，那我們還得比 attributes，而要比較兩個節點的所有 attributes，不如直接 replace 上新的就好。但要注意，因為 attributes 很多，所以會產生多個 patch 函數需要被 apply 到 Real DOM 上，我們額外用一個陣列暫存，最後回傳一個 wrapper patch 函數，把所有暫存的 patch 函數都 apply 到傳進來的 Real DOM ：

```js diffAttrs.js
const diffAttrs = (oldAttrs, newAttrs) => {
  // 因為 attributes 很多，需要一個 array 來存所有需要的 patch 函數
  const patches = [];
  // 放上新的 attributes
  for (const [k, v] of Object.entries(newAttrs)) {
    patches.push(rNode => {
      // 暫存 patch 函數
      rNode.setAttribute(k, v);
      return rNode;
    });
  }
  // 移除舊的 attributes
  for (const k in oldAttrs) {
    if (!(k in newAttrs)) {
      patches.push(rNode => {
        // 暫存 patch 函數
        rNode.removeAttribute(k);
        return rNode;
      });
    }
  }
  // 最後傳出去的外層 patch 函數
  return rNode => {
    for (const patch of patches) {
      // 把每個暫存的 patch 函數都 apply 到 Real DOM 上
      patch(rNode);
    }
    return rNode;
  };
};
export default diffAttrs;
```

處理完 attributes 後，我們還得考慮 children，diff children 的方式其實跟 diff 整棵樹一樣，但我們要考慮到子樹的長度：

1. `oldVChildren.length === newVChildren.length`，那就直接 `diff(oldVChildren[i], newVchildren[i])`，i 從 0 到 `oldVChildren.length`。
2. `oldVChildren.length > newVChildren.length`，跟 case 1 其實一樣，因為新子樹比較少，就代表有 Node 被刪除，在我們原本的 diff 函式中有處理了。
3. `oldVChildren.length < newVChildren.length`，新子樹比較長，那就先把舊子樹的所有點先 update 好，再把剩餘的新子樹 patch 上去。

從上述三個 cases 來看，我們橫豎都需要 loop oldVChildren 一次，最後若有多餘的 newVChildren 再想辦法 update 上去。另外，這邊一樣需要暫存多個 patch 函數，實作細節我註解在 code 裡比較清楚，最後回傳的 patch 函數比較特別：

```js diffChildren.js
const diffChildren = (oldVChildren, newVChildren) => {
  // 無論如何都 loop 過 oldVChildren 一次，把所有 diff 回傳的 patch 函數暫存在 childrenPatches 內
  // 這是一定會 apply 到 old tree 的部分。
  const childPatches = [];
  oldVChildren.forEach((oldVChild, i) => {
    childPatches.push(diff(oldVChild, newVChildren[i]));
  });
  // 接著我們看看 `newVChildren` 是否有多餘的子樹需要處理
  // 若有，我們產生的 patch 函數就是單純 `render` 出 Real Node 並且
  // appendChild 到 patch 傳進的 Real Node 上（實際要被 patch 的 parent 節點）
  const additionalPatches = [];
  for (const additionalVChild of newVChildren.slice(oldVChildren.length)) {
    additionalPatches.push(rNode => {
      rNode.appendChild(render(additionalVChild));
      return rNode;
    });
  }
  return rParent => {
    // 由於這是 children 的 patch，吃進來的會是 parent 的 Real DOM
    // 我們要抓出 `rParent.childNodes` 來針對 old tree 做 patch
    // zip 函數其實就是 lodash 的 zip，成對將 childPatches, rParent.childNodes 的元素並排傳出，這樣比較簡潔
    for (const [patch, rChild] of zip(childPatches, rParent.childNodes)) {
      patch(rChild);
    }
    // 最後把 new tree 多餘的 patches 直接 patch 到 parent 的 Real DOM 下即可（因為我們是 appendChild）
    for (const patch of additionalPatches) {
      patch(rParent);
    }
    return rParent;
  };
};
```

最後在我們原本的 `diff.js` 中的最後面加上：

```js diff.js
import render from './render';
const diff = (oldVTreeRoot, newVTreeRoot) => {
  if (newVTreeRoot === undefined) {
    // ....
  }
  if (typeof oldVTreeRoot === 'string' ||
    typeof newVTreeRoot === 'string') {
    if (oldVTreeRoot !== newVTreeRoot) {
      // ....
    } else {
      // ....
    }
  }
  if (oldVTreeRoot.tagName !== newVTreeRoot.tagName) {
      // ....
  }
  const patchAttrs = diffAttrs(oldVTreeRoot.attrs, newVTreeRoot.attrs);
  const patchChildren = diffChildren(oldVTreeRoot.children, newVTreeRoot.children);

  return rNode => {
    patchAttrs(rNode);
    patchChildren(rNode);
    return rNode;
  };
};
export default diff;
```

完整的 diff code 可以看這邊 [codesandbox](https://codesandbox.io/s/434xr5mr84)

到這邊為止，Virtual DOM 算是告一段落了！

## 最後修改下 main.js，做點變化讓大家看製作出的 VDOM 效果

我們讓 `createVApp` 柯里化，多傳一個參數 `count` 進去改變 attributes 跟圖片尺寸，接著 `setInterval` 讓每兩秒產生一個隨機數字當作 `count` 值，用來 update 我們的節點：

```js main.js
const createVApp = count => createElement('div', {
  attrs: {
    id: 'root',
    dataCount: count, // we use the count here
  },
  children: [
    'Hello Kitty',
    createElement('img', {
      attrs: {
        src: `http://placekitten.com/${count}00/${count}00`,
      },
    }),
  ],
});

let vApp = createVApp(0);
const rApp = render(vApp);
let rRootEl = mount(rApp, document.getElementById('rootApp'));
setInterval(() => {
  const n = Math.floor(Math.random() * 10);
  const vNewApp = createVApp(n);
  const patch = diff(vApp, vNewApp);
  // 每次 patch 完就 assgin 回原有變數，這樣下個 interval 才會抓到更新的樹
  rRootEl = patch(rRootEl);
  vApp = vNewApp;
}, 2000);
```

效果如下，可以看到圖片一直變動，但是我們真的只改到了需要改的節點與 attributes，並不會整個頁面重新刷新：

![demo](/img/arvinh/vdom-demo.gif)

## 結論

雖然沒辦法跟市面上實際的 VDOM 相提並論，但是從這簡單的實作可以很清楚的知道整個概念與要解決的問題，我覺得是蠻不錯的小練習，接下來再去看 React 或是 Vue 在這方面的實作應該會比較有頭緒一些！
最後再附上一次 codesandbox 連結讓想玩的人直接試試：[codesandbox](https://codesandbox.io/s/434xr5mr84)

## 資料來源

1. [Video: Building a Simple Virtual DOM from Scratch - Jason Yu](https://youtu.be/85gJMUEcnkc)
2. [Blog: Building a Simple Virtual DOM from Scratch - Jason Yu](https://dev.to/ycmjason/building-a-simple-virtual-dom-from-scratch-3d05#mount-node-target)
3. [React 源碼剖析系列 － 不可思議的 react diff](https://zhuanlan.zhihu.com/p/20346379)
4. [深度剖析：如何实现一个 Virtual DOM 算法](https://github.com/livoras/blog/issues/13)

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化