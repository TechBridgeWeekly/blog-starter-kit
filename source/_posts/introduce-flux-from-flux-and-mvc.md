---
title: 從 Flux 與 MVC 的差異來簡介 Flux
date: 2016-04-29 23:09:42
tags:
  - Flux
  - React
  - MVC
---
今天這篇主要是想藉由`Flux`的介紹，進而探討`Flux`與`MVC`的差異，到底`Flux`做對了什麼，而`MVC`又犯了什麼錯呢？

首先我們回顧一下[2014 F8大會 - Hacker Way](https://www.youtube.com/watch?v=nYkdrAPrdcw)的影片
<a href="http://www.youtube.com/watch?feature=player_embedded&v=nYkdrAPrdcw" target="_blank">
<img src="http://img.youtube.com/vi/nYkdrAPrdcw/0.jpg" alt="2014 F8大會 - Hacker Way" width="240" height="180" border="10" />
</a>
其中 Jing Chen 用了兩張圖來做對比，說明MVC的觀念在複雜的Application下難以維護。

![facebook MVC](/img/arvinh/flux-react-mvc.png "facebook MVC")

經驗豐富一點的Developer看到這張圖的第一個念頭應該會是："What the fxxx... 誰家的MVC長這樣啊？" 沒錯！Jing Chen 事後在Reddit上也有回覆，主要是想利用這張圖把MVC在大型架構下，資料 與 視圖 之間的 `bi-directional data flow`，容易造成`cascading effects`的問題凸顯出來。

為了解決她說的這個問題，Facebook 提出了`Flux`

## Flux
Flux是一種讓你很容易做到 one-way data flow 的<span style="color:red">概念</span>，讓你View中的每個component的state都能夠`predictable`。

![facebook Flux](/img/arvinh/flux-react.png "facebook Flux")

### Views Dispatch Actions

`Dispatcher`是一個重要的event system，用來broadcast events以及registers callbacks，一般來說Dispatcher是唯一且global的，可以參考Facebook的[Dispatcher Library](https://github.com/facebook/flux/blob/master/src/Dispatcher.js)（題外話，有許多一些Flux的framework並非這樣使用）

簡單來看個Dispatcher的例子：

```js
// 假設你initiate一個dispatcher
var AppDispatcher = new Dispatcher();

//.
//..
//...
//在你的 component.jsx 中，可能會有這樣的程式
createNewItem: function( evt ) {
  AppDispatcher.dispatch({
    actionName: 'newPhoto',
    newItem: { name: 'Happy Holiday' } // example data
  });
}

render: function() {
  return (<button onClick={ this.createNewItem }>New Photo</button>);
}
```

當每次的`onClick`發生後，`View`就會透過`Dispatcher`dispatch出一個`Action`，該Action可以包含一個`payload`，說明`你想做什麼事情`以及`你需要操作什麼資料`。

### Store Responds to Dispatched Actions

Store在Flux的架構內，通常是Singleton(一樣，有些framework並非這樣做，尤其是想達成isomorphic時，可以參考[Yahoo Fluxible](http://fluxible.io/))

在Flux的概念中，Store基本上是你唯一可以**操作資料**與**儲存資料**的地方。去除操作資料的部分，聽起來有點像MVC中的Model? 更明確一點來說，**Store contains Models**

舉例來說，當你需要存放一些照片以及其Meta data時，你會Create一個PhotoStore來存放Photo model與Meta model。你會依照資料的`Domain`來切割你的Store。

```js
var PhotoStore = {
  // collection of model data
  photos: []
}

AppDispatcher.register(function(payload) {
  switch( payload.actionName ) {
    case 'newPhoto':
      PhotoStore.photos.push(payload.newPhoto);
      break;
  }
})
```

`Store`會向`Dispatcher`註冊`Callback`，依照各種action的類別執行相對應的資料操作。

### Store Emits "Change" Event to View

當你的Store資料做完更新後，要告訴前端頁面去刷新視圖，通常可以在Store註冊的Callback中執行以下動作：
```js
AppDispatcher.register(function(payload) {
  switch( payload.actionName ) {
    case 'newPhoto':
      PhotoStore.photos.push(payload.newPhoto);
      // trigger "Change" event 通知View去做更新
      PhotoStore.trigger('change');
      break;
  }
})
```

接著，如果你是搭配React當作你的View的話，可能會在`componentDidMount`時，binding一個Store listener

```js
componentDidMount: function() {  
    PhotoStore.bind( 'change', this.photoChange );
},
```

在listener中重新fetch store資料，並且setState來re-render Component

```js
photoChange: function() {  
    var newPhotoData = PhotoStore.getPhoto();
    this.setState({
      photos: newPhotoData
    });
}
```

你的Component的render function大概會像這樣：

```js
render: function() {
  var photosComponet = this.state.photos.map(function(photo, i){
    return (
      <li key={'photo'+i}>
        {photo}
      </li>
    );          
  });
  return (
    <div>
      <ul>
        {photosComponet}
      </ul>
    </div>
  );
}

```

看完簡單的Flux介紹後，讓我們再複習一次Flux的流程圖

![facebook Flux](/img/arvinh/flux-react.png "facebook Flux")

相信在業界打滾多年的Developer們應該早有疑惑了，Flux的那張圖，跟最原始的MVC圖不是很像嗎？！

![MVC definition in wikipedia](https://upload.wikimedia.org/wikipedia/commons/thumb/a/a0/MVC-Process.svg/500px-MVC-Process.svg.png "MVC definition in wikipedia")

User操作View所產生的任何event，都會經由Controller來修改與更動相關的Model，而Model再告知View是否需要做更動，聽起來也是蠻`one-way direction`的呀。

事實上，MVC 跟 Flux 都只是一個概念，因此有各種不同的實作，加上MVC在`資料流`的處理上，並不像Flux一般有較為明確的定義，多數時候Model的更動與View的刷新可能會透過Controller來管理，讓Model單純存放data。

如此一來，假若今天View的操作更動了Model，而Model的變化又刷新了View，在系統龐大的時候，一來一往，就會讓你的資料與頁面狀態變得非常複雜，要追蹤某個頁面的變動到底是誰觸發的，或是哪個資料改變了，你必須從Controller去慢慢trace。而若是遵照Flux的流程，任何View的update都只要去追蹤其State的來源Store即可，有一個明確的flow可以遵循，並且每個View所需要監聽的資料來源，可以依照Store來區分，這之間的資料流不會互相干擾。
另外一個Flux的好處是，能夠更輕鬆的做出更Unit的Unit test。這是你在複雜的Controller中難以達成的。

當然，你可能會想：“這是你MVC用得不好“。 

我覺得這樣講也沒什麼不對，如果你MVC用得很熟很順手，的確單單是Flux這個東西對你的誘因可能不高，但當一間明星公司大力Promote，對於基本概念的定義又夠清楚單純時，還是值得你試試。更別說Flux搭配上React的宣告式寫法，用起來更是如魚得水。

### 結論

Flux做的是：
1. 改善`資料狀態`與`視圖狀態`的 Data Flow
2. 讓頁面的狀態`Predictable`
3. 資料流不會互相污染
4. 讓你的測試更加容易

而MVC在關注點分離上的貢獻不可小覷，重視在將資料(Model)、視圖(View)、邏輯(Controller)拆開，各自負責各自的工作。

因此並非是MVC不好、不對，所以我們應該採用Flux；Flux是在MVC建立的基礎下，定義出一個清楚的`one-way direction`資料流，並且透過`Action`、`Dispatcher`與`Store`來幫助整個概念的實現。

### One more thing

如同前面所提，Flux的實作有很多種，這邊介紹的只是最基本的流程，很多Framework在設計自己的Dispatcher, Action 與 Store時，會有不同的方式，或許可以從這邊的比較下去看看
* [Flux Comparison](https://github.com/voronianski/flux-comparison)
* [Awesome React](https://github.com/enaqx/awesome-react)

參考資料
* [Flux for stupid people](http://blog.andrewray.me/flux-for-stupid-people/)
* [Facebook: MVC Does Not Scale, Use Flux Instead](http://www.infoq.com/news/2014/05/facebook-mvc-flux)


關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化