---
title: Reactive Programming 簡介與教學(以 RxJS 為例)
date: 2016-05-28 00:03:18
tags: Rx, RxJS
---

Rx 其實也出來一段時間了，前陣子看到一篇[文章](https://gist.github.com/staltz/868e7e9bc2a7b8c1f754)解釋得蠻好的，擷取重點並加入一些自己的心得範例供大家參考，畢竟 Rx 中文的介紹似乎少了點...

相信聽過 Rx 的讀者，應該也會耳聞 RxJS, RxJAVA, RxAndroid等等，因此被 Google 慣壞的我們一定會試著去搜尋一下 Rx 到底是什麼

然後在 Google Search Result page 的最下方會看到 MSDN 的 Reactive Extensions 和 ReactiveX 

![Rx goolge search result](/img/arvinh/RxGoogleResult.png "Rx goolge search result")

好的看來就是我們要找的東西，但這好眼熟喔? 是每天在寫的React嗎？難道是相關的東西？!

定睛一看會發現... 恩，基本上沒什麼關係。

Reactive Extensions 是 Microsoft open source 推廣的一個lib

`Reactive Extensions (Rx) is a library for composing asynchronous and event-based programs using observable sequences and LINQ-style query operators.`

網站連結：[rx](https://rx.codeplex.com/)
網站連結：[ReactiveX.io](http://reactivex.io/)

![ReactiveX](/img/arvinh/reactiveX.png "ReactiveX")

`ReactiveX is a combination of the best ideas from
the Observer pattern, the Iterator pattern, and functional programming`

ok, Observer pattern, Iterator patter, functional programming 都聽過，而這些串起來似乎就是傳說中的 Reactive Programming.

# What is Reactive Programming?

Reactive Programming 是一種以 asynchronous data streams 為中心思想出發的程式撰寫方式，比較常聽到的是 asynchronous event，像是 user click event, mouse hover event 等等，而這邊特別的則是 **<span style="color:red; font-style: italic;">data<span>** 與 **<span style="color:red; font-style: italic;">stream<span>**，顧名思義，Reactive Extensions 將 event 延伸為 data，並且注重在 stream （串流）上，也就是 **<span style="color:red; font-style: italic;">時間序列上的一連串資料事件<span>**，Rx讓你將任何事情都變化為 data streams : variables, user inputs, properties, caches, data structures 等等皆可，透過 Observe 這些 data streams，並依據其造成的 side effects 進行對應的動作。

### **Stream**:	 **時間序列上的一連串資料事件**

以一個 click event 來說，在 user 點擊的動作發生後，會有一段時間觸發了幾個事件 (event stream)：value, error or completed signal

![click event stream (source：https://gist.github.com/staltz/868e7e9bc2a7b8c1f754)](/img/arvinh/clickeventstream.png "click event stream ")

而在 Reactive Programming 的概念下，你可以把任何事情都看作 **Stream**，並且 **Observe** stream 中的變化，以下面一個例子來說：

假設我們想要印出一個包含 1 到 5 的 Array，一般我們會這樣做：

[example 1](http://jsbin.com/fiyiyo/edit?js,console,output)
``` javascript iterate1To5.js
	var source = [1,2,3,4,5];
	source.map((item) => {
	  console.log("onNext: "+item);
	})
```

然而，以Rx來說，任何事情都要 Observable，因此我們可以這樣做：

``` javascript observe1To5.js
	// Creates an observable sequence of 5 integers
	var source = Rx.Observable.range(1, 5);

	// catch every status and print out value
	var subscription = source.subscribe(
	    x => console.log('onNext: ' + x),
	  
	    e => console.log('onError: ' + e.message),
	  
	    () => console.log('onCompleted'));
```
在上面的例子中，我們創建了一個 Observable 的整數陣列，並且透過 **subscribe** 的動作去 **listening** 這個陣列，當有我們設定的 event 觸發時，我們就會 **observe** 到，並採取對應動作，這基本上就是 **Observer Design Pattern** 做的事情

## Why RxJS

以 Javascript 來說，想要抓取這些事件，一般可以用 callback 或是 Promise 來達成，然而 Promise 主要設計於一次性的事件與單一回傳值，而 RxJS 除了包含 Promise 外，提供了更豐富的整合應用。

|                              | Single return value | Mutiple return values                  |
|------------------------------|---------------------|----------------------------------------|
| Pull/Synchronous/Interactive | Object              | Iterables (Array/Set/Map/Object) |
| Push/Asynchronous/Reactive   | Promise             | Observable                             |

還記得前面 ReactiveX 的定義嗎？ "combination of Observer pattern, **Iterator pattern** and functional programming" 

RxJS 結合 Array#extras 的優點，讓你能夠方便處理 **Multiple return values**

延伸上面的例子來說：

[example 2](http://jsbin.com/yizequ/edit?html,js,console,output)
``` javascript iteratorPattern.js
	const data = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'];
	const source = Rx.Observable
	                 .interval(500)
	                 .take(6)
	                 .map(i => data[i])

	const result = source

	result.subscribe(x => console.log(x));

```
當你 subscribe 這個 Observale 的 data source 時，他能讓你 **監聽** 陣列中，每 500ms (interval) 取一個值 (map(i => data[i])) 並取 6 次 (take(6))

再回到最早的 click event 來說，假若我們想要能夠抓取 single click 與 double click 的事件，用最原始的 javascript 可能會需要許多變數來紀錄狀態、時間等等，但透過 RxJS 提供的 library，你只需要短短四行 code 就可以達成：

[完整範例](http://jsfiddle.net/staltz/4gGgs/27/)
``` javascript multiClickStream.js
	var multiClickStream = clickStream
	    .buffer(function() { return clickStream.throttle(250); })
	    .map(function(list) { return list.length; })
	    .filter(function(x) { return x >= 2; });
```

![Double click event (source：https://gist.github.com/staltz/868e7e9bc2a7b8c1f754)](/img/arvinh/muticlick.png "double click")     

由上圖可以清楚看到，RxJS 幫你把 Stream 上的 event 依照你想要的時間做 **整理**，`buffer` 住觸發時間在 250ms 間的 click events，並且利用 `map` 函式抓出每個 event list 的長度，並進一步抓出長度大於 2 ，也就是 double click 的 event 出來。

接著你只需要 `subscribe` 你剛剛定義的 event stream，即可做出反應(reaction)

``` javascript multiClickStream.js
	multiClickStream.subscribe(function (numclicks) {
	    document.querySelector('h2').textContent = ''+numclicks+'x click';
	});
```

### More examples (Autocompletion service)

以現在的 web app 來說，大量依賴 user 互動的效果與呈現，在不影響使用者體驗的前提下，多是用非同步的方式去抓取資料、渲染頁面等等，因此 Rx 系列的出現絕對是一個很大的助益。

最後讓我們再以一個例子來做結尾，利用 RxJS 與 Jquery 打造 Wikipedia Autocompletion Service。

[完整範例](http://jsbin.com/yojuwu/edit?html,js,output)
(source: http://xgrommx.github.io/rx-book/why_rx.html)

### Step 1
``` javascript
	var keyups = Rx.Observable.fromEvent(input, 'keyup')
	    .map(e => e.target.value)
	    .filter(text => text.length > 2);

	/* Now throttle the input for 500ms */
	var throttled = keyups.throttle(500 /* ms */);

	/* Now get only distinct values, so we eliminate the arrows and other control characters */
	var distinct = throttled.distinctUntilChanged();
```

1. 我們先利用 Rx.Observable.fromEvent 來 Create 一個 binding keyup event 的 Observalbe keyups object，並且針對每次事件發生時，回傳被綁定的元素其 value 值長度大於二的

2. 接著設定 keyups 的 throttle 時間，將 500ms 內的input當做一次event去觸發

3. 再來剔除掉不相干的控制輸入，只抓取 distinct 的 value

### Step 2
``` javascript
	function searchWikipedia (term) {
	    return $.ajax({
	        url: 'http://en.wikipedia.org/w/api.php',
	        dataType: 'jsonp',
	        data: {
	            action: 'opensearch',
	            format: 'json',
	            search: term
	        }
	    }).promise();
	}
```

4. 簡單撰寫一個 ajax 來 fetch search api
5. 這邊我們直接 return promise，RxJS 會幫你 wrap 起來變成 Obserbale，或是你也可以利用 `Rx.Observable.fromPromise `來將原有的 Promise 改裝

### Step 3

``` javascript
	var suggestions = distinct.flatMapLatest(searchWikipedia);
```

6. 利用 [flatMapLatest](http://xgrommx.github.io/rx-book/content/observable/observable_instance_methods/flatmaplatest.html) 將剛才的 Observable object `distinct` 與 `searchWikipedia` function 做結合，then we good to go! (先不管flatMapLatest是什麼，總之他會將 distinct 這個 Observable sequence內的元素丟給 searchWikipedia，並將回傳回來的資料再轉換成 Observable sequence，讓人可以 subscribe)

### Step 4

```javascript
	suggestions.subscribe(data => {
	    var res = data[1];

	    $results.empty();

	    $.each(res, (_, value) => $('<li>' + value + '</li>').appendTo($results));
	}, error => {
	    /* handle any errors */
	    $results.empty();

	    $('<li>Error: ' + error + '</li>').appendTo($results);
	});
```

7. 接著就是 Subscribe 剛剛的 Observable sequence **suggestions**，並將 listen 到的資料 show 出來

就這麼簡單完成了一個 Autocompletion 的 service 了！


### 題外話

先簡單介紹什麼是 flatMap 與 flatMapLatest，畢竟剛剛範例有用到，而實際上 RxJS 還有很多複雜的 function 可以應用，待之後我有時間再繼續專研吧！但有興趣的讀者可以在文章最下方的連結找到資源。

flatMap 會將 一個 Observable Sequence 的元素 映射到 另一個新的 Observable Sequence，並且subscribe 原先的 Observable Sequence 的人也都可以聽得到

簡單的例子如下：

```javascript flatMap
	console.clear();
	var source = Rx.Observable
	    .range(1, 2)
	    .flatMap(function (x) {
	        return Rx.Observable.range(x, 2);    
	    });

	var subscription = source.subscribe(
	    function (x) {
	        console.log('Next: ' + x);
	    },
	    function (err) {
	        console.log('Error: ' + err);   
	    },
	    function () {
	        console.log('Completed');   
	    });

    // Result: 
	// => Next: 1 
	// => Next: 2  Rx.Observable.range(1, 2)
	// => Next: 2 
	// => Next: 3  Rx.Observable.range(2, 2)
	// => Completed    
```

See? 他會把 sequence 中的元素丟進 callback，並回傳 Observable sequence，你也可以丟入 Promise，就像範例中做的。

而 flatMapLatest 則是只會進行最後一次的 sequence，以剛剛的範例來說，最後subscribe的人接收到的會是最新的那個 Observable sequence 的結果！而不會每打一個字所搜尋的結果都一直累加顯示上去。

## One more thing

在我前面放的圖中，描繪 Click event 的 叫做 **marble** 圖，這邊有個網站可以讓你以視覺化互動的方式去操作這些 event，幫助你理解 Rx 當中的各個 function 之功用！非常推薦！！
去玩玩吧！[rxmarbles](http://rxmarbles.com/)

## 總結

Rx 真的是蠻有趣的東西，提供的lib又號稱毫無相依性，可以應用在各種framework上方，只是必須要懂得如何Think in Reactive Programming，否則這些lib的用法還真的是不好理解，這篇拋磚引玉簡單介紹一下，之後會再有更深入的研究！ 有什麼說明不對的地方也請見諒與指教！



參考資料

* [The introduction to Reactive Programming you've been missing (by @andrestaltz)](https://gist.github.com/staltz/868e7e9bc2a7b8c1f754)
* [Rx-book](http://xgrommx.github.io/rx-book/)
* [Reactive-Extensions RxJS](https://github.com/Reactive-Extensions/RxJS/tree/master/doc)
* [rxmarbles](http://rxmarbles.com/)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化