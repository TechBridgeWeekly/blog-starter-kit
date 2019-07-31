---
title: DOM 的事件傳遞機制：捕獲與冒泡
date: 2017-07-15 13:08:56
tags:
  - DOM
  - event
  - bubble
  - capture
  - propagation
author: huli
---

# 前言

（補充：感謝 othree 前輩的指點，指出這其實是在講 DOM 裡面事件傳遞的順序，因此把標題以及內文修正，原標題為：JavaScript 的事件傳遞機制：捕獲與冒泡）

今天為大家帶來的內容是 DOM 裡面的事件傳遞機制，而與這些事件相關的程式碼，相信大家應該不太陌生，就是`addEventListener`, `preventDefault`跟`stopPropagation`。

簡單來說，就是事件在 DOM 裡面傳遞的順序，以及你可以對這些事件做什麼。

為什麼會有「傳遞順序」這一詞呢？假設你有一個`ul`元素，底下有很多`li`，代表不同的 item。當你點擊任何一個`li`的時候，其實你也點擊了`ul`，因為`ul`把所有的`li`都包住了。

假如我在兩個元素上面都加了`eventListener`，哪一個會先執行？這時候呢，知道事件的執行順序就很重要。

另外，由於某些瀏覽器（沒錯，我就是在說 IE）的機制比較不太一樣，因此那些東西我完全不會提到，有興趣的可以研究文末附的參考資料。

# 簡單範例

為了之後方便說明，我們先寫一個非常簡單的範例出來：

``` html
<!DOCTYPE html>
<html>
<body>
  <ul id="list">
    <li id="list_item">
      <a id="list_item_link" target="_blank" href="http://google.com">
        google.com
      </a>
    </li>
  </ul>
</body>
</html>
```

在這個範例裡面，就是最外層一個`ul`，再來`li`，最後則是一個超連結。為了方便辨識，id 的取名也跟階層架構有關係。

DOM 畫成圖大概是長這樣：

![](/img/huli/event/event_p1.png)


有了這一個簡單的 HTML 結構之後，就可以很清楚的說明 DOM 的事件傳遞機制了。

# 事件的三個 Phase

要幫一個 DOM 加上 click 的事件，你會這樣寫：

``` js
const $list = document.getElementById('list');
$list.addEventListener('click', (e) => {
  console.log('click!');
})
```

而這邊的`e`裡面就蘊含了許多這次事件的相關參數，其中有一個叫做`eventPhase `，是一個數字，表示這個事件在哪一個 Phase 觸發。

``` js
const $list = document.getElementById('list');
$list.addEventListener('click', (e) => {
  console.log(e.eventPhase);
})
```

`eventPhase`的定義可以在[ DOM specification ](https://www.w3.org/TR/DOM-Level-2-Events/events.html#Events-interface)裡面找到：

``` C
// PhaseType
const unsigned short      CAPTURING_PHASE                = 1;
const unsigned short      AT_TARGET                      = 2;
const unsigned short      BUBBLING_PHASE                 = 3;
```

這三個階段，就是我們今天的重點。

DOM 的事件在傳播時，會先從根節點開始往下傳遞到`target`，這邊你如果加上事件的話，就會處於`CAPTURING_PHASE`，捕獲階段。

`target`就是你所點擊的那個目標，這時候在`target`身上所加的`eventListenr`會是`AT_TARGET`這一個 Phase。

最後，事件再往上從子節點一路逆向傳回去根節點，這時候就叫做`BUBBLING_PHASE `，也是大家比較熟知的冒泡階段。

這邊用文字你可能會覺得霧煞煞，我直接引用一張[ w3c 講 event flow 的圖](https://www.w3.org/TR/DOM-Level-3-Events/#event-flow)，相信大家就清楚了。

![](/img/huli/event/eventflow.png)

你在點擊那一個`td`的時候，這一個點擊的事件會先從`window`開始往下傳，一直傳到`td`為止，到這邊就叫做`CAPTURING_PHASE`，捕獲階段。

接著事件傳遞到`td`本身，這時候叫做`AT_TARGET`。

最後事件會從`td`一路傳回去`window`，這時候叫做`BUBBLING_PHASE`，冒泡階段。

所以，在看一些講事件機制的文章的時候，都會看到一個口訣：

> 先捕獲，再冒泡

就是這樣來的。

可是，我要怎麼決定我要在捕獲階段還是冒泡階段去監聽這個事件呢？

其實，一樣是用大家所熟悉的`addEventListener`，只是這函數其實有第三個參數，`true`代表把這個 listener 添加到捕獲階段，`false`或是沒有傳就代表把這個 listener 添加到冒泡階段。

# 實際演練

大概知道事件的傳遞機制之後，我們拿上面寫好的那一個簡單範例來示範一下，一樣先附上事件傳遞的流程圖（假設我們點擊的對象是`#list_item_link`）

![](/img/huli/event/event_p2.png)

接著，來試試看幫每一個元素的每一個階段都添加事件，看一看結果跟想像中的是否一樣：

``` js
const get = (id) => document.getElementById(id);
const $list = get('list');
const $list_item = get('list_item');
const $list_item_link = get('list_item_link');
  
// list 的捕獲
$list.addEventListener('click', (e) => {
  console.log('list capturing', e.eventPhase);
}, true)
  
// list 的冒泡
$list.addEventListener('click', (e) => {
  console.log('list bubbling', e.eventPhase);
}, false)
  
// list_item 的捕獲
$list_item.addEventListener('click', (e) => {
  console.log('list_item capturing', e.eventPhase);
}, true)
  
// list_item 的冒泡
$list_item.addEventListener('click', (e) => {
  console.log('list_item bubbling', e.eventPhase);
}, false)
  
// list_item_link 的捕獲
$list_item_link.addEventListener('click', (e) => {
  console.log('list_item_link capturing', e.eventPhase);
}, true)
  
// list_item_link 的冒泡
$list_item_link.addEventListener('click', (e) => {
  console.log('list_item_link bubbling', e.eventPhase);
}, false)
```

點一下超連結，console 輸出以下結果：

```
list capturing
1
list_item capturing
1
list_item_link capturing
2
list_item_link bubbling
2
list_item bubbling
3
list bubbling
3
```

1 是`CAPTURING_PHASE`，2 是`AT_TARGET`，3 是`BUBBLING_PHASE`。

從這邊就可以很明顯看出，事件的確是從最上層一直傳遞到 target，而在這傳遞的過程裡，我們用`addEventListenr`的第三個參數把 listener 添加在`CAPTURING_PHASE`。

然後事件傳遞到我們點擊的超連結（`a#list_item_link`）本身，在這邊無論你使用`addEventListener`的第三個參數是`true`還是`false`，這邊的`e.eventPhase`都會變成`AT_TARGET`。

最後，再從 target 不斷冒泡傳回去，先傳到上一層的`#list_item`，再傳到上上層的`#list`。

# 先捕獲，再冒泡的小陷阱

既然是先捕獲，再冒泡，意思就是無論那些`addEventListener`的順序怎麼變，輸出的東西應該還是會一樣才對。我們把捕獲跟冒泡的順序對調，看一下輸出結果是否一樣。

``` js
const get = (id) => document.getElementById(id);
const $list = get('list');
const $list_item = get('list_item');
const $list_item_link = get('list_item_link');
  
// list 的冒泡
$list.addEventListener('click', (e) => {
  console.log('list bubbling', e.eventPhase);
}, false)
  
// list 的捕獲
$list.addEventListener('click', (e) => {
  console.log('list capturing', e.eventPhase);
}, true)
  
// list_item 的冒泡
$list_item.addEventListener('click', (e) => {
  console.log('list_item bubbling', e.eventPhase);
}, false)
  
// list_item 的捕獲
$list_item.addEventListener('click', (e) => {
  console.log('list_item capturing', e.eventPhase);
}, true)
  
// list_item_link 的冒泡
$list_item_link.addEventListener('click', (e) => {
  console.log('list_item_link bubbling', e.eventPhase);
}, false)
  
// list_item_link 的捕獲
$list_item_link.addEventListener('click', (e) => {
  console.log('list_item_link capturing', e.eventPhase);
}, true)
```

一樣點擊超連結，輸出的結果是：

```
list capturing
1
list_item capturing
1
list_item_link bubbling
2
list_item_link capturing
2
list_item bubbling
3
list bubbling
3
```

可以發現一件神奇的事，那就是`list_item_link`居然是先執行了添加在冒泡階段的 listener，才執行捕獲階段的 listener。

這是為什麼呢？

其實剛剛上面有提到，當事件傳遞到點擊的真正對象，也就是 e.target 的時候，無論你使用`addEventListener`的第三個參數是`true`還是`false`，這邊的`e.eventPhase`都會變成`AT_TARGET`。

既然這邊已經變成`AT_TARGET`，自然就沒有什麼捕獲跟冒泡之分，所以執行順序就會根據你`addEventListener`的順序而定，先添加的先執行，後添加的後執行。

所以，這就是為什麼我們上面把捕獲跟冒泡的順序換了以後，會先出現`list_item_link bubbling`的原因。

關於這些事件的傳遞順序，只要記住兩個原則就好：

1. 先捕獲，再冒泡
2. 當事件傳到 target 本身，沒有分捕獲跟冒泡

[jsbin 範例程式碼](https://jsbin.com/mogujivera/edit?html,js,console,output)

# 取消事件傳遞

接著要講的是，這一串事件鏈這麼長，一定有方法可以中斷這一條鏈，讓事件的傳遞不再繼續。而這個方法相信大家應該都不陌生，就是：`e.stopPropagation`。

你加在哪邊，事件的傳遞就斷在哪裡，不會繼續往下傳遞。

例如說以上面那個例子來講，假如我加在`#list`的捕獲階段：

``` js
// list 的捕獲
$list.addEventListener('click', (e) => {
  console.log('list capturing', e.eventPhase);
  e.stopPropagation();
}, true)
```

這樣子，console 就只會輸出：

```
list capturing
1
```

因為事件的傳遞被停止，所以剩下的 listener 都不會再收到任何事件。

不過，在這邊依然有一個地方要特別注意。

這邊指的「事件傳遞被停止」，意思是說不會再把事件傳遞給「下一個節點」，但若是你在同一個節點上有不只一個 listener，還是會被執行到。

例如說：

``` js
// list 的捕獲
$list.addEventListener('click', (e) => {
  console.log('list capturing');
  e.stopPropagation();
}, true)
  
// list 的捕獲 2
$list.addEventListener('click', (e) => {
  console.log('list capturing2');
}, true)
```

輸出結果是：

```
list capturing
list capturing2
```

儘管已經用`e.stopPropagation`，但對於同一個層級，剩下的 listener 還是會被執行到。

若是你想要讓其他同一層級的 listener 也不要被執行，可以改用`e.stopImmediatePropagation();`

例如說：

``` js
// list 的捕獲
$list.addEventListener('click', (e) => {
  console.log('list capturing');
  e.stopImmediatePropagation();
}, true)
  
// list 的捕獲 2
$list.addEventListener('click', (e) => {
  console.log('list capturing2');
}, true)
```

輸出結果是：

```
list capturing
```

## 取消預設行為

常常有人搞不清楚`e.stopPropagation`跟`e.preventDefault`的差別，前者我們剛剛已經說明了，就是取消事件繼續往下傳遞，而後者則是取消瀏覽器的預設行為。

最常見的做法就是阻止超連結，例如說：

``` js
// list_item_link 的冒泡
$list_item_link.addEventListener('click', (e) => {
  e.preventDefault();
}, false)
```

這樣子，當點擊超連結的時候，就不會執行原本預設的行為（新開分頁或是跳轉），而是沒有任何事情發生，這就是`preventDefault`的作用。

所以呢，`preventDefault`跟 JavaScript 的事件傳遞「一點關係都沒有」，你加上這一行之後，事件還是會繼續往下傳遞。

有一個特別值得注意的地方是 W3C 的文件裡面有寫到：

> Once preventDefault has been called it will remain in effect throughout the remainder of the event's propagation.

意思就是說一旦 call 了`preventDefault `，在之後傳遞下去的事件裡面也會有效果。

我們來看一個範例：

``` js
// list 的捕獲
$list.addEventListener('click', (e) => {
  console.log('list capturing', e.eventPhase);
  e.preventDefault();
}, true)
```

我們在`#list`的捕獲事件裡面就先寫了`e.preventDefault()`，而根據文件上面所說的，這個效果會在之後傳遞的事件裡面一直延續。

因此，等之後事件傳遞到`#list_item_link`的時候，你會發現點超連結一樣沒反應。

# 實際應用

知道了事件的傳遞機制、取消傳遞事件跟取消預設行為之後，在實際開發上有什麼用處呢？

最常見的用法其實就是事件代理（Delegation），例如說你今天有一個 ul，底下 1000 個 li，如果你幫每一個 li 都加上一個 eventListener，你就新建了 1000  個 function。

但我們剛剛已經知道，任何點擊 li 的事件其實都會傳到 ul 身上，因此我們可以在 ul 身上掛一個 listener 就好。

``` html
<!DOCTYPE html>
<html>
<body>
  <ul id="list">
    <li data-index="1">1</li>
    <li data-index="2">2</li>
    <li data-index="3">3</li>
  </ul>
</body>
</html>
```

``` js
document.getElementById('list').addEventListener('click', (e) => {
  console.log(e.target.getAttribute('data-index'));
})
```

而這樣的好處是當你新增或是刪除一個 li 的時候，不用去處理跟那個元素相關的 listener，因為你的 listener 是放在 ul 身上。這樣透過父節點來處理子節點的事件，就叫做事件代理。

除此之外，我有想到幾個滿有趣的應用，大家可以參考看看。

例如說剛剛提到的`e.preventDefault()`，既然我們知道原理跟使用技巧，就可以這樣用：

``` js
window.addEventListener('click', (e) => {
  e.preventDefault();
  e.stopPropagation();
}, true);
```

只要這樣一段程式碼，就可以把頁面上所有的元素停用，點了都沒有反應，像是`<a>`點了不會跳出超連結，`<form>`按了`submit`也沒用，而且因為阻止事件冒泡，所以其他的`onClick`事件也都不會執行。

或是，也可以這樣用：

``` js
window.addEventListener('click', (e) => {
  console.log(e.target);
}, true)
```

利用事件傳遞機制的特性，在`window`上面使用捕獲，就能保證一定是第一個被執行的事件，你就可以在這個 function 裡面偵測頁面中每一個元素的點擊，可以傳回去做數據統計及分析。

# 結論

DOM 的事件傳遞機制算是 JavaScript 眾多經典面試題裡面相對簡單很多的，只要能掌握事件傳遞的原則跟順序，其實就差不多了。

而`e.preventDefault`與`e.stopPropagation`的差別在知道事件傳遞順序之後也大概能理解，前者就只是取消預設行為，跟事件傳遞沒有任何關係，後者則是讓事件不再往下傳遞。

希望這篇能讓大家理解 DOM 的事件傳遞機制，如果有哪邊有講錯，也麻煩大家不吝指證，感謝。

參考資料（比較推薦後面那些原文資料）：

1. [JavaScript 详说事件机制之冒泡、捕获、传播、委托](http://www.cnblogs.com/bfgis/p/5460191.html)
2. [Javascript 事件冒泡和捕获的一些探讨](https://github.com/zhukejin1223/blogs/blob/master/JAVASCRIPT/Javascript-event.md)
3. [浅谈 javascript 事件取消和阻止冒泡](http://wiki.jikexueyuan.com/project/brief-talk-js/event-cancellation-and-prevent-bubbles.html)
4. [What Is Event Bubbling in JavaScript? Event Propagation Explained](https://www.sitepoint.com/event-bubbling-javascript/)
5. [What is event bubbling and capturing?](https://stackoverflow.com/questions/4616694/what-is-event-bubbling-and-capturing)
6. [Event order](https://www.quirksmode.org/js/events_order.html)
7. [Document Object Model Events](https://www.w3.org/TR/DOM-Level-2-Events/events.html#Events-flow-capture)


關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好
