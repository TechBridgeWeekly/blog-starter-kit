---
title: Browser Rendering Optimization
date: 2016-04-02 00:13:00
tags:
	- Browser Rendering Optimization
	- frontend
	- web
	- web performance
	- 網頁效能
	- 網站效能 
---
> "My name is Barry Allen, and I'm the fastest man alive ..."
> "Wait..reverse flash is faster then you, zoom is faster than you.."
<!-- more -->

今天來談談Browser Rendering Optimisation。

相信大家小時候都好奇過早期的電影、卡通或動畫，是怎麼樣製作出來的，而我們也都知道基本上就是一連串的`畫面`以非常快速的方式做切換，矇騙你的視覺讓你感受到是連續的`影片`，而Browser其實也是類似的做法，當取得Server的回應後，瀏覽器便把接收到的HTML畫出來，而每當你的網頁有所變化時，Browser就會再依據其邏輯將網頁重新渲染一遍，也因此才能看到這麼多漂亮的效果。

而所謂browser rendering optimization就是要讓渲染的過程能夠更快速、更順暢，讓你的網頁畫面如夢似幻、細緻耀眼。

要做到這件事情，就必須提及FPS(Frames per Second)，顧名思義就是每秒畫幾個Frames，一般影片大概是[30fps](https://en.wikipedia.org/wiki/Frame_rate)，而多數螢幕畫面的更新頻率是60Hz，因此若我們能將fps極致提升到60fps，理當能有非常棒的畫面呈現！

知道目標後，let's do some math...
`60fps = 60 frames per second = 60 frames every 1000ms`
好的從我們優異的數學能力下得知，要達到60fps，我們繪製一個frames的時間必須在16.6666...ms左右(1000/60)

ok，那要怎麼知道自己的網頁每個frames花了多少時間繪製？拜Google大神所賜，只要打開Chrome的devTool，切換到Timeline的tab，就可以很容易的知道網頁在browser上的render時間。讓我們用畫面超級old school的pchome拍賣來Demo一下：

![Devtool-timeline(點圖放大)](/img/arvinh/googleDevTool-pchome.png "DevTool-timeline")

先別被圖中五顏六色的區塊嚇到，在我們說明這Tool該如何使用之前，必須先瞭解Browser的Render過程，以及這圖片中每個顏色所代表的意義。

接觸過HTML的人一定都知道所謂的DOM Tree，Browser會解析HTML並轉換成DOM Tree做操作，但知道[Render Tree](https://developers.google.com/web/fundamentals/performance/critical-rendering-path/render-tree-construction?hl=en)的人可能就不多了，Render tree由DOM tree與CSS結合產生，Browser就是依照Render Tree來決定該在畫面上呈現什麼東西。若你今天在某個div的css上加上`display: none`，那這個div就不會再Render tree上呈現。

![Render Tree (source from google)](/img/arvinh/RenderTree.png)

既然Render的過程也是一棵Tree，有點概念的人可能會想：每次style改變或是我更動DOM的位置時，這棵Tree也就會變動了吧？
沒錯！在整個Browser的Render過程中，就是會不斷去recalculating style、layout的關係，來建構這棵樹。

當然Browser的渲染過程不止這些，還必須載入js, css, web api等等的資源，因此有所謂的Rendering Pipeline

![Render Pipeline (source from google)](/img/arvinh/render_pipeline.png)
從圖上可以發現其順序為：
1. Javascript: 載入與執行JS/CSS
2. Style: 根據js與css計算style
3. Layout: 當style套入元素時，瀏覽器要檢查是否會影響到整個畫面的排列，並進行排列
4. Paint: 排列後有更動到的元素需要重新繪製（第一次載入的話當然是全部繪製）
5. Composite: 最後就是將所有元素重新合成回來（重新繪製的元素與原本的元素）

實際上並非每次的前端更動都會執行這五個步驟，若你只有切換顏色、圖片等不影響版面配置的動作，browser會跳過Layout，直接進行Paint；或著你捲動網頁、css的動畫效果，這類操作則會跳過style與layout，直接進行Composite。

接下來我們回到Timeline這個tool。

![Timeline record](/img/arvinh/Timeline2.png)
你可以直接在你想觀察的網頁打開Timeline，並重新整理，他就會自行錄製；或是你可以在你想觀察的操作步驟進行前（ex. 滑動頁面、打開menu bar等等）按下錄製。

![Timeline finish](/img/arvinh/Timeline3.png)

接著在你覺得適合的地方（通常就是步驟執行完後）按下Finish。

![Timeline result](/img/arvinh/Timeline1.png)

基本上就能得到下列的Timeline結果。
從這張圖片可以清楚看到每個frame花了多少時間，在Render pipeline中發生了哪些事情，值得注意的是，圖片右上角有個紅色小三角形，若你的frame中有這個標記，就代表Chrome認為這段frame有可以改善的部分。大家不妨到自己的網頁打開Timeline，看看有多少東西需要改進XD



在認識了工具後，總是要實際操作一遍才會有感受，這邊以三種角度來進行Optimization。

## Javascript

一般來說，前端工程師常常會利用`setTimeout`或是`setInterval`來製作一些動畫效果，你可能會寫下面這樣的code:

[example setInterval](https://jsbin.com/xuconawipi/5/edit?html,js,output)

``` javascript setInterval.js
var timer = {time: 0};
function counter(timer) { 
  timer.time = timer.time+1;
  document.querySelector("#counter").innerHTML = timer.time;
}
setInterval(counter.bind(null,timer), 1000); 
```

或著是

[example setTimeout](https://jsbin.com/yocokorelu/edit?html,js,output)
``` javascript setTimeout.js
		var timer = {time: 0};
		function counter(timer) { 
			setTimeout(counter.bind(null,timer), 1000);
			timer.time = timer.time+1;
			document.querySelector("#counter").innerHTML = timer.time;
		}
		counter(timer);
```
兩者都可以讓你達到同樣的效果，但是setTimeout跟setInterval對Browser來說都有個主要缺點，就是他**<span style="color:red; font-style: italic;">想執行時就會執行，而不會依據你的Browser狀況<span>**；另外當你切換到別的Tab時，setTimeout還會繼續Render。（理論上user看不到畫面，Browser就可以不用執行，以節省效能）
*<span style="color:rgba(228, 85, 85, 0.81); font-style: italic;">不過現在許多瀏覽器都已經利用某些方式讓setTimeout在不需要Render時暫停工作。</span>*

除了setTimeout與setInterval外，我們其實還有另一個選擇 <span style="color:red;">`requestAnimationFrame`</span>

假設我們有個Draw的動畫function，用setTimeout是這樣實作：
``` javascript draw.js	
		function draw() {
		    setTimeout(draw, 16); // 16ms per frame!
		    // Drawing
		}
		draw();
```
若是用requestAnimationFrame：
``` javascript draw-RAF.js	
		function draw() {
			// Drawing
			requestAnimationFrame(draw);
		}
		requestAnimationFrame(draw);
```
就這麼簡單，用了<span style="color:red;">`requestAnimationFrame`</span>之後，Browser就會綜合考量javascript所產生的動畫，一起刷新螢幕，並在動畫不在viewport時，暫停工作，以節省資源。

當然你會想說，那如果我想控制animation的timing怎麼辦？以前面的counter例子來看，你可以這樣做：

[example reqeustAnimationFrame](https://jsbin.com/qerude/edit?html,js,output)
``` javascript RAF_timeer		
		var timer = {time: 0};
		function counter(timer) { 
		  setTimeout(function(){
		    requestAnimationFrame(counter.bind(null, timer));
		    timer.time = timer.time+1;
		    document.querySelector("#counter").innerHTML = timer.time;
		  }, 1000);
		}
		counter(timer);
```
## Style & Layout

除了JS外，在頁面上操作畫面大多免不了觸發`Style` & `Layout`這兩個Render Pipeline的步驟。既然免不了這些步驟，我們能做的就是盡量**減少**這些步驟的產生，大家可以到這裡（[How (not) to trigger a layout in WebKit](http://gent.ilcore.com/2011/03/how-not-to-trigger-layout-in-webkit.html)）看看在有哪些操作我們要盡量減少。

當然，光說不練感受不到差別，讓我們來練習一個例子：

[example Layout trigger](https://jsbin.com/yesika/edit?html,css,js,output)
``` javascript befor-optimization.js
		(function() {
		  // noprotect
		  var sizer = document.querySelector('.sizer');

		  document.querySelector('.set-size').addEventListener('click', function(event) {
		    var ps = document.querySelectorAll('.article .article-block');
		    var i = ps.length;
		    var size;
		    while (i--) {
		      finalHeight = sizer.offsetHeight;
		      ps[i].style.height = finalHeight + 'px';
		    }
		    event.preventDefault();
		  });
		}());
```
[example Less Layout trigger](https://jsbin.com/wukuzi/2/edit?html,css,js,output)
``` javascript after-optimization
		(function() {
		  // noprotect
		  var sizer = document.querySelector('.sizer');

		  document.querySelector('.set-size').addEventListener('click', function(event) {
		    var ps = document.querySelectorAll('.article .article-block');
		    var i = ps.length;
		    var size;
		    var finalHeight = sizer.offsetHeight;
		    while (i--) {
		      ps[i].style.height = finalHeight + 'px';
		    }
		    event.preventDefault();
		  });
		}());
```
仔細看就會發現其實只有一行code有變動，效果卻差很多！當你在操作類似DOM物件的時候可以參考上述的網站，避免不必要的re-layout動作。


## Composite

在一個網頁的頁面當中，實際上並非是平面的，通常會由許多`Layer`所組成，而在Browser rendering pipeline的最後一個步驟`Composite`中，就是負責將這些Layer組合成完整的頁面。

我們這次以最近Live直播統一獅頗為熱門的Yahoo首頁為例子，依照先前的方式打開Dev tool中的Timeline，選取其中一段frame後，點選下方的`Layers`標籤，應該就可以看到類似下面的圖：
![Composite Layers](/img/arvinh/layer.png)
![Composite Layers (而在Timeline的工具中，旁邊還能讓你拖拉旋轉，方便查看各個Layer的狀況。)](/img/arvinh/compositeLayers.png )

可以發現yahoo首頁也是由許多層Layer所組成。

眼尖的讀者可能會發現為什麼頁面中只有一個區塊是綠色的？這就是這小節的重點了，在Composite的過程中，只有需要重新Style與Layout的Layer才需要重繪，而這些需要重繪的Layer就會被標註成綠色；以yahoo首頁為例，綠色的那條是影音列表，理當會不斷變動，因此會被Highlight為綠色。

那為什麼我們需要針對這個步驟做Optimize呢？因為假如在同一層layer中，你其實只有某一個div需要re-layout，但由於是在同一個Layer，會變成整個Layer都需要重新繪製，如此一來就會增加不必要的負擔。

因此我們可以在你確定會需要重繪制的div中，加入`will-change: transform;`這個屬性，告訴browser說，“欸~我知道我可能會變動喔，請不要把大家跟我視為一樣的”，這樣就能解決上述的問題。


大家不妨打開自己的網站，利用Chrome的Timeline玩看看，提升頁面的渲染效率，讓我們一起追求60ps的極致快感吧！

## 延伸閱讀
1. [How not to trigger layout in webkit](http://gent.ilcore.com/2011/03/how-not-to-trigger-layout-in-webkit.html)
2. [Google Web Fundamentals](https://developers.google.com/web/fundamentals/performance/?hl=en)
3. [Accelerated Rendering in Chrome](http://www.html5rocks.com/zh/tutorials/speed/layers/)


關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化
