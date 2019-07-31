---
title: 前後端分離與 SPA
date: 2017-09-16 10:13:35
tags:
  - frontend
  - backend
  - mvc
  - SPA
authot: huli
---

# 前言

這篇的原文（[你走你的陽關道，我走我的獨木橋：前後端分離](http://ithelp.ithome.com.tw/articles/10187675)）是我之前在 iT 邦幫忙鐵人賽的其中一篇文章，寫完之後有陸陸續續收到一些回饋，就想說可以重新整理一下這篇文章，讓它變得再更清楚一點。

如果你有以下疑惑的話，這篇文章非常適合你：

1. 為什麼前端會有 MVC？
2. 前端 MVC 跟後端 MVC 有什麼不一樣？
2. 為什麼要有 SPA（Single Page Application）？

（其實關於 MVC 到底是什麼就有很多討論了，但因為這篇文章的重點不在於此，所以對這方面就不多加描述，有興趣的朋友們可參考：[MVC是一個巨大誤會](http://blog.turn.tw/?p=1539)）

# 先從你熟悉的流程開始

如果你要寫一個簡易的部落格，你會怎麼做？

這答案很簡單嘛，可以先挑一個喜歡的框架，例如說 Rails、Laravel 等等，然後先定義好幾個 URL，再把 DB 的 Schema 想好，最後開始動手 coding。

例如說首頁的部分就是去 DB 把所有文章都撈回來，然後把資料丟到 view 裡面去 render，搞定！

總而言之，流程大概是這樣：

![](/img/huli/split/server.png)

當你想要訪問文章列表這個頁面的時候，瀏覽器會送 request 到 server，然後經過 controller 與 model，最後把資料帶給 view。

view 再回傳一份完整的 HTML 檔案（這個動作就叫做 render），而瀏覽器拿到之後，只要顯示出來就好。因為 render 在 server side，所以這也叫做 server side render。

這個流程照理來說，應該會是你最熟悉的流程，因為一大堆網頁都是這個樣子做的。

在這個狀況底下，一個只負責前端的工程師，基本上就是負責 view 這個資料夾底下的所有東西，必須用框架提供的 template 把資料跟 HTML 整合在一起。而當他需要 debug 的時候，必須要把整個專案都跑起來，才能看到畫面輸出的結果。

這樣的工作流程讓前後端切得沒有那麼開，畢竟前端工程師還需要會跑 rails，需要設定 DB，搞不好還要會設定 nginx！

現在的方法雖然把資料（Model）跟顯示（View）切開了，但都還是在後端，有沒有更好的方法呢？有沒有辦法，讓後端專注在提供資料，前端專注在顯示資料呢？

有！

# client side render

剛剛我們提到了 server side render，由後端直接回傳整份 HTML，瀏覽器直接顯示就好，因為 response 就是完整的網頁了。

但既然會特別區分 server 跟 client，就代表還有一種方式叫做 client side render，這又是什麼呢？

大家都知道，JavaScript 可以動態的產生內容，而 client side render 指的就是當前端拿到資料以後，才用 JavaScript 動態的把那些內容填到網頁上面。

直接拿程式碼出來說明，大家會比較好理解一點。

首先，我們的 server 現在就只專注在提供資料，所以就開一個 API 出來：

``` javascript
// 首頁，直接輸出所有留言
app.get('/', function (req, res) {
  
  // 從資料庫拿出所有的留言
  db.getPosts(function (err, posts) {
    if (err) {
      res.send(err);
    } else {
  
      // 直接把所有 posts 丟出去
      res.send({
        posts: posts
      });
    }
  })
});
```

如果用瀏覽器打開這個 API 的網址，應該會看到 JSON 格式的資料：

``` json
{
  "posts": [
    {
      "_id": "585f662a77467405888b3bbe",
      "author": "huli",
      "content": "2222",
      "createTime": "2016-12-25T06:24:42.990Z"
    },
    {
      "_id": "585f662777467405888b3bbd",
      "author": "huli",
      "content": "1111",
      "createTime": "2016-12-25T06:24:39.601Z"
    }
  ]
}
```

後端的部分已經準備就緒，順利地提供資料了，再來我們來看前端，只需要一個`index.html`就可以了。

``` html
<!DOCTYPE html>
<html>
<head>
  <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css" />
  <script src="https://code.jquery.com/jquery-1.12.4.min.js"></script>
  <script>
    $(document).ready(function() {
      getPosts();
    })
  
    // ajax 抓取文章
    function getPosts() {
      $.ajax({
        url: 'http://localhost:3000/',
        success: function(response) {
          if (!response.posts) {
            return alert('Error');
          }
          for(var i = 0; i < response.posts.length; i++) {
  
            // 丟給 render function
            addPost(response.posts[i]);
          }
        },
        error: function(err) {
          console.log(err);
          alert('抓取失敗');
        }
      })
    }
  
    function addPost(post) {
      var item = '' + 
        '<div class="panel panel-default">' +
          '<div class="panel-heading">' +
            '<h3 class="panel-title">' + post.author +', 發佈時間：' + post.createTime + '</h3>' +
          '</div>' +
          '<div class="panel-body">' +
            post.content
          '</div>' +
        '</div>';
      $('.posts').append(item);
    }
  </script>
</head>
<body>
  <div class="container">
    <a class="btn btn-primary" href="/posts">發表新留言</a>
    <h2>留言列表</h2>
    <div class="posts">
    </div>
  </div>
</body>
</html>
```

接著打開 `index.html`，就可以看到預期中的畫面出現了，跟我們之前用 server side render 產生的畫面應該一模一樣。用右鍵 -> 檢查去檢視，會發現所有元素都在。

![](/img/huli/split/ele1.png)

但是如果你改用右鍵 -> 檢視原始碼，會發現幾乎是空的：

![](/img/huli/split/ele2.png)

這個就是 client side render 跟 server side render 最大的差別。因為對於前者，我們是在執行期間「動態」去跟後端伺服器拿資料，再動態產生你看到的那些元素。

而那些元素原本不存在 index.html 裡面，是我們後來自己用 jQuery append 上去的，所以檢視原始碼當然不會出現任何東西。

我們來看一張 client side render 的示意圖：

![](/img/huli/split/client.png)

在 server side 的地方，view 這層直接被忽略了，因為後端只輸出 JSON 格式的資料而已，而這邊的第五步：把傳回來的資料 render 成 HTML 指的就是我們上面那個用 jQuery 動態 append 上去的步驟。

而在這種狀況底下，你有沒有發現前後端已經切開了？

後端工程師從此之後不用再管 view 裡面到底裝什麼，不用再教前端工程師怎麼跑 rails，他只要負責制定 API 文件，提供前端需要的資料就好。

而前端工程師也不需要跑那些服務了，只需要用他們最熟悉的瀏覽器打開 HTML 檔案就行了，利用 ajax 從後端拿資料，並且在自己這邊用 JavaScript 動態產生內容。

在這種情況下，前後端的部署也可以完全拆開，前端的部分最簡單，只需要隨便找一個可以放 html 檔案的地方就好，例如說 Amazon S3。所以前端幾乎不會有掛掉的問題，也不會有流量的問題，因為它只是個靜態檔案而已。

假如 server 有一天掛了，API 也跟著掛了，使用者依然可以造訪網頁，只是看不到資料而已，或者是你可以顯示出一個錯誤的圖案。但如果是舊的那種綁在一起的架構，server 一旦掛掉，你連畫面都渲染不出來。

再者，因為現在把 data 跟 view 完全切開來，你要替換任何一邊都是很方便的。例如說你後端不想用 rails 了你要用 go，完全沒問題！只要保持 API 的格式一樣，你後端就算要用 C 也沒人管你。

前端也是一樣，你要選 Angular，選 React 或是選 Vue 甚至你要手刻都可以，反正都不關後端工程師的事情。

# 可是，事情沒那麼簡單

雖然這種場景聽起來很美好，但千萬不要忽略了這樣子的改變會造成的後果。

什麼後果？那就是前端會變得有夠複雜。

仔細想想我們一開始提到的那種開發架構，統一從後端 render，所以我一個頁面就準備一個 view 的檔案，如果使用者訪問`/posts`，我就`render('posts.ejs')`；若是訪問`/about`，我就`render('about.ejs')`。

第一個問題來了：

> 既然剛剛說前端只有一個 index.html，那不就代表使用者訪問 `/posts`跟訪問`/about`，都是到同一個檔案？那我要怎麼渲染不同頁面？

因為在以往，路由的部分是 server 負責，就如同我上面說的一樣，server 依據不同的路由來決定要渲染哪一個頁面。可是現在切開之後，前端只剩下一個 index.html 了，那怎麼辦呢？

只好讓前端來負責了。

前端可以透過`window.location`或者是 [history API](https://developer.mozilla.org/en-US/docs/Web/API/History_API) 來管理網址，就可以知道使用者現在想要造訪哪一個頁面。

這邊有一個小細節稍微提一下，那就是剛剛講過前端只有一個 html 檔案，所以網址可能像是這樣：`https://example.com/index.html`。

只有一個網址而已，怎麼知道使用者要造訪哪一個頁面？

在以前我們要造訪`posts`的話，網址可能會是：`https://example.com/posts`，可是現在我們前端已經變成一個靜態檔案了，只有那一個路徑而已，該怎麼做呢？

第一種方法是利用 hash，例如說`https://example.com/index.html#posts`，前端再去解析後面的字串。

第二種是利用`nginx`或其他類似的服務，把所有`https://example.com/*`的網址都一律輸出 index.html 這個檔案，這樣子看起來就會跟以前一樣了。

我到`/posts`，server 會回傳 index.html，我到`/about` server 也會回傳一樣的內容。

但總之呢，因為後端不再處理這邊的路由了，所以這個部分完全轉交給前端負責，你必須在前端自己管理 URL 的狀態，去決定現在要顯示哪一個頁面。

那這邊要怎麼做呢？最簡單的方法就是跟以往後端做法一樣，你到哪一個網址，我就根據網址輸出怎樣的東西。

前端的程式碼大概會長這樣：

``` js
function render(path) {
  // 清空整個畫面
  $(body).empty();
  if (path === 'posts') {
    renderPostsPage();
  } else if (path === 'about') {
    renderAboutPage();
  }
}
```

只要每一次到一個新的網址，我就把現在內容全部清空再渲染一次就好。

是很簡單沒錯，但是效能有大大的問題，因為有些部分你其實根本不用清空。例如說網站最上方會有的導覽列跟最下面的 footer，基本上每一個頁面都有，是不會變的。

針對那些不變的部分，應該保留起來才對，不然每一次都清空重建一樣的東西很沒有效率。你可能會說這不就跟後端以前寫 view 一樣，抽出共通的部分然後放在 layout 裡面之類的。

不不，這不一樣。後端 render 本質上就是「每一個不同頁面就回傳一份不同的 html 檔案」，而我們現在前端 render 其實也有把共通的部分抽出來，但前端的難題在：「要怎麼只更新部分畫面，而不是暴力的每次都砍掉重練」。

你有沒有開始漸漸覺得前端越來越多事情要做了？

# Single Page Application

當你把這種東西發揮到極致，會覺得很像在寫一個 App 一樣，這東西就叫做 SPA，Single Page Application。

就如同字面上的意思一樣，我們現在只有一個 index.html 檔案，但是用起來卻像一個 App。

最經典的例子就是 Gmail。你在用 Gmail 的時候，完全沒有換頁。全部的動作都是在「同一個頁面」上面發生的，所以你載入的檔案從頭到尾就只有一個 index.html，完全沒有換過。

你在 Gmail 上面做的任何動作，都是用 ajax 發 request 給 server，server 回傳資料以後 client 端再用 JavaScript 把畫面 render 出來。

所以你在用 Gmail 的時候，會感覺好像在用一個 App 而不是在用網頁，因為頁面之間的跳轉很流暢，不像一般網頁中間可能會有白屏出現。

既然都叫做 Application 了，這時候的前端工程師已經不是大家以往想像的那樣，只需要會 HTML 跟 CSS 刻刻畫面，用 JavaScript 做點小特效跟互動。

要寫 SPA 的話，最難的就是狀態的管理。因為很多東西以前後端都幫你做掉了，所以你完全不用考慮這件事，但現在要了。

舉個例子好了，以前你訪問一篇文章，假設是`/post/12`，一點下去之後快速切回首頁再點擊其他文章，server 也就只是再回傳相對應的 HTML 而已。

可是呢，SPA 就不一樣了，考慮一下下面的流程：

1. 使用者點擊`/post/12`
2. query API
4. 使用者返回首頁
5. 使用者點擊`/posts/13`
6. query API
7. 拿到 response，渲染頁面

假設使用者點擊的速度很快，在第七步的時候，很有可能會先拿到第二步的 response，就會發生使用者明明點進去 A 文章，內容卻是 B 文章的狀況。

這只是一個簡單的例子，實戰上還有一大堆要考慮的問題，例如說還沒有拿到資料的時候要顯示什麼，拿到之後要怎麼更新等等。

# 前端 MVC

在前端變得愈來愈複雜之後，你應該也能理解為什麼前端需要 MVC 了。如果你有寫過純 PHP，經歷過那種商業邏輯跟 view 跟 model 混雜在同一個檔案的時期，應該也很能理解為什麼需要 MVC。

因為我們要把職責切開來嘛，讓大家各自負責該負責的東西，才不會全部混在一起變成義大利麵。

前端 MVC 其實跟後端 MVC 滿類似，然後前端也要設定路由，剛剛有提過了。就是設定哪一個 URL 要去哪一個 Controller，再去相對應的 Model 拿資料，最後輸出 View。

這邊稍微比較一下前後端的 MVC 在做的事情：

&nbsp; | 前端 | 後端
----- | ---- | ---
Model | 去跟後端 API 拿資料 | 去跟 DB 拿資料
View  | 在前端動態產生畫面 | 無
Controller | 呼叫相對應的 Model 並 render 畫面 | 呼叫相對應的 Model 並傳回資料

![](/img/huli/split/mvc.png)

你會發現其實前後端做的事情都差不多，只是前端注重在 render 畫面，後端注重在輸出資料。還可以畫出這一張完整的流程圖：

![](/img/huli/split/all.png)

用文字來解釋的話，流程是這樣的：

1. 使用者造訪 /posts 這個網址，代表他想看全部文章
2. 前端的路由去處理，負責呼叫對應到的 controller
3. 前端 controller 去呼叫 Model 拿資料
4. 前端 Model 透過 API，去 /api/posts 這個網址拿資料
5. 後端路由接到 request，丟給對應到的後端 controller
6. 後端 controller 跟後端 Model 拿資料
7. 後端 controller 把資料傳回去
8. 前端 Modle 拿到資料以後回傳給前端 controller，並且把資料丟給 view
9. client side render，把畫面渲染出來

你上面看到的這整套，大概就是最基本的 SPA 的架構了。後端只負責輸出資料，前端來負責抓資料跟渲染畫面。把前後端完完全全的切開了，就算你後端壞掉，你前端還是看得到畫面（只是可能會顯示個錯誤畫面之類的）；你前端壞掉，後端還是能安穩的輸出資料供其他服務使用。

兩邊乾乾淨淨，而且任何一邊都會比較好維護，前端工程師想要改任何跟介面有關的東西，都跟後端完全沒有關係，其實就等於是兩個不同的專案的意思。

# 我們真的需要 SPA 嗎？

上面提到了這麼多，我認為前端開發的複雜化跟 SPA 有滿大的關係，畢竟你現在就等於是在開發一個完整的 App，怎麼可能不複雜？

可是別忘記問自己：

> 我們真的需要 SPA 嗎？

有些場景是一定要用的，例如說音樂播放網站。

為什麼？因為你必須一邊播放音樂，一邊讓他可以在網站上看其他的資料，例如說歌手介紹、專輯介紹之類的。如果你今天不是用 SPA，那使用者點到別的頁面的時候，瀏覽器就跳頁了，音樂就停了。哇靠這個體驗也太差了，完全不能接受。

所以這種網站一定要用 SPA，沒有其他選擇。用了之後因為不會跳頁，所以你點擊歌手介紹的時候，只是發一個 ajax request，然後收到 response 之後用 JavaScript 把接收到的資料 render 成 HTML 顯示出來。不管到哪一個頁面，都不會真的跳轉，不會載入新的 HTML 檔案。

那有些地方我覺得不用也行，但用了之後可以加強使用者體驗，例如說 Twitch 前一陣子的新功能，在你跳去看其他頁面的時候，你原本在看的實況會縮到左下角。

![](/img/huli/split/twitch.png)

總之，會需要用到 Single Page Application 的場合有兩個，一個是因為必須要這樣做，另一個是因為可以增進使用者體驗，讓使用者覺得操作起來更順暢。

如果你發現你要做的東西不符合這兩種場合，你可以選擇不要做 SPA，可以選擇就依照之前那樣的 MVC 架構，由 server side 去 render、去處理。這一切都是可以選擇的。

除此之外，其實 SPA 也有一些缺點，例如說你明明只要看一個頁面而已，卻要把一大包的 JavaScript 或是其他頁面的 template 一起下載下來。

又或者因為是 client side render，所以有些搜尋引擎爬不到任何資料（因為你的 index.html 幾乎是空的），不過 Google 很厲害，會爬 JavaScript 執行完之後的結果。可是這對 SEO 來說還是不太好。

當然上面都有一些方法可以解決啦，例如說可以把 js 檔案分開，你到那一個頁面就只要下載那一個頁面的 js 即可。SEO 的解決方法則是將兩種結合，第一次先在 server side render，之後的操作都改用 client side render，就可以保證搜尋引擎也能爬到完整的 HTML。

但你知道的，可以解決是一回事，要花多少心力去解決又是另外一回事了。

# 總結

這篇大概講到了一開始最常見的網站架構，然後到近期導致前端開發複雜化的 SPA。我以前剛接觸的時候也是一頭霧水，想說到底前端為什麼需要 MVC。可是經過這一連串的脈絡思考下來，就很能理解原因了。

當你東西變得愈來愈複雜，就需要一個架構去把職責切割開來，不然會造成日後維護上的困難。

當你越瞭解 SPA 所帶來的優缺點，你在選擇要不要用的時候就有更多面向可以參考，就有更多的理由去支持你所做的決定，而不單單僅是「哇！好潮喔！別人用了我也要用！」

希望這篇對大家有幫助，最後附上一篇延伸閱讀：[Why I hate your Single Page App](https://medium.freecodecamp.org/why-i-hate-your-single-page-app-f08bb4ff9134)

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好