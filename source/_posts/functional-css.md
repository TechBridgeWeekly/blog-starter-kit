---
title: 邪魔歪道還是苦口良藥？Functional CSS 經驗分享
date: 2019-01-26 08:49:34
tags:
  - functional CSS
author: huli
---

# 前言

以 CSS 架構方法來說，主流的大概可以分成三個：OOCSS、SMACSS、BEM，這些架構的提出都是為了讓 CSS 更好維護，這幾個的介紹以及差異可以參考 @arvinh 寫過的 [淺談 CSS 方法論與 Atomic CSS](https://blog.techbridge.cc/2017/04/29/css-methodology-atomiccss/)。

但我們今天要談的不是上面提到的這三種，而是另外一種相較之下沒那麼主流（但好像有慢慢流行的趨勢），而且很少人第一眼看到就會贊同的方法：functional CSS。

# 什麼是 functional CSS

直接舉一個例子最快：

``` html
// 一般的寫法
<div class="hello">Hello</div>
  
.hello {
  font-weight: 700;
  color: red;
  padding: 1rem;
}
  
// Functional CSS
<div class="fw7 red pa3">Hello</div>
  
.fw7 {
  font-weight: 700;
}
.red {
  color: red;
}
.pa3 {
  padding: 1rem;
}
```

就好像 functional programming 那樣，每個函式都沒有副作用而且可以互相組合，在 functional CSS 裡面每一個 class name 都只負責一個部分（不一定是一個屬性），就像我上面舉的那個範例，它會產生一個字是粗體、紅色並且有 padding 的 div。

順帶一提，如果你有用過 [Bootstrap4](https://getbootstrap.com/docs/4.2/getting-started/introduction/) 的話，你很有可能已經體驗過 functional CSS 了，裡面就出現了一大堆這種類型的 class name 。

你喜歡這種風格的寫法嗎？

如果你是第一次看到這種寫法，我覺得你心裡想的應該是：「這是三小，好噁心」、「這不就是 inline style 嗎」、「這根本不是 CSS 吧！」

沒關係，我第一眼看到也是這樣覺得的，但會寫這篇文章就是因為我後來改觀了，於是接下來我要談談我與 functional CSS 從互相厭惡到和解的過程。

# 我與 functional CSS 的愛恨糾葛

一開始看到 functional CSS 覺得很特別但也很奇怪，老實說連想試的感覺都沒有，只覺得這樣寫 CSS 也太奇怪了吧，根本就是邪魔歪道！而且 class name 一點可讀性都沒有。

但有天我在 hacker news 上面讀到了這篇文章：[In defense of Functional CSS](https://www.mikecr.it/ramblings/functional-css/)，徹底改變了我對它的想法。

這篇文章針對幾個常見的批評給予了反駁，我下面舉幾個文章裡面提的例子：

## 跟 inline style 差在哪？

1. Inline style 不能有 media query
2. inline style 的屬性可以隨意設置（這之後我會講詳細一點）
3. Inline style 沒辦法處理 `:before`, `:after`
4. Inline style 無法重用，但是 css class 可以，我可以定義一個叫做 .bg-red 的規則，想要背景是紅色的加上去就行了
5. Inline style 跟 functional css 的可讀性還是有差，比比看 `class="f-sm bg-blue"` 跟 `style="font-size: 10px; background-color: #0000ff;"`

我覺得作者的幾個反駁都滿合理的，inline style 跟 functional CSS 的確是有差，我想讀到這裡大家應該可以認同如果這兩個一定要選一個，選後者是合理許多的，因為可重用而且可讀性較高。

但大多數人反對 functional CSS 的主要理由還有一個，那就是會把 html 弄的很髒而且不知道在幹嘛。

例如說原文中提到的範例：

``` html
<div class="profile-card">
  ...
</div>
<style>
  .profile-card {
    padding: 20px;
    margin: 20px;
    color: #eee;
    background: #333;
    border: 1px solid #555;
  }
</style>
<div class="m-5 p-5 text-gray-light bg-gray-darker border border-gray-light">
  ...
</div>
```

前者你一看就知道是一個 profile card，但後者你光看 html 根本看不出來他是什麼。

在這邊作者給的解釋我覺得也很不錯：

> 你可以一起用啊

對欸，你可以變成這樣：

``` html
<div class="profile-card m-5 p-5 text-gray-light bg-gray-darker border border-gray-light">
  ...
</div>
```

這樣你可以維持原有的 class name 命名方式，而這個命名只是為了方便讓你辨識出這個元素是什麼，實際上在做 styling 的還是後面的那些 functional class name。

如果還想再反駁，大概就是 html 看起來還是很髒而且一大堆 class name。我覺得這是優點也是缺點，端看你怎麼去看它。

若是你完全不知道那些 class name 是什麼意思，你當然會覺得那是一堆垃圾；但如果你知道是什麼意思，你會發現光是看 HTML，你就可以知道樣式長怎樣，你不必在 HTML 與 CSS 之間切換，而是只要專注在 HTML 就好，因為它的樣式都寫在 class name 裡面了。

舉例來說，你原本的開發流程可能是這樣：

1. 建立一個 profile-card 的html
2. 加上 .profile-card class name
3. 開一個 profile-card.css 開始寫樣式
4. 新增 profile-card-avatar 的 html
5. 加上 .profile-card-avatar 的 html
6. 開始幫這個 class name 寫樣式

但採用 functional CSS 之後，開發流程變這樣：

1. 建立一個 profile-card 的html
2. 幫 profile-card 加上 class name
3. 新增 profile-card-avatar 的html
4. 幫 profile-card-avatar 加上 class name

不用再在 HTML 與 CSS 間切換，因為沒有 CSS 檔案讓你切換。

## 可是重用性太低了吧，我要幫每個 button 都寫 20 個 class？

這個批評基本上是說，假設我有一個 button 用 functional css 之後長這樣：
`<div class="bg-blue fw5 pa1">Click me</div>`

那如果我有其他地方要用這個 button，我不就要複製這一串？如果我 button 的樣式換了，那我不就所有地方都要改？這個重用性也太差了吧。

原文中給的反駁是如果真的有這種情況發生，你應該優先考慮的是把這段 HTML 抽成可重用的 template 之類的，而不是把這個問題怪到 class 身上。

或者我這樣說好了，應該把這個東西變成一個 component，這樣問題就迎刃而解了，因為你只要改 component 就好，不用每個地方都改。

以上就是這篇文章大概的內容，有興趣的可以去看原文，原文寫的比我摘要得清楚滿多的而且講了更多。但總之我看完這篇文章之後有了些想法，並且開始認識到 functional CSS 的好處到底在哪。

# Functional CSS 到底好在哪？

第一個好處就是你從此（幾乎）不必再寫 CSS！而且你也不用再猶豫 class name 到底要取什麼了！

這簡直拯救了一堆有命名恐懼症的開發者。用了 functional CSS 以後只要幫 html 加上相對應的 class 就好，就如同我上面舉的範例那樣。

這時候你可能會說：「那我要怎麼知道這段 HTML 到底是幹嘛的？」

第一種解法上面有提過，就是把原本的 class 也加回去，所以有意義的 class name 當作辨識用途，functional css 當作樣式來使用，但這個方法我個人覺得有點多此一舉就是了，而且還要再花時間想要命名什麼 class。

第二種解法是 component，我後來意識到有些 functional CSS 會碰到的問題可以靠 component 來解，這個 component 可以是 web-component 也可以是 React 或是 Vue 裡面的那種 component。

當我們有了 component 以後，就沒有那麼需要 class name 了，因為你看 component 的名稱就知道它是個按鈕，從元件的命名就可以知道，不必再從 class name。況且就算原本有 class name，你還是要先去對照畫面才能確定你到底要改哪裡，畢竟有些 class name 命名的超級模糊，這我相信大家一定都有體會過。

原本在寫 CSS 時你需要考慮的很多東西，到了 functional CSS 幾乎都不存在，你要做的只有幫 HTML 加上 functional CSS 的 class name 幫它裝飾而已。

第二個好處是你一但採用 functional CSS，就可以立刻幫你的 project 產生出一套規範，有點像是 design guideline 那樣。

這是什麼意思呢？

首先，大家可能會對 functional CSS 有個錯誤認知，那就是覺得它說穿了還是另外一種形式的 inline style，只是多此一舉寫成 class 而已。

不是的，它並不是你想用什麼就用什麼，而是先把規範訂好，你再從可以用的 class 裡面去挑你要用的出來。例如你們產品的網站的背景有兩種主色分別是紅色與藍色，於是你寫了 .bg-red 跟 .bg-blue 這兩個 class。

今天有一個新人來你們公司，他想用紅色就會直接用 bg-red 而不是自己再寫一個 class。如果他真的再寫一個，那 code review 時也可以輕易抓到，因為用了 functional css 的專案通常 CSS 檔案寫好後就不會再變動了，所以有更動的時候特別明顯。

如果今天是按照以前寫 CSS 的方式，有可能他在 CSS 裡面就偷懶直接寫色碼而不是用你在 color.scss 定義好的變數，或也有可能他在 color.scss 裡面眼殘沒看到 bg-red，所以自己加個 bg_red 的 class。 

是的，這一樣在 code review 的時候可以抓到，但我想表達的是前者耗費的心力會比後者少，因為要檢查的地方比較少。

一但 functional CSS 的主要 style.css 完成後，這份檔案同時也代表網站的規範，可以使用的顏色、padding、margin、字體、字體大小都在裡面了，想用的時候只能從這裡面找，不能自己隨意新增，所以你可以很輕鬆就規定網站的 padding 只能是 4、8 或是 16，或是行距只能有 1、1.25 跟 1.5。

其實以前用 SCSS 或任何 CSS 預處理器時也可以這樣做，把所有規範都訂成變數，並且規定所有規則都只能用這些變數。但我認為 functional CSS 天生就蘊含著規範在裡面。

第三個好處是檔案大小驟減，因為 padding: 4px 只會在 CSS 檔案裡面出現唯一一次，`color: red` 也只會出現唯一一次，以 functional CSS 的 framework [Tachyons](https://tachyons.io/) 來說，minified 跟 gzipped 過後的 CSS 大小是 14kb。

現在是 14kb，以後也會是 14kb，因為你所有需要的規則都在裡面了，你的 CSS 大小不會再隨著網站的複雜度增加，這也是很棒的一點。

另外一個 functional CSS framework [Tailwind](https://tailwindcss.com/) 的作者寫了一篇很棒的文章來探討一些優缺點，並且有脈絡地帶你看 functional CSS 的優勢在哪，我自認絕對不可能寫的比那篇好，所以有興趣再深入理解的可以參考：[CSS Utility Classes and "Separation of Concerns"](https://adamwathan.me/css-utility-classes-and-separation-of-concerns/)。

總之呢，在爬了一大堆文章以及跟同事討論過以後，我們決定把公司的產品換成 functional CSS，會想換的原因有兩個：

1. CSS 越來越多以後很難維護，只要不小心一個偷懶就成了未來的技術債
2. CSS 檔案越來越大，但其實可以小很多


# Function CSS 實戰經驗分享

之前看過一篇 [Full re-write in 10 days with tachyons and functional CSS: A case study](https://hackernoon.com/full-re-write-with-tachyons-and-functional-css-a-case-study-part-1-635ccb5fb00b)，作者講述他如何輕鬆寫意的在十天裡面把整個網站改寫完成。

而我們那時除了要重構這些 CSS 以外還要修 bug 跟開發新的功能，所以前前後後大概一個月才把整個網站換完，而且實際下去重構才發現以前寫的有些 CSS 真的是超難維護，因此在這部分也多花了點時間。

上面有提到幾個相關的 CSS framework，但我認為 functional CSS 的概念簡單好懂，自己從頭實作一個反而比較符合自己的需求，於是就參考 Tachyons 的 class name 來實作。

第一步大概是先把一些常用的 class 定出來，例如說顏色：

``` css
.c-red { color: $color-red; }
.c-yellow { color: $color-yellow; }
.c-white { color: white; }
.c-green { color: $color-green; }
.c-grey-83 { color: $color-grey-83; }
.c-grey-4a { color: $color-grey-4a; }
.c-grey-bb { color: $color-grey-bb; }
.c-grey-f8 { color: $color-grey-f8; }
```

還有必備的 flex 排版：

```css
.flex { display: flex; }
.inline-flex { display: inline-flex; }
.flex-auto { flex: 1 1 auto; }
.flex-column  { flex-direction: column; }
.flex-row     { flex-direction: row; }
.flex-wrap    { flex-wrap: wrap; }
.flex-nowrap    { flex-wrap: nowrap; }
.items-start    { align-items: flex-start; }
.items-end      { align-items: flex-end; }
.items-center   { align-items: center; }
.items-baseline { align-items: baseline; }
.items-stretch  { align-items: stretch; }
.justify-start   { justify-content: flex-start; }
.justify-end     { justify-content: flex-end; }
.justify-center  { justify-content: center; }
.justify-between { justify-content: space-between; }
.justify-around  { justify-content: space-around; }
```

除此之外，也可以自己寫一些 utility class：

``` css
.ellipsis {
  overflow: hidden;
  text-overflow: ellipsis;
}
  
.limit-line {
  overflow: hidden;
  text-overflow: ellipsis;
  display: block;
  display: -webkit-box;
  -webkit-line-clamp: 1;
  -webkit-box-orient: vertical;
}
  
.pointer:hover { cursor: pointer; }
```

這就呼應我前面提到的，一個 class name 其實可以有一個以上的規則，只要你能從 class 的名稱清楚知道它在做什麼就好。

重構時的流程其實很固定，基本上就是這幾步：

1. 選定要重構的 component
2. 先從最裡層開始，右鍵檢查，確定這個 class name 沒有其他副作用
3. 把原本的 style 換成 functional CSS
4. 把原本的 class name 移除

在此過程中可以順便把網站的樣式做個規範，例如說原本 padding 是 5 的地方統一變成 4 等等，網站就會變得越來越規範。

不過重構時當然也碰到一些困難，那就是有些以前寫的 CSS 為了圖方便沒有考慮到維護性的問題，到頭來這個坑還是落到自己身上。舉例來說，有個元件叫做 Card，需求是在首頁以及在餐廳頁面它的 padding 不一樣，所以以前就這樣寫：

``` scss
// home_page.scss
.home-page {
  .card {
     padding: 10px
    }
}
  
// restaurant_page.scss
.restaurant-page {
  .card {
    padding: 15px;
  }
}
  
// card.scss
.card {
  padding: 20px;
}
```

問題是什麼？問題是如果你只看 .card 的 CSS，你根本不會發現它在不同的頁面下會有不同的 padding！如果只是 padding 的話問題還小，但如果依照這個邏輯繼續往下寫，有可能連顏色跟 margin 都變了，像是：

``` scss
.home-page {
  .card {
     padding: 10px
    &__title {
      margin-top: 20px;
      background: red;
     }
   }
}
```

這種做法是把顯示的邏輯放在 CSS 裡面，利用 CSS 去操控，所以 JS 裡面不用額外寫任何東西，Card 這個 component 在不同地方就會有不同的樣式。

但我後來覺得這樣不是種好作法，應該要把邏輯移回 JS 裡面比較好，所以改成這樣寫：

``` js
// home page
<Card type="home" />
  
// restaurant page
<Card type="restaurant" />
  
// Card component
function Card({ type }) => (
  <div className={cx({
    'padding-20': !type,
    'padding-10': type === 'home',
    'padding-15': type === 'restaurant'
  })} />
)
```

在不同地方我用 component 的 props 來區分，並且把這段邏輯放在 component 裡面，比起 CSS 的做法當然有好有壞，但至少可以保證當我 render 單純的 `<Card />` 的時候，在任何一個頁面它的樣式都會是一致的，不用擔心在不同地方會突然出現不同的樣式。

在重構的過程中其實發現很多這種問題，如果沒有趁早除掉的話 CSS 只會越來越多而且越來越亂，到最後會變得超級難維護，會很容易發生改一個 class 壞兩個地方，牽一髮動全身的現象。因此剛好趁著改寫成 functional CSS 的時候來處理這些問題。

許多人對 functional CSS 還有一個誤解，那就是不能寫「其他的」 CSS。舉例來說，我前面提到 functinoal CSS 自成一個規範，沒有寫成 class 的東西你不能用，但其實有些特殊情況還是可以的。

例如說你今天有一個 div 的高是 333px，難道你就要寫一個 `.height-333` 的 class 嗎？如果真是這樣的話那真的跟 inline style 沒兩樣了。

但 functional CSS 考量的點應該是「能否重用」，能夠重複使用的才把它寫成 class name，像是高 333px 這種我就會直接用 styled-component 或甚至直接寫 inline style，不會特地給它一個 `.height-333` 的 class，因為整個 App 可能就只有它用得到。

最後讓我們來看一下改寫的成果，這是改寫前，CSS 大約 400kb（gzipped 前）：

![](/img/huli/functional_css/css-before.png)

這是改寫後，可以看到各項數據都下降很多，CSS 大約 130kb，其實還可以再小，會比較大是因為裡面有一些轉成 base64 的小圖：

![](/img/huli/functional_css/css-after.png)

改寫之後減少了將近 70% 的 CSS 體積。

而且重點是無論以後 App 大了十倍還是一百倍，CSS 都能夠維持在差不多的大小，因為常用的屬性都被我們變成 class name 了。

改寫的難易度取決於你原本 CSS 的質量，像我們很多 CSS 為了求快沒有多思考耦合就很高，常常要參考兩三個 CSS 檔案才能拼湊出最後 style 的長相。但如果原本就有處理好這個問題，速度應該能夠快滿多的。但整體來說改寫還是算是容易，而且每次改寫完成就感很大，可以直接把一大堆 CSS 規則刪掉，滿有快感的。

有興趣的朋友可以用這個網站對自己的產品做測試：[https://cssstats.com/](https://cssstats.com/)。

# 總結

如果要說 functional CSS 有什麼缺點的話，我目前想到的就是剛開始學習需要一段時間以及 style 多的話 HTML 會變得充滿一堆 class name，比較難閱讀而且檔案也較大，但相較之下我依然認為優點是多過缺點的。

優點前面也大概都說過，基本上就是不用擔心 CSS 的耦合性問題，絕對不會發生改一個 class name 壞兩個 component 的情況，因為每個 class name 都不會互相干擾，也可以保證你把這個 component 搬到任何地方都還是長得一樣，背後不會有特別的 CSS 在那邊搞你。

也不用再煩惱 class name 要怎麼命名了因為你不需要，這個就可以節省滿多時間的。也不用再手寫 CSS 了，所以開發速度也變快了，因為不用再在 CSS 檔案與 component 之間切換，你邊寫 HTML 的時候就可以順手把 style 寫完，存檔以後看個畫面再調整一下就行了，跟以前相比步驟少了很多。

其實對 CSS 我的經驗不是那麼多，可能有很多 case 沒有考慮到或是優缺點沒有講得很清楚，如果對 functional CSS 想要更深入研究的，我文末附的資料都很有參考價值，大家可以看看。

但總之，我現在是 functional CSS 的支持者之一了。

參考資料：

1. [In defense of Functional CSS](https://www.mikecr.it/ramblings/functional-css/)
2. [Tachyons](https://tachyons.io/docs/table-of-properties/)
3. [Full re-write in 10 days with tachyons and functional CSS: A case study ](https://hackernoon.com/full-re-write-with-tachyons-and-functional-css-a-case-study-part-1-635ccb5fb00b)
4. [Tailwind: style your site without writing any CSS!](https://jvns.ca/blog/2018/11/01/tailwind--write-css-without-the-css/)
5. [CSS Utility Classes and "Separation of Concerns"](https://adamwathan.me/css-utility-classes-and-separation-of-concerns/)
6. [HN 上面的討論](https://news.ycombinator.com/item?id=18084013)


關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好
