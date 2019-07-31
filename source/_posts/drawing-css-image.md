---
title: 用 CSS 畫畫的小技巧
date: 2017-11-17 19:55:02
author: arvinh
tags: 
    - css
    - drawing
---

## 前言
我想每個人小時候應該都蠻喜歡畫畫的吧？在沒有遊戲機、沒有 iPhone 的年代，拿著畫筆跟一張紙就可以開心度過幾個小時，但不知道從什麼時候開始就忘掉了這份快樂。

今天想邀請大家把『快樂』找回來！

身為工程師，我們拿起畫筆可能畫不出一個漂亮的圓，但透過熟悉的 CSS 我們可以做到。

『等等，平常上班調 CSS 就已經夠耗神了，你還要拿來畫圖？這除了炫技以外，對專業能力有什麼實質幫助嗎？你看連[知乎上也有人這樣想](https://www.zhihu.com/question/27474099)。』

那你滑 ig 對專業能力有什麼實質幫助嗎？

威～話不是這樣講，休閒活動是很重要的，以工作上來說，需要圖片時，當然還是出 SVG 圖檔比較有效率，效果也較好，但是透過 CSS 畫圖有幾個我個人認為蠻重要的好處：

1. **讓你對 CSS 的 Layout 佈局、style 屬性的操作更加熟悉**

2. **透過這種娛樂性質的練習，培養對 CSS 掌控性的自信，[找回對工作與技術的愛](https://blog.lalacube.com/mei/Reveal_love_remember.php)**
    * 工作總是會遇到需要支援舊版瀏覽器的狀況吧？很多新玩意兒你無法在工作中使用，那就透過這種機會好好把玩一下吧～

3. **訓練拆解頁面模組的能力**
    * 在 codepen 上臨摹他人著作時，可以想想如果是你，會怎麼組織你的元素，然後再跟作者比對看看誰的作法更棒！

4. **鍛鍊在有限資源下發展出無限創造力，就像李麥克一般。**
    * 這點是我覺得最有趣的，在 CSS 有限的能力下，你要想辦法找出方法來實作想要的效果，不知不覺中會鍛鍊你的思考與解決問題的能力！
    * 很多時候你會發現，看似複雜的東西，透過 CSS 的一些簡單屬性，其實非常容易。

5. **你可以透過 CSS 動畫輕易的繪製出動態圖片，這就不是你單純畫筆能做到的了！**

**今天就來介紹幾個簡單的小技巧，讓你能用 CSS 畫出如下的圖片！（什麼？你不知道 Rick and Morty？先去看再回來。）**

![Rick](/img/arvinh/rick_pure_css.png)

看得出來是用 CSS 畫的嗎？

[Rick - pure css](https://codepen.io/arvin0731/full/zPPaOK/)
原始碼在此，由於沒有處理 RWD，所以就不直接放過來啦～

接下來我會先從繪圖的工具開始介紹起，接著提供一些顏色搭配與繪圖靈感的資源，最後進入一連串 CSS 繪圖的技巧介紹。

## 繪圖小工具

### Pre-processor

CSS 畫圖的兩個主要工具就是 HTML 與 CSS，你當然可以直接用一般的 HTML 與 CSS 來製作你的 Pure CSS image，但我比較推薦使用 `Pug` 與 `SCSS` 兩個 pre-processor 來幫助你。

`Pug` 其實就是以前 Nodejs 的 Template engine `Jade` 改名後的版本，他能讓你利用下面的方式來架構你的 HTML：

```pug
div.sky
div.rick
    div.body
    div.head
```

等同於：

```html
<div class="sky">
    <div class="rick">
        <div class="body"></div>
        <div class="head"></div>
    </div>
</div>
```

看起來簡潔又省了許多重複的打字作業。

`SCSS` 相信就更不陌生了，透過 `SCSS`，你可以用更直覺的方式來撰寫 CSS 階層，並利用變數來讓你的 css 更有彈性：

```scss
$rick-body-color = #DDD8D0;
.rick {
    .body {
        background-color: $rick-body-color;
    }
    &:before {
        content: "Rick's body";
        position: absolute;
        width: 100px;
        height: 100px;
    }
}
```

等同於：

```css
.rick .body {
    background-color: #DDD8D0;
}

.rick .body::before {
    content: "Rick's body";
    position: absolute;
    width: 100px;
    height: 100px;
}
```

### 繪圖平台

上述兩套 pre-process 在 codepen、jsbin 等平台上都可以很方便的使用，我推薦大家到 Codepen 上面去練習並發布自己的作品，同時也看看上面的神人們（真的是超多神人...）是怎麼玩弄 CSS 的。

Codepen 上有許多 css 的活動，像是 `#Codevember`、`#DailyCSS` 等等，鼓勵大家每天到 Codepen 上展現自己的創意與能力，我其實也蠻想參加的，但是 Daily 真的太困難哈哈，如果也有人跟我一樣的感受，不如我們自己來弄個 `#WeeklyCSS` 吧！

### 顏色選擇工具

畢竟我們不是設計師，在顏色的敏銳度上可能欠缺鍛鍊，但沒關係啊，我們是為了休閒、為了把愛找回來。

可以透過 [coolors](https://coolors.co/) 這個網站幫你產生一些對應的顏色組合，或是利用 [Colorzilla](http://www.colorzilla.com/chrome/) 來從別人頁面取得你看上眼的顏色。

## 靈感來源

唯一推薦 [dribbble](https://dribbble.com/)，許多人都會從上面找尋靈感，然後轉成 pure css 的版本，甚至是加上動畫效果，讓圖片活起來。

也可以去 [behance](https://www.behance.net/) 以及剛剛提到的 [codepen](https://codepen.io/)，當然還有其他許多平台，但是我覺得光 Dribbble 上的圖就夠你玩了XD


## CSS 繪圖技巧

好的，終於要進入 CSS 繪圖的實戰技巧。
只要掌握住這邊介紹的小撇步，你就能繪製出如範例一般的 Pure CSS image！

### 善用各種形狀疊加組合

大家都知道 HTML 與 CSS 的基本形狀就是一個四四方方的 box，但是我們可以透過幾種方式「製造」出各種形狀：

* 最常見的三角形：
    如果你們曾經有需要製作 三角形的 div 的話，或許有看過 [css arrow please](http://www.cssarrowplease.com/) 這個網站，
    若仔細研究一下，你會發現它的原理很簡單，就是利用寬高皆為 0 的 div 上的 border 寬度來做變化，舉例來說：
    我們若設置一個 div 的寬高，以及四邊的邊寬，會呈現如下情況：
    <div style="height:20px;
    width:20px;
    border-color:red blue yellow green;
    border-style:solid;
    border-width:20px;"></div>
    ```css
        .rect {
            height:20px;
            width:20px;
            border-color:red blue yellow green;
            border-style:solid;
            border-width:20px;
        }
    ```
    接著我們把 寬、高 都設為零，不就看到三角形了嗎：
    <div style="height: 0px;
    width: 0px;
    border-color: red blue yellow green;
    border-style: solid;
    border-width: 20px;"></div>
    ```css
        .rect {
            height: 0px;
            width: 0px;
            border-color:red blue yellow green;
            border-style: solid;
            border-width: 20px;
        }
    ```
    所以只要留下一種顏色，就是我們要的結果了：
    <div style="height: 0px;
    width: 0px;
    border-color: red transparent transparent transparent;
    border-style: solid;
    border-width: 20px;"></div>
    ```css
        .rect {
            height: 0px;
            width: 0px;
            border-color:red transparent transparent transparent;
            border-style: solid;
            border-width: 20px;
        }
    ```
    [詳細介紹可以看這邊，還有教你如何支援到 IE6！](http://caibaojian.com/css-border-triangle.html)

* 神奇的 border 之不是只有一種角度：
    除了用來製作三角形以外，很多人常常忽略 border 其實可以根據各種方位來調整，像是 `border-top-left-radius` 或是 `border-left-width`，舉例來說，Rick 身上襯衫的衣領，我們可以這樣做：

    ```css
    .shirt {
        border-bottom-right-radius: 100%;
        border-right: 3px solid #666;
        top: 25px;
        left: 0;
        position: absolute;
        background-color: white;
        width: 10px;
        height: 40px;
    }
    ```

    <div style="border-bottom-right-radius: 100%;
    border-right: 3px solid #666;
    margin: 10px;
    background-color: white;
    width: 10px;
    height: 40px;"></div>

    透過 `border-bottom-right-radius` 就可以有一個小括弧的效果。
    

* 堆疊組合：
    以前面 `Rick` 的範例來看，他的頭髮部分就可以有很多種實作方式，你可能會想使用剛剛介紹的三角形，加上 `transform: rotate()`，組合出其雜亂有型的頭髮，但我太懶了，直接用 div 來堆疊組合：

    ![Rick hairs](/img/arvinh/rick-hairs.gif)

    堆疊組合有時候需要搭配 `overflow:hidden`，像是 Rick 的嘴巴，就是透過外層設定 overflow，並搭配適當角度的 border-radius，才能製作出這種弧度的效果：

    <img src="/img/arvinh/rick-mouth.png" style="width: 400px;height:400px;" />
    
* css clip-path:
    若是真的想要有多邊形，可以使用 clip-path。
    clip-path 是原本就存在於 SVG 中的一個屬性，讓我們能利用遮罩的方式來裁切出多邊形，在用 css 畫圖的時候，基本上就不要管瀏覽器支援度了吧，大膽用下去就對了！這邊提供幾個資源讓有興趣的人再去研究一下：
    [Clip path generator](https://bennettfeely.com/clippy/)
    [Clip path 詳細介紹](http://www.oxxostudio.tw/articles/201503/css-clip-path.html)
    [Can I use clip-path](https://caniuse.com/#search=clip-path)

### 偽元素 Before & after 是最佳幫手

偽元素在 CSS 繪圖當中佔據的角色真是太重要了，透過 `before` 與 `after` 的操作，你可以更有組織的來管理元素，以 Rick 的眼睛來說，我們可以先畫出一個圓形，在透過 `before` 和 `after` 兩個偽元素來產生 “眼皮” 與 “黑眼球”：

![Rick eyes](/img/arvinh/rick-eyes.gif)

```scss
.right-eye, .left-eye {
    position: relative;
    float: left;
    box-sizing: border-box;
    width: 80px;
    height: 80px;
    border-radius: 50%;
    border: 2px solid $rick--border;
    background-color: #fff;
    &:before {
        // 眼皮
    }
    &:after {
        // 眼球
    }
}
```

透過 **偽元素**，我們可以把相關的元素綁定在一起，也少了一些 div 的宣告，對於 Layout 上的調整來說也更加方便，因此一定要善加利用這個特性！

### box-shadow 與 background-image 給予的另種可能性

有了上述的知識，基本上你要湊出一個 Rick 或是 Morty 都是沒問題的，不過還有個小技巧想介紹給大家，就是 `box-shadow` 與 `background-image`。

首先是 `box-shadow`，是我覺得最酷的一個東西，利用 `box-shadow`，我們可以只用一個 div 就做出各種圖案，像是 [Calendar Icon](https://codepen.io/sashatran/full/BpWLbN)： <img src="/img/arvinh/calendar-icon.png" style="width:60px; height:60px;"/> 
或是棋盤：
<a class="jsbin-embed" href="http://jsbin.com/fibina/embed?html,css,output">JS Bin on jsbin.com</a><script src="http://static.jsbin.com/js/embed.min.js?4.1.0"></script>

運用 `box-shadow` 與 `background-image` 的原理都是利用到可以設定多個值的特性，以上面的棋盤為例子，4 x 4 的棋盤有 16 個方塊，扣除掉本身 div 佔據的一個，你需要設定 15 個 `box-shadow` 的值： `10px 0 0 #f00,` 代表在 X offset 10px, Y offset 0px 的位置有一個 #f00 的方塊（blur 與 spread 為 0），依此類推去組出棋盤。

懂得原理以後，慢慢就可以製作出更複雜的效果，像是 Rick 範例中的名字，每一個字母就是一個 div 去透過 `box-shadow` 組成的：

![single div using box-shadow](/img/arvinh/name-box-shadow.png)

`background-image` 在我這次的範例中沒有使用到，但基本原理差不多，可以看這邊的範例：

<p data-height="300" data-theme-id="29194" data-slug-hash="XKBwoZ" data-default-tab="css,result" data-user="yumeeeei" data-embed-version="2" data-pen-title="Pure CSS  Hotdog   🌭" class="codepen">See the Pen <a href="https://codepen.io/yumeeeei/pen/XKBwoZ/">Pure CSS  Hotdog   🌭</a> by yumeeeei (<a href="https://codepen.io/yumeeeei">@yumeeeei</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://production-assets.codepen.io/assets/embed/ei.js"></script>

### 總結

用 CSS 畫圖實際上是個蠻辛苦的事情，但就如我先前所說，可以訓練到很多思考能力以及創意發想，更重要的是，當你用一些很特別的技巧來達成你想要的效果時，成就感是非常大的啊～～

我也還在努力中，有興趣的話就跟我一起來 #WeeklyCSS 吧！

### 資料來源
1. [愛，可曾記得 - Paul Li](https://blog.lalacube.com/mei/Reveal_love_remember.php)
2. [How I started drawing CSS Images](https://blog.prototypr.io/how-i-started-drawing-css-images-3fd878675c89)
3. [Pure-CSS-Hotdog](https://dribbble.com/shots/3275208-Pure-CSS-Hotdog)
4. [Clip path generator](https://bennettfeely.com/clippy/)
5. [Clip path 詳細介紹](http://www.oxxostudio.tw/articles/201503/css-clip-path.html)
6. [Can I use clip-path](https://caniuse.com/#search=clip-path)
7. [Class: how-to-make-pure-css-images](https://coding-artist.teachable.com/p/how-to-make-pure-css-images)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化