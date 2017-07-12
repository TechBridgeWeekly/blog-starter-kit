---
title: Sass/SCSS 簡明入門教學
date: 2017-06-30 09:54:49
tags: LESS, SASS, SCSS, Sass, PossCSS, CSS, OOCSS, CSS in JS, CSS Modules
author: kdchang
---

![Sass/SCSS 簡明入門教學](/img/kdchang/sass.png)

# Sass/SCSS 簡介
>Sass is an extension of CSS3, adding nested rules, variables, mixins, selector inheritance, and more. It’s translated to well-formatted, standard CSS using the command line tool or a web-framework plugin.

[Sass（Syntactically Awesome StyleSheets）](http://sass-lang.com/)是一種 CSS 的擴充，是為 CSS 的超集合（透過編譯會 compiled 成傳統 CSS，讓瀏覽器可以閱讀）。使用 Sass 的出現是為了解決在大型專案時傳統 CSS 會遇到的重複、可維護性差等問題（新增了 nested rules, variables, mixins, selector inheritance 等特性）。讓開發者可以撰寫簡潔、富語意（expressive）、重複性佳（reusable）、可維護性佳和可延展性佳的 CSS 程式碼。
gem install sass

Sass 的語法分為新的 SCSS (`Sassy CSS`、`Sass 3`，檔案名稱為 `*.scss`) 和舊的 SASS（向 `Haml`取經，具備不使用大括弧格式、使用縮排，不能直接使用 CSS 語法、學習曲線較高等特性，檔案名稱為 `*.sass`）。由於新的 SCSS 語法是 CSS3 的超集合，所以傳統的 CSS3 檔案就算直接複製過來也不會出錯，學習曲線相對較緩，因此我們將使用 SCSS 語法。 

關於兩者比較的補充可以參考這篇文章 [What's the difference between SCSS and Sass?](http://stackoverflow.com/questions/5654447/whats-the-difference-between-scss-and-sass)

## SASS 初體驗
在開始介紹 SASS 特性之前，我們先來學習如何將 Sass 轉譯成 CSS。

首先，先按照官網先行[安裝 Sass](http://sass-lang.com/install)，接著在專案資料夾建立一個 `main.scss` 檔案，檔案內容如下：

```scss
// 引用
@import url(https://fonts.googleapis.com/css?family=Pacifico);
//Add variables here:

h1 {
  font-family: Roboto, sans-serif;
  text-align: center;
}

.banner {
  font-family: 'Pacifico', cursive;
  height: 400px;
  background-image: url("lemonade.jpg");
}

.container {
  text-align: center;
  font-family: 'Pacifico', cursive;
}
```

在終端機下以下指令進行轉譯：

```
// sass 指令 欲轉譯檔案 輸出的檔案
sass main.scss main.css
```

此時你就會看到資料夾中多了 `main.css` 和 `main.css.map` 兩個檔案，前者是轉譯過後的 CSS 檔案，後者是方便使用瀏覽器除錯工具除錯時連結原檔案和轉譯檔案，方便除錯。

1. Variables：變數可以用來儲存值，方便重複利用

	有些時候 Variables，這時候若能使用變數儲存重複使用的值可以增加重用性。在 Sass 中我們使用 `$` 來表示變數，變數的資料型態可以是 Numbers（可以有單位或無單位）、Strings、Booleans、null 值（視為空值），甚至可以使用 Lists、Maps 來。

	變數的使用：

	```scss
	$translucent-white: rgba(255,255,255,0.3);
	p {
		background-color: $translucent-white;
	}
	```

	Lists 可以空格或加逗號分隔屬性值：

	```
	$font-style-2: Helvetica, Arial, sans-serif;
	$standard-border: 4px solid black;

	p {
		border: $standard-border;
	}

	// maps key:value
	$font-style-2: (key1: value1, key2: value2);	
	```

2. Nesting：降低父元素重複性

	轉譯前：

	```scss
	.parent {
	  color: blue;
	  .child {
	    font-size: 12px;
	  }
	}
	```

	轉譯後：

	```css
	.parent {
	  color: blue;
	}

	.parent .child {
	    font-size: 12px;
	}
	```

	在 Nesting 中不僅只有 child selectors 可以使用，還可以使用在相同的 Properties 上：

	```scss
	.parent {
	  font : {
	    family: Roboto, sans-serif;
	    size: 12px;
	    decoration: none;
	  }
	}
	```

	轉譯後：

	```css
	.parent {
	  font-family: Roboto, sans-serif;
	  font-size: 12px;
	  font-decoration: none;
	}
	```
3. Mixins：降低 pseudo 元素撰寫的重複性，如：`::before`、`::after`、`:hover`，在 Sass 中使用 `&` 代表父元素

	轉譯前：

	```scss
	.notecard{ 
	  &:hover{
	      @include transform (rotatey(-180deg));  
	    }
	  }
	```

	轉譯後：

	```scss
	.notecard:hover {
	  transform: rotatey(-180deg);
	}
	```

	重用群組的 CSS，例如跨瀏覽器的 prefix，使用 @include 加入群組：

	轉譯前：

	```scss
	@mixin backface-visibility {
	  backface-visibility: hidden;
	  -webkit-backface-visibility: hidden;
	  -moz-backface-visibility: hidden;
	  -ms-backface-visibility: hidden;
	  -o-backface-visibility: hidden;
	}
	.notecard {
	  .front, .back {
	    width: 100%;
	    height: 100%;
	    position: absolute;

	    @include backface_visibility;
	  }
	}
	```

	轉譯後：

	```css
	.notecard .front, .notecard .back {
	  width: 100%;
	  height: 100%;
	  position: absolute;

	  backface-visibility: hidden;
	  -webkit-backface-visibility: hidden; 
	  -moz-backface-visibility: hidden;
	  -ms-backface-visibility: hidden;
	  -o-backface-visibility: hidden;
	}
	```

	`@mixin` 也可以透過 `@include` 使用參數，也可以使用預設值：

	```scss
	@mixin backface-visibility($visibility:hidden) { //Add an argument
	  backface-visibility: $visibility;
	  -webkit-backface-visibility: $visibility;
	  -moz-backface-visibility: $visibility;
	  -ms-backface-visibility: $visibility;
	  -o-backface-visibility: $visibility;
	}

	.front, .back {
	    @include backface-visibility(hidden);
	}
	```

	有時我們也需要處理參數複雜的情形，此時可以使用 Lists、Maps 資料型態當輔助：

	```scss
	@mixin stripes($direction, $width-percent, $stripe-color, $stripe-background: #FFF) {
	  background: repeating-linear-gradient(
	    $direction,
	    $stripe-background,
	    $stripe-background ($width-percent - 1),
	    $stripe-color 1%,
	    $stripe-background $width-percent
	  );
	}
	```

	使用 Maps 資料格式存欲傳入變數：

	```scss
	$college-ruled-style: ( 
	    direction: to bottom,
	    width-percent: 15%,
	    stripe-color: blue,
	    stripe-background: white
	);
	```

	變數使用 `...` 進行傳遞：

	```scss
	.definition {
	      width: 100%;
	      height: 100%;
	      @include stripes($college-ruled-style...);
	 }
	```

	還有種情況是參數傳入當作字串（String interpolation）：

	轉譯前：

	```scss
	// 使用 #{$file} 接收
	@mixin photo-content($file) {
	  content: url(#{$file}.jpg); //string interpolation
	  object-fit: cover;
	}

	.photo { 
	  @include photo-content('titanosaur');
	  width: 60%;
	  margin: 0px auto; 
	}
	```

	轉譯後：

	```css
	.photo { 
	  content: url(titanosaur.jpg);
	  width: 60%;
	  margin: 0px auto; 
	}
	```

	更可以搭配 Nesting 使用：

	```scss
	@mixin hover-color($color) {
   		&:hover {
    		color: $color;
    	}
   	}

   	.word {
	   	@include hover-color(red);
   	}
	```

4. Functions 

	在 Sass 內建一些好用 functions 可以簡單設定色相、漸層，例如：`adjust-hue($color, $degrees)`、`fade-out`：

	```scss
	$lagoon-blue: fade-out(#62fdca, 0.5);
	```

	更多內建 Functions 可以[參考這邊](http://sass-lang.com/documentation/Sass/Script/Functions.html)

5. Operations：透過加減乘除和取餘數等運算子可以方便運算所需的屬性值

	顏色加法：

	```scss
	$color: #010203 + #040506;
	/*
		01 + 04 = 05
		02 + 05 = 07
		03 + 06 = 09
		color: #050709;
	*/
	```
	`//` 使用上需要注意：
	```scss
	width: $variable/6; //division
	line-height: (600px)/9; //division
	margin-left: 20-10 px/ 2; //division
	font-size: 10px/8px; //not division
	```

	也可以使用 `@each` 語法迭代 list 內容：

	```scss
	$list: (orange, purple, teal);

	@each $item in $list {
	  .#{$item} {
	    background: $item;
	  }
	}
	```

	使用 `@for` 迭代，並加入條件判斷：

	```scss
	@for $i from 1 through $total {
	  .ray:nth-child(#{$i}) {
		background: adjust-hue(blue, $i * $step);
		// 
		width: if($i % 2 == 0, 300px, 350px);
  		margin-left: if($i % 2 == 0, 0px, 50px);
	  }
	}
	```

6. @include 引用：可以引入其他 Sass、SCSS 檔案：
	
	我們通常使用 `Partials` 去處理特定功能，方便管理和維護。以下是引用 `_variables.scss` 檔案範例，其中檔名前的 `_` 表示引用前要先 compile：

	```scss
  	@import "variables";
	```
7. @extend 共用：
	
	編譯前：

	```scss
	.lemonade {
	  border: 1px yellow;
	  background-color: #fdd;
	}
	.strawberry {
	  @extend .lemonade;
	  border-color: pink;
	}
	```

	轉譯後：

	```scss
	.lemonade, .strawberry {
	  border: 1px yellow;
	  background-color: #fdd;
	}

	.strawberry {
	  @extend .lemonade;
	  border-color: pink;
	}
	```

	搭配 Placeholders 使用：

	轉譯前：

	```scss
	a%drink {
		font-size: 2em;
		background-color: $lemon-yellow;
	}

	.lemonade {
		@extend %drink;
		//more rules
	}
	```

	轉譯後

	```scss
	a.lemonade {
    	font-size: 2em;
    	background-color: $lemon-yellow;
 	}

	.lemonade {
  		//more rules
	}
	```

8. `@mixin` 和 `@extend` 比較

	轉譯前：

	```scss
	@mixin no-variable {
	  font-size: 12px;
	  color: #FFF;
	  opacity: .9;
	}

	%placeholder {
	  font-size: 12px;
	  color: #FFF;
	  opacity: .9;
	}

	span {
	  @extend %placeholder;
	}

	div {
	  @extend %placeholder;
	}

	p {
	  @include no-variable;
	}

	h1 {
	  @include no-variable;
	}
	```

	轉譯後：

	```css
	span, div{
	  font-size: 12px;
	  color: #FFF;
	  opacity: .9;
	}

	p {
	  font-size: 12px;
	  color: #FFF;
	  opacity: .9;
	  //rules specific to ps
	}

	h1 {
	  font-size: 12px;
	  color: #FFF;
	  opacity: .9;
	  //rules specific to ps
	}
	```

9. Sass 資料夾結構

	```
	sass/
		components/
			_buttons.scss
		helpers/
			_variables.scss
			_functions.scss
			_mixins.scss
		layout/
			_grid.scss
			_header.scss
			_footer.scss
		pages/
			_home.scss
			_contact.scss
	```

# 總結
以上就是 Sass/SCSS 簡明入門教學，在這篇文章中我們大致上介紹了 Sass 使用語法。除了 Sass 外，目前市面上還有許多 CSS 的變形，包括語法較易學的 [LESS](http://lesscss.org/#)、元件化思考的 CSS in JS、主要解決全域問題和模組引用的 [CSS Modules](https://github.com/css-modules/css-modules)，取經於 JavaScript Task Runner 的 [PostCSS](https://github.com/postcss/postcss) 、網格樣式表單 [GSS](https://gridstylesheets.org/) 等，這些最終都是要解決傳統 CSS 維護不易、重用性低下等問題。事實上，有些人覺得使用 preprocessor 較好維護，有些人認為每次都要編譯很麻煩，至於要選擇哪一種 CSS preprocessor 或是根本不使用，取決於團隊的內部規範和討論。俗話說後端一天，前端一年，說不定現在選邊站的技術，下個月就不流行了呢。

# 延伸閱讀
1. [Sass & Susy教學手冊](http://sam0512.blogspot.tw/2013/10/sass.html)
2. [learn-sass](https://www.codecademy.com/learn/learn-sass)
3. [CSS Modules 用法教程](http://www.ruanyifeng.com/blog/2016/06/css_modules.html)
4. [如何從 SASS 無痛轉移到 POSTCSS](http://blog.hellojcc.tw/2015/12/11/sass-to-postcss/)
5. [Using PostCSS Together with Sass, Stylus, or LESS](https://webdesign.tutsplus.com/tutorials/using-postcss-together-with-sass-stylus-or-less--cms-24591)
6. [[心得] 什麼是 postcss?](http://huli.logdown.com/posts/262723-experiences-what-is-postcss)
7. [聊聊 CSS 預處理器，LESS、Sass、Stylus 及 postCSS](http://solidzoro.com/blog/article/2015/09/talk-about-css-preprocessor.html)


關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 