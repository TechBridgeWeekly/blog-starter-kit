---
title: 用 JavaScript 學習資料結構和演算法：堆疊（Stack）篇
date: 2016-06-24 22:00:00
tags:
	- JavaScript
	- ECMAScript2015
	- ES6
	- Data Structure
	- Algorithm
	- Dictionary
	- Hash Table
	- 資料結構
	- 演算法
	- 陣列
	- 雜湊表
	- 字典
	- 佇列
	- Queue
	- set
	- 集合
	- 堆疊
	- stack
	- 棧
author: kdchang
---

![用 JavaScript 學習資料結構和演算法：堆疊（Stack）篇](/img/kdchang/stack.png)

# 前言
在 CS 江湖上曾傳言：`程式設計 = 資料結構 + 演算法`。在一般的大專院校裡，資料結構（Data Structure）與演算法（Algorithm）幾乎都是電腦科學（Computer Science）和資訊相關科系的基礎必修課，在這些課堂中多半是使用 C/C++ 或是 Java 進行教學，許多初學學生也因為對於這些語言的掌握度不夠，反而迷失在資料結構和演算法的世界裡，然而本系列文章將透過 JavaScript 去學習一些經典的資料結構和演算法。作為一個早期限於瀏覽器端開發的程式語言，現在的 JavaScript 早已不能同日而語，不管是前後端開發、行動端、桌面端、硬體開發等都可以看到它的身影，而 JavaScript 輕量和搭配便利的瀏覽器開發者工具特性也讓學習資料結構和演算法更為有趣！本篇將使用 JavaScript 來介紹堆疊（Stack）這個經典的資料結構。

# 什麼是堆疊（Stack）？
根據教科書的說法：堆疊是一種遵循 `後進先出`（Last In First Out，`LIFO`）的有序集合。在堆疊的世界裡，由於遵守後進先出原則，較新的元素會靠近頂部（堆疊尾部），較舊的元素會在堆疊的底部。在程式語言中，方法（method）的呼叫、運算式的轉換（例如：中序轉後序）或是編譯器和記憶體中儲存變數等都可以看到堆疊的應用。

聽起來離生活有點遙遠？事實上，在生活中有許多使用堆疊的案例：自助餐店的盤子的擺放，書店擺放在桌上的書堆等等，當你把盤子從盤堆最上面拿起來一個盤子就是執行堆疊的 `pop()` 方法，若是把盤子放回盤堆最上面，就是執行 `push(element)` 方法，有沒有很簡單？下次去自助餐廳用餐時，可別忘了和朋友說你現在正在實作一個堆疊喔！

# 建立堆疊
接下來我們正式要開始實作我們堆疊。首先，我們開啟一個 `stack.js` 檔案，建立一個 `Stack` 類別：

```js
function Stack() {
	// 類別內部宣告 Stack 的屬性和方法
}
```

屬性：
1. let items = []; // 我們這邊使用陣列 array 的方式來儲存堆疊內的元素

方法：
1. `push(elements)`：新增一個或多個元素到堆疊頂部
2. `pop()`：移除堆疊頂部元素，同時返回被移除的元素
3. `peek()` 或 `top()`：僅返回堆疊元素，不做任何修改
4. `isEmpty()`：檢查堆疊是否為空？若堆疊內無任何元素即返回 `true`，反之返回 `false`
5. `clear()`：清空堆疊裡的所有元素
6. `size()`：返回堆疊裡的元素個數，類似於陣列的 `length` 屬性

## 1. `push(elements)`、`pop()`
由於我們使用陣列 `Array` 當做堆疊的儲存方式，因此我們可以使用陣列內建的 `push(element)` 和 `pop()` 方式來實作後進先出（Last In First Out，LIFO）的特性，讓元素的新增和刪除只能在堆疊尾端發生。

```js
function Stack() {
	let items = [];
	this.push = function(element) {
		items.push(element);
	}
	this.pop = function() {
		return items.pop();
	}
}
```

![用 JavaScript 學習資料結構和演算法：堆疊（Stack）篇](/img/kdchang/stack-book.png)

## 2. `peek()`
若是我們想知道堆疊中最後一個元素（最頂端），我們可以實作 `peek()` 方法，返回頂部元素。
```js
function Stack() {
	let items = [];
	this.peek = function() {
		return items[items.length - 1];
	}
}
```

## 3. `isEmpty()`
若我們想知道堆疊內部是否還有元素，我們可以使用 `isEmpty()` 來判斷，若堆疊為空返回 `true`，反之返回 `false`。
```js
function Stack() {
	let items = [];
	this.isEmpty = function() {
		return items.length === 0;
	}
}
```

## 4. `clear()`
若我們想清空整個堆疊的話可以使用 `clear()` 方法，將堆疊元素都刪除。
```js
function Stack() {
	let items = [];
	this.clear = function() {
		items = [];
	}
}
```

## 5. `size()`
透過 `size()` 方法我們可以取得堆疊的大小（共有幾個元素）。
```js
function Stack() {
	let items = [];
	this.size = function() {
		return items.length;
	}
}
```

## 完整程式碼

```js
function Stack() {
	var items = [];
	this.push = function(element) {
		items.push(element);
	}
	this.pop = function() {
		return items.pop();
	}
	this.peek = function() {
		return items[items.length - 1];
	}
	this.isEmpty = function() {
		return items.length === 0;
	}
	this.clear = function() {
		items = [];
	}
	this.size = function() {
		return items.length;
	}
	// 加入印出結果方法
	this.print = function() {
		console.log(items.toString());
	}
}
```

# 成果展示
接下來我們可以打開瀏覽器開發工具（例如：Chrome Developer Tool 或是 Firebug）中的 `console` 來實驗我們的堆疊（stack）。
我們可以將上述完成的 `Stack` 類別貼到 `console` 中，並 new 一個 `Stack` 物件給 stack 變數：

```js
// 先貼上上述完整 Stack 類別，然後 new 一個物件
var stack = new Stack();
stack.push('松下問童子');
stack.push('言師採藥去');
stack.push('只在此山中');
stack.push('雲深不知處');
stack.pop();

// 接下來我們將可以透過 stack 物件來操作堆疊的方法：

console.log(stack.size()); // 你答對了嗎？
console.log(stack.peek());
stack.clear();
console.log(stack.isEmpty());
```

# 總結
`stack.push('葡萄美酒夜光杯');`
`stack.push('欲飲琵琶馬上催');`
`stack.push('古來征戰幾人回');`
`stack.pop();`
`stack.push('醉臥沙場君莫笑');`

嘿，你知道現在 `stack.peek()` 的結果嗎？沒錯，答案就是：`醉臥沙場君莫笑`。若是答錯的讀者，別灰心，我們不會笑你的。你可以再回到上面重新閱讀和動手實作，相信很快就能了解堆疊的核心概念。

事實上，堆疊（Stack）、佇列（Queue）都是非常典型的資料結構。以上分享了堆疊（Stack）的基本概念，相信透過動手實作並和生活結合，你會更了解堆疊是一種遵循 `後進先出`（Last In First Out，`LIFO`）的有序集合的意思，接下來我們將持續使用 JavaScript 來介紹常見的經典資料結構和演算法。


# 延伸閱讀
1. [Wiki 堆疊](https://zh.wikipedia.org/wiki/%E5%A0%86%E6%A0%88)
2. [資料結構(Data Structures) (Data Structures) Course 5: Stack and Queue](http://sjchen.im.nuu.edu.tw/Datastructure/98/course05.pdf)
3. [nzakas/computer-science-in-javascript](https://github.com/nzakas/computer-science-in-javascript)

(image via [stack-machine](https://igor.io/img/stack-machine/stack-ops.png))

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 

