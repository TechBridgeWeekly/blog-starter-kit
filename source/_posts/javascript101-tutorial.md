---
title: JavaScript 101 快速入門教學
date: 2017-01-14 22:00:00
tags:
	- JavaScript
	- ECMAScript2015
	- ES6
	- Front End
	- Front End Development Environment
author: kdchang
---

![JavaScript 101 快速入門教學](/img/kdchang/javascript.png)

# 前言

Java 和 JavaScript 雖然名稱相似，但卻是熱狗和狗的差別。JavaScript 是由 Netscape 工程師 Brendan Eich 於 1995 年僅花 10 天所設計的程式語言，也因為一些歷史因素，JavaScript 成為被誤解最深的程式語言。JavaScript 是一種直譯式、基於原型（prototype based）的物件導向程式語言，但又具有函數式程式設計（Functional programming）的特性。其具備簡單好上手、應用範圍廣泛，容易有成就感，但精通不易等特性。過去一段時間 JavaScript 一直被認為是玩具語言，難登大雅之堂。但隨著版本的演進，再加上 NodeJS 的出現，讓 JavaScript 由黑翻紅成為程式語言的當紅巨星，目前雄據在程式語言排行榜前幾名，除了網頁開發外，在許多的領域都可以看到 JavaScript 的身影。本文將快速帶領大家掌握 JavaScript 重要且核心的觀念。

# 環境建置

JavaScript 的檔案只要使用一般的文字編輯器即可編輯（存成 .js 檔案），也可以使用 [Sublime Text](https://www.sublimetext.com/) 等編輯器進行開發，[JSBin](https://jsbin.com/) 這個線上工具進行測試（當然你也可以使用 Chrome、Firefox 等瀏覽器的開發者工具）。除了編輯器外，實際開發上我們也會需要用到 [Node.js 開發環境](https://nodejs.org/en/) 和 NPM 套件管理系統（按照 Node.js 官方指示即可安裝）。Node.js 是一個開放原始碼、跨平台的、可用於伺服器端和網路應用的執行環境。

# JavaScript 程式使用方式（HTML 中引入）

1. 內嵌於 `<head></head>` (因 JS 為直譯式程式語言，讀到即會執行)

	```html
	<head> 
		<script>
			alert(Hello JavaScript);
		</script>
	</head>
	```

2. 內嵌於 `<body></body>` 之中 (可以讓 HTML 優先載入)

	```html
	 <body> 
		<script>
			alert(Hello JavaScript);
		</script>
	 </body>
	```

3. 外部引入檔案置於 `<head>` 或 `<body>` 內 (推薦使用)

	```html
	 <script src="js/main.js"></script>
	```

注意外部引入的 `<script></script>` 中不可再寫 JS。

# 變數

在程式語言中變數是一個暫時儲存資料的地方，賦值給變數的值都會有對應的資料型態，然而 JavaScript 是弱型別（Weak type）所以會有自動轉型的情形。

在 JavaScript 中，全域變數將使用 `var`，而在 ES6 中為了解決變數污染或誤用情形將 `block-scope` 的變數使用 `let`，若你需要固定不變的常數則是使用 `const`。 其中變數型別又分為：Primitive Data Type：String、Number、Boolean（ true 或 false）、undefinded、null，Reference Data Types：Object 兩種，差別在於將物件變數賦值給另外一個變數時是把引用賦值給新變數，所以當新變數更改屬性時會影響到原來物件。

Primitive Data Type：

```javascript
var num = 12;
const str = 'Mark';

function checkLike() {
	let isActived = true;
}

// console.log 可以讓開發者透過瀏覽器開發者工具看到訊息，方便除錯
console.log(num);
console.log(str);
// 因為 let 是 function block scope，故會產生 "ReferenceError: isActived is not defined 錯誤
console.log(isActived);
```

Reference Data Types：

```javascript
// 將物件變數賦值給另外一個變數時是把引用賦值給新變數，所以當新變數更改屬性時會影響到原來物件
let a = { name: 'Zuck' };
console.log(a);
let b = a;
b.name = 'Jack';
console.log(a);
```

# 運算子/運算元

在程式語言中都需要運算子，在 JavaScript 中有賦值運算子、算數運算子、比較運算子、邏輯運算子等不同運算子。

1. 賦值運算子：使用 `=` 給定值給變數

	```javascript
	// Mark
	const name = 'Mark';
	```

2. 算數運算子：四則運算

	```javascript
	// 4
	const sum = 1 + 3;
	```

3. 比較運算子：數值比較

	```javascript
	const age = 23;
	// true
	const canVote = age >= 20;
	// false
	const canVote = age < 20;
	```

4. 邏輯運算子：邏輯判斷

	```javascript
	const a = true;
	const b = true;
	// 且，要兩個都 true
	const result1 = a && b;
	// 或，只要有一個 true，即為 true
	const result2 = a || b;
	```

# 流程控制（flow control）

在 JavaScript 中和許多程式語言一樣有 `if...else`、`switch` 條件判斷以及在處理陣列上很常使用的迴圈（當有明確次數時使用 `for`，當沒有明確數字時使用 `while`）

另外要注意的是在 JavaScript 中的 falsey 值：`undefined`、`null`、`NaN`、`0`、`""`（空字串）和 `false`，以上幾種情況在邏輯判斷時會轉換成 false

1. if...else

	```javascript
	// 可以投票
	if(age > 20) {
		console.log('可以投票！');
	}
	```

2. switch：當條件很多時可以善用 switch 判斷，記得要在每個 case 後寫 break，不然會全部都執行	
	
	```javascript
	const country = 'Taiwan';
	switch(grade) {
		case 'Taiwan':
			console.log('hello' + country);
			break;
		case 'Japan':
			console.log('hello' + country);
			break;
		case 'Korea':
			console.log('hello' + country);
			break;
		default:
			console.log('hello' + country);		
	}
	```

3. for：當你知道程式需要重複執行幾次時可以使用 for 迴圈

	```javascript
	const arr = ['Mark', 'Zuck', 'Jack'];
	for(let i = 0; i < arr.length; arrr++) {
		console.log(arr[i]);
	}
	```

4. while：當你程式不知道需要重複執行幾次時可以使用 while 迴圈

	```javascript
	// 從 1 累加到 10
	const num = 1;
	while(num <= 10) {
		let sum += num; // sum = sum + num
		num += 1;
	}
	```

5. do...while：當迴圈次數不明確時，可以使用 while，而 do while 會至少執行一次

	```javascript
	let x = 0;
	while(x < 10) {
		console.log(x);
		x++;
	}

	let y = 0;
	do {
		console.log(y);
		y++;	
	} while(i < 10);
	```

# 函式/函數（function）

函數是一種可以讓一段程式區塊重複使用的程式撰寫方式，在 JavaScript 中函數屬於一級物件（first class object），可以將函數當參數或變數傳遞，其扮演非常重要的角色，也讓 JavaScript 在函數式程式設計（functional programming）上更容易發揮。
函數可以傳入參數（如下的 x, y），也有可能沒有。函數使用 return 回傳值，若沒寫則回傳 undefined

```javascript
function sum(x, y) {
	return x + y;
}

sum(12, 20);
```

在 ES6 簡化了函數的使用出現了箭頭函數（arrow function）：

```javascript
const sum = (x, y) => {
	return x + y;
};

sum(1, 2);
```

# 物件（object）

物件是一種儲存資料的資料結構，可以對應成真實世界的物件（有屬性值和方法），一般而言主要有三種建立方式：

1. 使用 `new Object`

	```javascript
	var obj = new Object();
	```

2. 使用 `{}`

	```javascript
	var obj = {
		name: 'Mark',
		age: 23
	}
	```

3. 使用建構函數

	雖然 JavaScript 並非是 class-based 的物件導向程式語言，而是 prototype-based 的物件導向程式語言，但在 JavaScript 可以透過 function 建立類別的宣告函數（在 ES6 中已有 class 可以使用，但只是個語法糖）：

	```javascript
	// 實務上建構函數命名採單字首字大寫。狗狗物件有 name, age 屬性，方法是 wow 
	function Dog(name, age) {
		// 屬性值
		this.name = name;
		this.age = age;
		// 每個實例都會有一份方法副本
		this.wow = function() {
			console.log('wow!wow');
		}
	}
	// 多個實例共用，可以減少記憶體等資源運用
	Dog.prototype.cry = function() {
		console.log('QQQ');
	}
	const dog = new Dog('lucky', 2);
	// wow!wow!
	dog.wow();
	```

# DOM & BOM

事實上 HTML 可以轉換成一棵物件樹，也稱為 Document Object Model（DOM）。在撰寫網頁應用程式時往往需要操作到瀏覽器元素，我們通常是透過選取我們想要改變的元素（選擇器），然後修改我們的物件屬性

1. 物件模型

	所謂的物件模型（Object Model）對於HTML 網頁來說，是一種規範如何存取HTML 元素、CSS 樣式和 JavaScript 程式碼的一種機制，可以將HTML元素、CSS樣式和 JavaScript 程式碼視為物件

2. BOM

	![js101](/img/kdchang/bom.jpg)

	BOM 就是 Browser Object Model  中文叫做瀏覽器物件模型，window 物件是瀏覽器最頂層物件，其下有 document（DOM）、history、location、navigator、screen 子物件。`window` 物件不須經過宣告，可直接使用，代表目前瀏覽器視窗。事實上，所有的全域變數、函式、物件，其實都是屬於 window 物件，而 BOM 物件的使用可讓我們操作包含開啟/關閉視窗，改變視窗大小，計時器與取得網址、存取瀏覽器屬性等

3. DOM

	文件物件模型（Document Object Model，DOM）是給 HTML 與 XML 文件使用的一組 API。簡單的說就是將文件（文件可以想成單一網頁）物件化，以便提供一套通用存取的方式來處理文件內容。DOM 提供  HTML 網頁一種存取的方式，可以將 HTML 元素轉換成一棵節點樹，每一個標籤和文字內容是為一個節點，讓我們可以走訪節點 (Nodes) 來存取 HTML 元素 

	```html
	<!doctype html>
	<html lang="en">
	<head>
	  <meta charset="UTF-8">
	  <title>My title</title>
	</head>
	<body>
		<h1>My header</h1>
		<a href="">My link</a>
	</body>
	</html>
	```

	![js101](/img/kdchang/DOM.gif)

	要操作 DOM 元素前要選取要操作哪個
	- 根據ID名稱選取 
	`document.getElementById(elementId)`

	- 根據元素名稱選取
	`document.getElementsByTagName(tagName)`

	- 根據名稱選取 
	`document.getElementsByName(name) `

	- 根據 Class 名稱選取 
	`document.getElementsByClassName(classname)` 

	有很多元素符合，回傳的是 `NodeList` 物件集合，使用 item() 存取 (注意 Element's')，迭代使用 forEach 不然就要轉陣列

	document 物件有提供使用 CSS 選擇器來選取元素，效能較好

	- `document.querySelectorAll()` 方法
	 document 物件的 `querySelectorAll()` 方法可以取得 HTML 的節點陣列或清單，為一個 `NodeList` 物件（若要使用 map 方法需要轉陣列，不然只能用 forEach ）
	 
	- `document.querySelector()` 方法
	只會回傳一個符合的元素，沒有就回傳 null

	範例：

	```html
	<div class="info"></div>
	<div class="info"></div>

	<div id="danger"></div>

	<script type="text/javascript">
		document.querySelector('#danger').innerHTML = '<h1>嘿嘿，是我</h1>';

		document.querySelectorAll('.info').forEach((value, index) => {
		  value.innerHTML = '<h1>坐著打，普天之下排名第二</h1>';
		});
	</script>
	```

# 事件處理（event handler）

事件處理（Event Handlers）是 JavaScript 非常重要的功能，事件是用來處理 JavaScript 與 HTML 之間的互動、建立動畫效果並和使用者互動

- 事件處理簡單說就是當一個事件發生時（網頁載入、按下右鍵等），程式會相對應做出怎樣的處理

- 例如：當使用者按下按鈕時會觸發 click 的事件（事件發生）並讓按鈕變成紅色（處理事件），這就是一種事件處理機制

## 事件處理機制

```
事件處理 = 事件種類 + 事件處理方法
```

- 事件種類（Event Type）
又稱事件名稱 (Event Name)，為一個字串，說明發生了什麼事件，例如：click (點擊)、mousemove (滑鼠滑過)

- 事件處理（Event Handlers）
係指處理事件的函數名稱，當事件發生時要呼叫哪個函數進行處理

```javascript
// 當發生 click 事件，會發出 alert 
btn.addEventListener('click', function() {
	  alert('被點了!');
});  
```

完整範例：

```html
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width">
  <title>JS Bin</title>
</head>
<body>
  <button id="btn">點我點我</button>
  <script type="text/javascript">
	const btn = document.querySelector('#btn');

	btn.addEventListener('click', function() {
	  alert('被點了!');
	});  	
  </script>
</body>
</html>
```

# 非同步處理（Ajax）

傳統上我們會使用 `<form>` 表單和後端程式作互動，然而每次提交表單送出請求給伺服器，伺服器接收並處理傳來的表單，然後送回一個新的網頁。使用 Ajax 應用可以僅向伺服器發送並取回必須的數據，並在客戶端採用JavaScript 處理來自伺服器的回應，不僅減少伺服器負擔也加快反應速度

## 同步 vs. 非同步

![JavaScript 101 快速入門教學](/img/kdchang/sync.png)

依序執行，等到上一個函數任務執行完才能執行下一個

![JavaScript 101 快速入門教學](/img/kdchang/async-1.png)

不會因為上一個函數尚未執行完（例如：回傳結果）就卡住，會往下執行下一個任務

## 什麼是同步/非同步？

- 非同步係指程式不會因為上一個函數尚未執行完（例如：回傳結果）就卡住，會往下執行下一個任務

- 同步就是要等到上一個函數任務執行完才能執行下一個，是依序執行

由於 `DOM 事件處理` 和 `Ajax 呼叫`是非同步處理，所以大部分人會為 JavaScript 貼上非同步程式設計的標籤

## 什麼是 Ajax？

- Ajax 全名：`Asynchronous Javascript And XML`，指的是一套綜合了多項技術的瀏覽器端網頁開發技術

- 雖然 Ajax 中使用 XML 為名，不過 Ajax 不是指一種單一的技術。現在許多應用都使用更輕量的 JSON 進行資料傳輸

- 可以完成不刷頁局部更新應用，使用者體驗較好。不過要小心回調地獄（callback hell）


## 簡易 Ajax 實作

```javascript
// 若需要支援跨瀏覽器，還需要額外檢驗
if (typeof XMLHttpRequest != 'undefined') {
    // 一般使用 XMLHttpRequest 物件
    const xhr = new XMLHttpRequest();
    const REQUEST_URL = 'http://163.29.157.32:8080/dataset/6a3e862a-e1cb-4e44-b989-d35609559463/resource/f4a75ba9-7721-4363-884d-c3820b0b917c/download/393625397fc043188a3f8237c1da1c6f.json';

    // 監聽是否完成
    xhr.onreadystatechange = function() {
        if (xhr.readyState === XMLHttpRequest.DONE) {
            console.log(xhr.responseText);
        }
    }
    
    xhr.open('GET', REQUEST_URL);
    xhr.send();
} 
```

## JSON 基礎概念

- JSON（JavaScript Object Notation）是一種由Douglas Crockford 構想設計、輕量級的資料交換語言，以文字為基礎，且易於讓人閱讀

- JSON 雖然起於 JavaScript，但資料格式與語言無關，目前很多程式語言都支援 JSON 格式資料的生成和解析

- JSON 的官方 MIME 類型是 `application/json`，其副檔名是 `.json`
- 基本格式 `{ "key": "value" }`、`{ "key": ["value1", "value2"] }`  

---

## JSON 長這樣

```json
{
     "name": "John Smith",
     "address": 
     {
         "streetAddress": "21 2nd Street",
         "city": "New York",
         "state": "NY",
         "postalCode": "10021"
     },
     "phoneNumber": 
     [
         {
           "type": "home",
           "number": "212 555-1234"
         },
         {
           "type": "fax",
           "number": "646 555-4567"
         }
     ]
 }
```

# 總結
以上介紹了新手上路 JavaScript 入門核心基礎概念，大家可以善用 [JSBin](http://jsbin.com/) 實際動手操作，並參考 [MDN](https://developer.mozilla.org/zh-TW/) 或 [W3CSchool](http://www.w3schools.com/) 的案例會更清楚整體觀念喔！

# 延伸閱讀
1. [用十分鐘瞭解 陳鍾誠的程式設計課 (採用JavaScript + C的原因)](http://www.slideshare.net/ccckmit/javascript-c)
2. [You-Dont-Know-JS](https://github.com/getify/You-Dont-Know-JS)
3. [JavaScript與前端程式設計入門自學參考](https://tw.twincl.com/javascript/*6731)
4. [JavaScript Garden  ](http://bonsaiden.github.io/JavaScript-Garden/zhtw/)
5. [JavaScript核心](http://weizhifeng.net/javascript-the-core.html)
6. [[稀土掘金日报] JavaScript 开发者必备的资源合集](https://segmentfault.com/a/1190000004253577)
7. [專為中學生寫的 JavaScript 程式書](https://www.gitbook.com/book/ccckmit/javascript/details) 
8. [nzakas/computer-science-in-javascript](https://github.com/nzakas/computer-science-in-javascript)
9. [重新介紹 JavaScript](https://developer.mozilla.org/zh-TW/docs/Web/JavaScript/A_re-introduction_to_JavaScript)

（image via [mpr](https://media.licdn.com/mpr/mpr/AAEAAQAAAAAAAAY4AAAAJDNmOTRkMTk2LTljNDEtNDAwOS05YTJlLTFmMTg2M2YzYzBiMQ.png)、[mahmoudzalt](http://www.blog.mahmoudzalt.com/wp-content/uploads/2014/07/tools.png)）

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 