---
title: 用 JavaScript 學習資料結構和演算法：陣列（Array）篇
date: 2016-10-08 22:00:00
tags:
	- JavaScript
	- ECMAScript2015
	- ES6
	- Data Structure
	- Algorithm
	- Queue
	- 資料結構
	- 演算法
	- 陣列
	- Array 
author: kdchang
---


![用 JavaScript 學習資料結構和演算法：陣列（Array）篇 ](/img/kdchang/array.jpg)

# 什麼是陣列（Array）？
陣列可以說是程式語言中暫時儲存資料的櫃子，幾乎所有的程式語言都具備陣列這個廣泛運用的資料結構，但值得注意的是一般程式語言中陣列只能儲存同樣型別的值，但在 JavaScript 則可以儲存不同型別的值（但我們還是盡量避免）。

下面是陣列的簡單使用情境，當今天我們想儲存整個班級的學生姓名時，我們不可能使用一個個變數去一個個儲存，這時候陣列的功能就派上用場了：

```javascript
const student1 = 'Mark';
const student2 = 'Zuck';
const student3 = 'Woz';

const students[0] = 'Mark';
const students[1] = 'Zuck';
const students[2] = 'Woz';
```

# 陣列初體驗

![用 JavaScript 學習資料結構和演算法：陣列（Array）篇 ](/img/kdchang/method.png)

接下來我們將實際操作陣列，讓讀者感受一下陣列的使用方式：

1. 建立和初始化陣列

	在 JavaScript 中建立陣列十分容易和彈性，主要有兩種方式：

	使用 `new Array()`，可以自定陣列大小 

	```javascript
	let newArray1 = new Array();
	let newArray2 = new Array(3);
	let students = new Array('Mark', 'Zuck');
	```

	使用 `[]` 宣告陣列，相對彈性

	```javascript
	let newArray1 = [];
	let students = ['Mark', 'Zuck'];
	```

2. 新增、刪除陣列元素

	在 JavaScript 有許多彈性好用的新增刪除陣列的方法，最簡單的新增方式就是把值直接賦值給陣列上最後一個元素：

	```javascript
	let students = ['Mark', 'Zuck', 'Pony', 'Elon'];
	// 由於陣列 index 是從 0 開始，使用陣列長度當 index 賦值就會新增一個元素 
	students[students.length] = 'Mary';
	```

	使用 `push` 可以將元素新增到最後，`pop` 可以移除最後陣列元素：

	```javascript
	let students = ['Mark', 'Zuck', 'Pony', 'Elon'];
	
	// 將 Mary 加入最後元素
	students.push('Mary');
	// 將 Mary 移除
	students.pop();
	```

	整個移動陣列元素，以下示範想在首位插入元素 12：

	```javascript
	let numbers = [1, 2, 3, 4];
	for(let i = numbers.length; i >= 0; i--) {
		numbers[i] = numbers[i - 1];
	}
	numbers[0] = 12;
	```

	`unshift`、`shift` 陣列可以分別將數值插入首位和移除首位元素：

	```javascript
	let students = ['Mark', 'Zuck', 'Pony', 'Elon'];
	
	// 將 Mary 插入首位
	students.unshift('Mary');
	// 將 Mary 移除
	students.shift();
	```

	使用 `splice` 方法可以指定 index 去新增、刪除元素：

	```javascript
	// splice(起始 index, 刪除個數，若為0即為插入, 欲插入元素1, 欲插入元素2, 欲插入元素N)
	let students = ['Mark', 'Zuck', 'Pony', 'Elon'];
	students.splice(0, 2); // students => ['Pony', 'Elon']
	students.splice(1, 0, 'John', 'Cath'); // students => ['Pony', 'John', 'Cath', 'Elon'];
	```

# 二維陣列

事實上在 JavaScript 中只支援一維陣列，但我們可以用陣列嵌套陣列的方式來達成多維陣列：

```javascript
	// 學生和他的成績
	let students = [];
	students[0] = [79, 74, 86];
	students[1] = [97, 54, 46];
```

列印出結果：

```javascript
	for(let i = 0; i < students.length; i++) {
		for(let j = 0; j < students[i].length; j++) {
			console.log(students[i][j]);
		}
	}
```

以上是二維陣列作法，若是多維則以此類推，但越多維維護起資料越複雜。

# 常見陣列方法

陣列相當彈性也功能強大的一種資料結構，除了上面介紹的方法外還擁有許多常用方法。我們這裡也借用 [@tooto1985](https://github.com/tooto1985) 所整理的圖解 JavaScript 陣列操作方法進行講解：


![用 JavaScript 學習資料結構和演算法：陣列（Array）篇 ](/img/kdchang/ja-array-operation.jpg)


## 陣列合併和切割

1. concat

	連接兩個或多個陣列並回傳結果：

	```javascript
	var hege = ["Cecilie", "Leo"];
	var stale = ["Emil", "May", "Linus"];
	var children = hege.concat(stale);

	// ['Cecilie', 'Leo', 'Emil', 'May', 'Linus']
	```

2. slice

	將 array 切片，回傳新的 array：

	```javascript
	var fruits = ['Banana', 'Orange', 'Lemon', 'Apple', 'Raspberry'];
	var citrus = fruits.slice(1, 3); // ['Orange', 'Lemon', 'Apple']	
	```

3. join

	將陣列用分隔符號（separator）串起來成字串

	```javascript
	var fruits = ['Banana', 'Orange', 'Lemon', 'Apple', 'Raspberry'];
	var citrus = fruits.join(); // 預設使用 , 分隔：Banana,Orange,Lemon,Apple,Raspberry		
	```

## 陣列迭代

1. every
	
	對陣列每個元素都進行迭代，若迭代函數內容皆為 true，則回傳 true，反之回傳 false：

	```javascript
	var ages = [32, 33, 16, 40];

	function checkAdult(age) {
	    return (age >= 18) ? true : false;
	}

	console.log(ages.every(checkAdult)); // false
	```

2. some

	與 every 的用法類似，但不同的是若迭代函數中有一個為 true 即回傳 true，反之回傳 false：

	```javascript
	var ages = [3, 10, 18, 20];

	function checkAdult(age) {
	    return (age >= 18) ? true : false;
	}

	console.log(ages.some(checkAdult)); // true
	```

3. map

	迭代每一個陣列元素：

	```javascript
	var numbers = [4, 9, 16, 25];

	numbers.map(function(value, index) {
		console.log(value, index);
	});
	```

4. reduce
	
	由左到右累加（reduce 方法的參數函數共有 previousValue, currentValue, index, array 四個參數）：

	```javascript
	var numbers = [65, 44, 12, 3];

	function getSum(total, num) {
	    return total + num;
	}
	numbers.reduce(getSum); // 124
	```

5. reduceRight

	由右到左累加：

	```javascript
	var numbers = [65, 44, 12, 3];

	function getSum(total, num) {
	    return total + num;
	}

	numbers.reduceRight(getSum); // 124

	```

6. filter

	依據條件回傳過濾後的陣列：

	```javascript
	var ages = [32, 33, 16, 40];

	function checkAdult(age) {
	    return age >= 18;
	}

	ages.filter(checkAdult); // [32, 33, 40]
	```

7. forEach

	迭代陣列元素：

	```javascript
	numbers.forEach(function(value, index){
		console.log(value, index);
	});
	```

## 搜尋和排序

1. indexOf

	回傳值位於陣列內的位置，若值未在陣列中則回傳 -1：
	
	```javascript
	var fruits = ["Banana", "Orange", "Apple", "Mango"];
	var a = fruits.indexOf("Orange"); // 1
	```

2. lastIndexOf

	與 indexOf 類似，但是是回傳最後一次 match 到的 index，同樣若值未在陣列中則回傳 -1：

	```javascript
	var fruits = ["Banana", "Orange", "Apple", "Mango", "Orange"];
	var a = fruits.indexOf("Orange"); // 4
	```

3. reverse

	反轉陣列元素：

	```javascript
	var fruits = ["Banana", "Orange", "Apple", "Mango"];
	fruits.reverse(); ['Mango', 'Apple', 'Orange', 'Banana']
	```

4. sort

	將陣列內容進行排序，回傳排序後陣列，若為文字則根據 [ASCII Code 編碼](https://zh.wikipedia.org/zh-tw/ASCII)大小進行比較：

	```javascript
	var fruits = ["Banana", "Orange", "Apple", "Mango"];
	fruits.sort();
	```

## 輸出陣列為字串

1. toString

	將陣列轉成字串，並回傳結果：

	```javascript
	var fruits = ["Banana", "Orange", "Apple", "Mango"];
	fruits.toString();	// Banana,Orange,Apple,Mango
	```

2. valueOf 
	
	回傳陣列內容：

	```javascript
	var fruits = ["Banana", "Orange", "Apple", "Mango"];
	var v = fruits.valueOf();	
	```

# 總結
以上就是 JavaScript 陣列（Array）介紹和操作方法，讀者們千萬別走看花，實際動手在 console 或是 JSBin 上感受一下，你就會知道 JavaScript Array 的彈性好用！接下來的資料結構介紹裡也會使用到許多 Array 的操作方式。

# 延伸閱讀
1. [JavaScript Array Reference](http://www.w3schools.com/jsref/jsref_obj_array.asp)

（image via [speakingcs](http://2.bp.blogspot.com/-Ek2hzh7Uef8/VcseOEkfxaI/AAAAAAAAAIo/wkiGtnpZcGo/s1600/ArrayJS.JPG)、[dhananjay25](https://dhananjay25.files.wordpress.com/2012/11/image7.png)、[tooto1985](https://github.com/tooto1985/js-array-operations)）

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 