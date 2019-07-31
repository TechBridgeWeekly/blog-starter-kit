---
title: 用 JavaScript 學習資料結構和演算法：佇列（Queue）篇
date: 2016-12-10 22:00:00
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
author: kdchang
---

![用 JavaScript 學習資料結構和演算法：佇列（Queue）篇](/img/kdchang/queue.png)

# 什麼是佇列（Queue）？
佇列（Queue）是一種先進先出（First In First Out, FIFO）的有序串列（Ordered List），與堆疊（Stack）後進先出（Last In First Out, LIFO）不同的是佇列（Queue）的新增和刪除元素是發生在不同端，新增元素在尾部、移除元素在頂部，最新加入的元素會從尾部排入。在一般生活中比較常見的例子是電影院排隊買票、小七便利商店排隊付款（當然不能有人想插隊啦），在計算機科學領域佇列（Queue）應用也十分常見，像是印表機列印順序排程（當我們點選列印鍵時，第一個列印文件會先被列印，後續送來的文件會依序列印）或是一般作業系統的工作佇列（job queue）、I/O Buffer 緩衝區，也是透過先進先出來處理。

以下是佇列（Queue）幾個特性：

1. 佇列（Queue）是一組相同性質的元素組合
2. 具有先進先出（First In, First Out, FIFO）特性
3. 新增元素時發生在 Rear 後端
4. 刪除元素時發生在 Front 前端 
5. 新增/刪除（Enqueue/Dequeue 或 Add/Delete）元素是發生在不同端

# 用陣列（Array）實作佇列（Queue）

接下來我們將使用 JavaScript 的一維陣列來實作佇列（Queue），其基本步驟如下：

1. 宣告佇列類別

	```javascript
	function Queue() {
		// 這裡將放置佇列的屬性和方法
	}
	```

2. 宣告一個一維陣列（Array）

	首先我們使用一個一維陣列（Array）當做儲存佇列元素的資料結構，這部份和我們之前提到的堆疊（Stack）類似，只是在佇列新增刪除元素時是在不同端點進行（這邊使用 `let` 讓 `scope` 維持在 `function` 中）：

	```javascript
	function Queue() {
		let items = [];
	}
	```

3. 建立佇列可用方法（method）

	- enqueue(element)：於佇列尾端新增一個或多個元素

	```javascript
	this.enqueue = function(element) {
		items.push(element);
	};	
	```

	- dequeue()：刪除佇列第一個（頭部）的元素，並回傳被移除的元素（在 	`JavaScript` 中 `shift()` 用於移除陣列第一個元素 `items[0]`，也就是頭部）

	```javascript
	this.dequeue = function() {
		return items.shift();
	}
	```

	- front()：回傳佇列中第一個元素，但佇列本身不作任何更動

	```javascript
	this.front = function() {
		return items[0];
	}
	```

	- isEmpty()：如果佇列中不包含任何元素，則回傳 `true`，反之回傳 `false`

	```javascript
	this.isEmpty = function() {
		return items.length === 0;
	}
	```

	- size()：回傳佇列所包含的元素個數，即為回傳陣列的 `length`

	```javascript
	this.size = function() {
		return items.length;
	}
	```

## 完整佇列（Queue）類別：

```javascript
function Queue() {
	let items = [];
	this.enqueue = function(element) {
		items.push(element);
	};
	this.dequeue = function() {
		return items.shift();
	};
	this.front = function() {
		return items[0];
	};
	this.isEmpty = function() {
		return items.length === 0;
	};
	this.clear = function() {
		items = [];
	};
	this.size = function() {
		return items.length;
	};
	this.print = function() {
		// 列印出佇列內容
		console.log(items.toString());
	}
}
```

## 使用 Queue 類別

```javascript
// 由於上面我們已經建立好了 Queue 的類別，我們這邊直接使用。亦可以瀏覽器 console 直接看結果
const queue = new Queue();
// 判斷佇列是否為空
console.log(queue.isEmpty());
// 將值放入佇列中
queue.enqueue('令狐衝');
queue.enqueue('西方不拜');
queue.enqueue('田薄光');
queue.enqueue('任贏贏');
// 回傳佇列首位
console.log(queue.front());
// 回傳佇列長度
console.log(queue.size());
// 列印出佇列內容：令狐衝,西方不拜,田薄光,任贏贏
queue.print();
// 移除佇列首位
queue.dequeue();
// 回傳佇列首位
console.log(queue.front());
// 回傳佇列長度
console.log(queue.size());
// 列印出佇列內容：西方不拜,田薄光,任贏贏
queue.print();
```

# 優先級佇列（Priority Queue）

優先級佇列（Priority Queue）是一般佇列的修改版本，為一種不必遵守佇列特性－FIFO(先進先出)的有序串列。其規定每個元素都要有優先級，優先級最高的會最早獲得服務，若是優先級相同則看排列順序。優先佇列可以利用陣列結構及鏈結串列來實作。在生活中我們也可以看到優先級佇列（Priority Queue）的真實應用，例如：VIP 會員可以優先排隊進場、道路行駛時救護車優先於其他車輛，甚至是急救時也會有病情嚴重分類。於計算機科學領域中，CPU 工作排程也常會用到優先佇列。

## 建立優先級佇列（Priority Queue）類別

```javascript
function PriorityQueue() {
	let items = [];
	function QueueElement(element, priority) {
		this.element = element;
		this.priority = priority;
	}
	this.enqueue = function(element, priority) {
		const queueElement = new QueueElement(element, priority);
		if(this.isEmpty()) {
			items.push(queueElement);
		} else {
			let added = false;
			for(let i = 0; i < items.length; i++) {
				if(queueElement.priority < items[i].priority) {
					items.splice(i, 0, queueElement);
                  	added = true;
                  	break;
				}
			}
			if(!added) {
				items.push(queueElement);
			}			
		}
	}
	this.dequeue = function() {
		return items.shift();
	};
	this.front = function() {
		return items[0];
	};
	this.isEmpty = function() {
		return items.length === 0;
	};
	this.clear = function() {
		items = [];
	};
	this.size = function() {
		return items.length;
	};
	this.print = function() {
		// 因為是物件，所以使用 JSON.stringify() 列印出佇列內容
		console.log(JSON.stringify(items));
	}
}
```

## 使用優先級佇列（Priority Queue）類別

```javascript
// 由於上面我們已經建立好了 PriorityQueue 的類別，我們這邊直接使用。亦可以瀏覽器 console 直接看結果
const priorityQueue = new PriorityQueue();
// 判斷佇列是否為空
console.log(priorityQueue.isEmpty());
// 將值和優先序放入佇列中
priorityQueue.enqueue('令狐衝', 2);
priorityQueue.enqueue('西方不拜', 1);
priorityQueue.enqueue('田薄光', 4);
priorityQueue.enqueue('任贏贏', 3);
// 回傳佇列優先序首位 QueueElement {element: "西方不拜", priority: 1}
console.log(priorityQueue.front());
// 回傳佇列長度
console.log(priorityQueue.size());
// 列印出佇列內容：[{"element":"西方不拜","priority":1},{"element":"令狐衝","priority":2},{"element":"任贏贏","priority":3},{"element":"田薄光","priority":4}]
priorityQueue.print();
// 移除佇列首位西方不拜
priorityQueue.dequeue();
// 回傳佇列首位 QueueElement {element: "令狐衝", priority: 2}
console.log(priorityQueue.front());
// 回傳佇列長度
console.log(priorityQueue.size());
// 列印出佇列內容：[{"element":"令狐衝","priority":2},{"element":"任贏贏","priority":3},{"element":"田薄光","priority":4}]
priorityQueue.print();
```

# 環狀佇列（Circular Queue）

環狀佇列（Circular Queue）是指一種環形結構的佇列，它是利用一種 Q[0: N-1] 的一維陣列，同時 Q[0] 為 Q[N-1] 的下一個元素。由於一般佇列會遇到明明前端頭部尚有空間，但再加入元素時卻發現此佇列已滿。此時的解決方法就是使用環形佇列（Circular Queue）。

環狀佇列（Circular Queue）特性如下：

1. 環狀佇列是一種環形結構的佇列
2. 環狀佇列最多使用（N-1）個空間
3. 指標 Front 永遠以逆時鐘方向指向佇列前端元素的前一個位置 Q[N]
4. 指標 Rear 則指向佇列尾端的元素

若再加入一個項目時，Rear 等於 0，也就是 Front 等於 Rear，如此無法分辨此時佇列是空的還是滿的。因此，環形佇列必須浪費一個空格。當 Front 等於 Rear 時，環形佇列為空的。當（Rear+1）Mod N 等於 Front 時，環形佇列為已滿，通常環狀佇列最多使用（N-1）個空間。

# 總結

在這章我們學到了：

1. 什麼是佇列（Queue）？它和堆疊（Stack）有什麼差別？
2. 如何使用 `JavaScript` 建立 `Queue` 類別？
3. 優先級佇列（priority queue）是什麼？有何特性？
4. 環狀佇列（circular queue）是什麼？有何特性？

事實上，除了上述介紹的一般佇列、優先級佇列和環狀佇列外，還有雙向佇列（不像堆疊的 LIFO 或佇列的 FIFO，其允許在兩端都可以新增刪除元素）等特殊佇列。若是對於佇列（Queue）掌握度還不夠的讀者不妨用生活的情境進行聯想，想想你平常在電影院或是大賣場買東西排隊結帳的情景，會更容易幫助理解喔！

# 延伸閱讀
1. [[Algorithm][C / C++] 佇列(Queue)、環狀佇列(Circular Queue)](https://j101044.wordpress.com/2014/08/19/algorithmc-c-%E4%BD%87%E5%88%97queue/)
2. [佇列(Queue)](http://epaper.gotop.com.tw/pdf/AEE032400.pdf)
3. [[ 資料結構 小學堂 ] 佇列 : 佇列的應用 (環狀佇列)](http://puremonkey2010.blogspot.tw/2010/10/blog-post_05.html)
4. [資料結構第4-7章補充教材](http://bilab.pro/ds/%E7%AC%AC4-7%E7%AB%A0%E8%A3%9C%E5%85%85%E6%95%99%E6%9D%90/ch4_%E8%A3%9C%E5%85%85%E6%95%99%E6%9D%90.pdf)

(image via [stack-machine](https://igor.io/img/stack-machine/stack-ops.png)、[wikipedia](https://upload.wikimedia.org/wikipedia/commons/thumb/5/52/Data_Queue.svg/2000px-Data_Queue.svg.png))

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 