---
title: 一起用 JavaScript 來複習經典排序法吧！
date: 2017-08-19 18:54:33
tags: sort, algorithm
author: huli
---

# 前言

最近剛好上到 [CS50 Week3](https://www.youtube.com/watch?v=jUyQqLvg8Qw)，這一週的主題是：Algorithms，裡面介紹到了幾種經典的排序法，像是選擇排序、泡沫排序、插入排序以及合併排序。

我覺得身為一個軟體工程師，大概一輩子都脫離不了排序了，畢竟這是經典演算法之一嘛！與其每次要面試之前都凌亂的準備，不如現在就整理出一篇，紀錄一下各個排序法的心得，幫自己做個統整。

因此，這一篇將利用 JavaScript 來實作各個經典排序演算法。

這次實做的排序法都會是由小到大排序，並且為了方便起見，每一個排序法「都會直接更改原本的 array」，但如果你不想改到原本的也很簡單，在每一個的函式最開頭加上：`arr = arr.slice()`複製一份原本的即可。

還有，因為文章裡面比較難放動畫，所以我只能放一些圖片而已，若是想搭配視覺化演算法一起學習的話，我非常推薦 [VISUALGO](https://visualgo.net/en)，這網站絕對會讓你對排序的理解度更上一層樓。

# 選擇排序法（Selection Sort）

選擇排序是我認為最好理解的排序法，因為它的原理超級簡單：

> 找到最小值，移到最左邊。

當你第一輪跑完之後，你就找到整個陣列的最小值了，然後你把尋找範圍從 0 ~ n-1 變成 1 ~ n-1，重複做一樣的事情就可以了。或是，你也可以想成是：找到最小值，第二小值，第三小值...第 n 小值。

![](/img/huli/sorting/selection.jpg)
（圖片來源：http://cheetahonfire.blogspot.sg/2009/05/selection-sort-vs-insertion-sort.html ）

``` js
const selectionSort = (arr) => {
  const length = arr.length;
  
  // 有幾個元素，就要找幾輪的最小值
  // 這邊的 i 代表 i 以前的元素都排序好了
  for (let i = 0; i < length; i++) {
  
    // 先預設第一個是最小的
    let min = arr[i];
    let minIndex = i;
  
    // 從還沒排好的元素開始找最小值
    for (let j = i; j < length; j++) {
      if (arr[j] < min) {
        min = arr[j];
        minIndex = j;
      }
    }
  
    // ES6 的用法，交換兩個數值
    [arr[minIndex], arr[i]] = [arr[i], arr[minIndex]];
  }
  return arr;
}
```

時間複雜度就是大家所熟知的 `O(n^2)`，最好、最壞、平均都是一樣的，因為無論原本的陣列長怎樣，都要經過這麼多輪比較。

# 泡沫排序法（Bubble Sort）

泡沫排序應該是很多人第一個接觸的排序法，原理也很簡單好懂：

> 跟隔壁互相比較，順序錯了就交換，讓大的元素一直浮到最後

就是這樣交換的過程，才讓它稱為「泡沫」排序法，因為元素很像「浮」了上來。

![](/img/huli/sorting/bubble.png)
（圖片來源：http://www.opentechguides.com/how-to/article/c/51/bubble-sort-c.html ）

``` js
const bubbleSort = (arr) => {
  const n = arr.length;
  
  // 一共要跑 n 輪
  for (let i = 0; i < n; i++) {
  
    // 從第一個元素開始，不斷跑到第 n - 1 - i 個
    // 原本是 n - 1，會再加上 - i 是因為最後 i 個元素已經排好了
    // 所以沒必要跟那些排好的元素比較
    for (let j = 0; j < n - 1 - i; j++) {
      if (arr[j] > arr[j + 1]) {
        [arr[j], arr[j + 1]] = [arr[j + 1], arr[j]];
      }
    }
  }
  
  return arr;
}
```

雖然泡沫排序法的平均跟最壞時間複雜度都是`O(n^2)`，但值得注意的是 best case，出現在輸入的陣列已經是排序好的情況下。在這種情況下呢，時間複雜度是 O(n)，不會做任何的交換。

但是呢，如果你要做到最優的情形是 `O(n)`，你必須要加上一個小優化才行。不然以我們上面的情況，雖然不會做任何交換，但還是會把每一個元素都 check 一遍。

可以加上一個 flag 標注內圈有沒有交換的情形發生，如果沒有，就代表陣列已經排序好了，就可以直接跳掉。

``` js
function optimzedBubbleSort = (arr) => {
  const  n = arr.length;
  let swapped = true;
  
  // 一共要跑 n 輪
  for (let i = 0; i < n && swapped; i++) {
  
    // 從第一個元素開始，不斷跑到第 n - 1 - i 個
    // 原本是 n - 1，會再加上 - i 是因為最後 i 個元素已經排好了
    // 所以沒必要跟那些排好的元素比較
    swapped = false;
    for (let j = 0; j < n - 1 - i; j++) {
      if (arr[j] > arr[j + 1]) {
        swapped = true;
        [arr[j], arr[j + 1]] = [arr[j + 1], arr[j]];
      }
    }
  }
  return arr;
}
```

改良之後，如果輸入是已經排好的陣列，就只會跑一次內圈，然後就跳掉了，所以時間複雜度會是`O(n)`。

# 插入排序法（Insertion Sort）

插入排序法是我認為相當直覺的一個排序法，簡單來說就是：

> 你玩撲克牌的時候會用到的排序法

就是不斷把撲克牌插入到適合的位置嘛，只是你玩牌的時候可能一次插入好多牌，而插入排序法是一次插入一張牌。

![](/img/huli/sorting/insertion.gif)
（圖片來源：https://commons.wikimedia.org/wiki/File:Insertion-sort-example.gif ）

這邊比較值得注意的是在插入時候的演算法，不斷往前找到適合的位置，並且在邊找的時候就邊挪動元素了，所以等找到的時候就可以直接插入。

``` js
const insertionSort = (arr) => {
  const n = arr.length;
  
  // 假設第一個元素已經排好，所以從 1 開始跑
  for (let i = 1; i < n; i++) {
  
    // position 表示可以插入的位置
    let position = i;
  
    // 先把要插入的元素存起來
    const value = arr[i];
  
    // 開始往前找，只要符合這條件就代表這個位置是可以插入的
    // 邊找的時候就可以邊把元素往後挪，騰出空間
    while (i >= 0 && arr[position - 1] > value) {
      [arr[position], arr[position - 1]] = [arr[position - 1], arr[position]];
      position--;
    }
  
    // 找到適合的位置，放入元素
    arr[position] = value;
  }
  return arr;
}
```

插入排序法的最佳情形出現在輸入元素已經是排序好的情況，這時候裡面的`while`只要跑一次就會結束了，所以時間複雜到就是外圈的`O(n)`而已。

這邊提一個小插曲，我當初在寫示範跟測試的程式碼的時候沒寫好，導致拿來測試的陣列都是已經排好的，我就想說：「怎麼插入排序法比快速排序法還快，不合理啊！」

# 合併排序法（Merge Sort）

接著要進入到比較快的排序法了，合併排序法算是滿好理解的一個：

> 切一半，排好左邊，排好右邊，合併

談合併排序法的時候我喜歡先談合併這個步驟，其實就是把兩個各自排序好的陣列合併成一個。這一步其實也滿簡單，因為兩邊都已經排序好了嘛，所以就是不斷看兩邊的第一個元素，誰小就抓誰下來，接著左邊抓完就抓右邊，反之亦然。

![](/img/huli/sorting/merge.png)
（圖片來源：http://www.java2novice.com/java-sorting-algorithms/merge-sort/ ）

我自己之前在看合併排序的時候，發現可以寫成一個比較好懂，但是空間耗費比較多的版本：

``` js
const simpleMergeSort = (arr) => {
  
  // 合併
  const merge = (leftArray, rightArray) => {
    let result = [];
    let nowIndex = 0, left = 0, right = 0;
    const leftLength = leftArray.length;
    const rightLength = rightArray.length;
  
    // 如果左右兩邊都沒抓完，就看誰比較小抓誰
    while (left < leftLength && right < rightLength) {
      if (leftArray[left] < rightArray[right]) {
        result[nowIndex++] = leftArray[left++];
      } else {
        result[nowIndex++] = rightArray[right++];
      }
    }
  
    // 跑到這裡代表左右兩邊其中一邊抓完了
    // 如果是左邊沒抓完，全部抓下來
    while (left < leftLength) {
      result[nowIndex++] = leftArray[left++];
    }
  
    // 右邊沒抓完，全部抓下來
    while (right < rightLength) {
      result[nowIndex++] = rightArray[right++];
    }
  
    // 把合併好的陣列直接傳回去
    return result;
  }
  const _mergeSort = (arr) => {
    const length = arr.length;
    if (length <= 1) return arr;
  
    // 切兩半
    const middle = Math.floor(length / 2);
  
    // 排左邊
    const leftArray = _mergeSort(arr.slice(0, middle));
  
    // 排右邊
    const rightArray = _mergeSort(arr.slice(middle, length));
  
    // 合併後丟回去
    return merge(leftArray, rightArray);
  }
  return _mergeSort(arr);
}
```

對我來說，比較簡單的理由是滿直覺的，你就直接用 slice 切成兩個陣列，排序好之後合併起來就好。

但比較省空間的做法是直接更改原來的陣列就好，這時候我們的參數會變得不太一樣：

``` js
function mergeSort = (arr) => {
  const merge = (array, start, middle, end) => {  
  
    // 宣告一個暫時的陣列來放合併後的結果
    let temp = [];
    let nowIndex = 0;
    let left = start;
    let right = middle + 1;
  
    // 這邊都跟上面一樣
    while (left <= middle && right <= end) {
      if (array[left] < array[right]) {
        temp[nowIndex++] = array[left++];
      } else {
        temp[nowIndex++] = array[right++];
      }
    }
  
    while (left <= middle) {
      temp[nowIndex++] = array[left++];
    }
  
    while (right <= end) {
      temp[nowIndex++] = array[right++];
    }
  
    // 要把合併後的陣列放回去 array[start ~ end]
    for (let i = start; i <= end; i++) {
      array[i] = temp[i - start];
    }
  }

  // 代表要從 start 排到 end
  const _mergeSort = (array, start, end) => {
    if (end <= start) return;
    const middle = Math.floor((start + end) / 2);
  
    // 對左右兩半排序
    _mergeSort(array, start, middle);
    _mergeSort(array, middle + 1, end);
    merge(array, start, middle, end);
    return array;
  }
  return _mergeSort(arr, 0, arr.length - 1);
}
```

因為是直接更改原本的陣列，所以要多傳幾個數字進去，代表我要排序這個陣列的那一段。而呼叫完之後，你就可以預設這一段的陣列已經是排序好的了。

基本上流程都跟上面簡單版的沒兩樣，但省了一些記憶體空間。

# 快速排序法（Quick Sort）

快速排序法我一開始覺得滿複雜，知道原理之後就覺得沒那麼難了，其實原理滿簡單：
> 找一個數，並且把這個數調整到：讓左邊的元素比它小，右邊的元素比它大，再對左右兩遍做一樣的事

那個數我們稱作 pivot，會把數列分割成左右兩邊。

例如說現在有一個數列是：14, 7, 6, 9, 10, 20, 15

我們挑選 14 當作 pivot，調整過後變成：7, 6, 9 , 10, `14`, 20, 15，左邊都比它小，右邊都比它大。

而當你把 14 調整好的時候，其實這個元素就排好了！因為左邊比它小，右邊比它大嘛，所以這一個數字就排好了。接著只要對左右兩邊還沒排好的也做快速排序就行了。

而快速排序的核心在於你要怎麼找到那個數，如果你找的數字剛好是數列的中位數，那當然效率最高。如果找的是最小的數，那就是最壞的情形，時間複雜度就變成`O(n^2)`，有分割跟沒分割一樣。

我們直接假設第一個數就是 pivot，這樣比較方便。

那再來有一個問題是，要怎麼把這個數字調整到左邊比它小，右邊比它大呢？我們可以維護一個變數叫做 `splitIndex`，讓這個 index 左邊的元素都比 pivot 小，而這個 index 本身以及它右邊的元素都比 pivot 大。

當你掃一遍陣列，發現某個元素比 pivot 小的時候，就把這個元素跟 splitIndex 上的元素交換，並且把 splitIndex + 1，就可以做到我們上面想做的事情了。最後記得把 pivot 跟 splitIndex - 1（也就是最後一個比它小的元素）交換，就能夠把 pivot 放到正確的位置上了。

可以參考下面的 gif，或是直接去[VISUALGO](https://visualgo.net/en)看看。

![](/img/huli/sorting/quick.gif)
（來源：https://github.com/hustcc/JS-Sorting-Algorithm/blob/master/6.quickSort.md ）

``` js
function quickSort = (arr) => {
  const swap = (array, i , j) => {
    [array[i], array[j]] = [array[j], array[i]];
  }
  const partition = (array, start, end) => {
    let splitIndex = start + 1;
    for (let i = start + 1; i <= end; i++) {
      if (array[i] < array[start]) {
        swap(array, i, splitIndex);
        splitIndex++;
      }
    }
  
    // 記得把 pivot 跟最後一個比它小的元素互換
    swap(array, start, splitIndex - 1);
    return splitIndex - 1;
  }
  const _quickSort = (array, start, end) => {
    if (start >= end) return array;
  
    // 在 partition 裡面調整數列，並且回傳 pivot 的 index
    const middle = partition(array, start, end);
    _quickSort(array, start, middle - 1);
    _quickSort(array, middle + 1, end);
    return array;
  };
  return _quickSort(arr, 0, arr.length - 1);
}
```

# 堆排序（Heap Sort）

Heap 是一種資料結構，並且有分兩種：max heap 跟 min heap，兩種的原理其實雷同，我們直接拿 max heap 來講。

先讓大家看一張 max heap 的圖片：

![](/img/huli/sorting/heap.jpg)
（資料來源：https://www.tutorialspoint.com/data_structures_algorithms/heap_data_structure.htm ）

大家可以發現，max heap 滿足了兩個性質：
1. 父節點一定大於子節點
2. 整個樹的根節點一定是最大值（可以由 1 推出來）

而要用陣列表示 heap 也很簡單，會像這樣：

![](/img/huli/sorting/heap2.png)
（資料來源：http://notepad.yehyeh.net/Content/Algorithm/Sort/Heap/Heap.php ）

所以 heap sort 就是利用這個資料結構做排序，流程很簡單：

1. 先把讀進來的陣列建成 max heap（這時候 arr[0] 一定是這陣列最大值）
2. 把 arr[0] 跟最後一個節點互換（其實是最後一個還沒排序過的節點）
3. 調整成 max heap，回到步驟 2

heap sort 其實有點複雜，複雜到可以再獨立出來一篇了...

但簡單來說呢，就是改良版的選擇排序法，每一次都選最大值出來，然後把剩下的數字再調整成 max heap。

``` js
function heapSort = (arr) => {  
  function heapify(arr, length, node) {
    const left = node * 2 + 1;
    const right = node * 2 + 2;
  
    // 先預設最大的節點是自己
    let max = node;
  
    if (left < length && arr[left] > arr[max]) {
      max = left;
    }
  
    if (right < length && arr[right] > arr[max]) {
      max = right;
    }
  
    // 如果左右兩邊有任何一個比 node 大的話
    if (max !== node) {
      // 就把兩個互換
      [arr[node], arr[max]] = [arr[max], arr[node]];
  
      // 接著繼續 heapfiy
      heapify(arr, length, max);
    }
  }
  
  // build max heap
  const length = arr.length;
  for (let i = Math.floor(length / 2) - 1; i>=0; i--) {
    heapify(arr, length, i);
  }
  
  // 排序
  for (let i = length - 1; i > 0; i--) {
    [arr[0], arr[i]] = [arr[i], arr[0]];
    heapify(arr, i, 0);
  }
  return arr;
}
```

# 總結

其實仔細研究過後，就會發現每一個排序演算法都有值得參考的地方，而且每個排序法都滿有趣的。也會發現懂原理是一回事，寫不寫的出來又是另外一回事了。這篇就當作自己的排序法筆記吧，如果有任何錯誤麻煩不吝指出。

如果想要自己玩玩看的話，[我有放到 Github 上](https://github.com/aszx87410/JavaScript-sorting-algorithm-demo)，有寫好 testcase，改一改就可以直接測了，應該滿方便的。

因為要測試的關係，所以每個排序法前面都會加上：`arr = arr.slice()`避免修改到原本的 array。

測試的過程也滿有趣的，我發現有些 ES6 語法（例如說很潮的交換語法或甚至是`let`）有時候會拖慢執行速度，因此我之前有把語法全部改回 ES5，發現效率快了不少，但這篇因為重點不在效能，所以還是全部用 ES6 的語法。

# 參考資料

1. [[演算法] 堆積排序法(Heap Sort)](http://notepad.yehyeh.net/Content/Algorithm/Sort/Heap/Heap.php)
2. [常见排序算法 - 堆排序 (Heap Sort)](http://bubkoo.com/2014/01/14/sort-algorithm/heap-sort/)
3. [排序之堆積排序法(Heap Sort)](http://marklin-blog.logdown.com/posts/1910116)
4. [js算法:heap sort 使用堆排序](https://my.oschina.net/wanglihui/blog/701263)
5. [JS-Sorting-Algorithm/7.heapSort.md](https://github.com/hustcc/JS-Sorting-Algorithm/blob/master/7.heapSort.md)
6. [用 JavaScript 學習資料結構和演算法：排序（Sort）與搜尋（Search）篇](http://blog.kdchang.cc/2016/09/27/javascript-data-structure-algorithm-sort-and-search/)

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好