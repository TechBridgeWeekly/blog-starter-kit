---
title: 基礎電腦科學：演算法概要
date: 2019-03-01 22:18:20
author: kdchang
tags: 
    - 程式設計
    - programming
    - coding
    - 演算法
    - 算法
    - computer science
    - algorithm
    - 電腦科學
    - computer science
---

![電腦科學概論：演算法入門概要](/img/kdchang/cs101/algorithm/algorithm-cover.png)

# 前言
隨著資訊科技發展，演算法已經無所不在存在我們的生活當中。舉凡上網 google 搜尋資料、下載檔案的壓縮方法、檔案的加密傳輸等，都可以看到演算法運作的蹤跡。一般來說資料結構和演算法是程式設計最基本的內涵，所以有人說：`程式設計 = 資料結構 + 演算法`。那究竟什麼是演算法/算法呢？

咱們維基百科給了個非常需要慧根才能理解的解釋：

>演算法（algorithm），在數學（算學）和電腦科學之中，為任何良定義的具體計算步驟的一個序列，常用於計算、資料處理和自動推理。精確而言，演算法是一個表示爲有限長，列表的有效方法。演算法應包含清晰定義的指令，用於計算函式。

我們把它翻譯成人話吧：

> 演算法是一個有輸入且有輸出的解決問題的步驟，它具有明確和有限步驟且有效的特性

舉例來說，我們今天要創作一道蔥花蛋或菜脯蛋，我的步驟會是把材料當做輸入：

1. 放點油
2. 打蛋
3. 如果喜歡蔥花可以加入蔥花，如果喜歡菜脯可以加入菜脯（程式術語：`if...else` 條件判斷）
4. 放入少許鹽巴
5. 中火快炒，翻五次面（程式術語：`for` 迴圈）
6. 當看到蛋面呈現金黃色時可以起鍋，結束料理（程式術語：`while` 迴圈）
7. 好吃的蔥花蛋或菜脯蛋上桌

透過清楚明確的有限步驟，我們可以解決我們想解決的問題並產出我們要的輸出結果

# 演算法的定義
一般演算法嚴謹的條件必須符合：

1. 輸入（Input）：0 或多個輸入
2. 輸出（Output）：至少有一個回傳結果（有可能回傳 0 或是 null）
3. 明確性（Definiteness）：每一個指令步驟必須明確
4. 有限性（Finiteness）：在有限步驟後一定會結束不會進入無窮迴圈
5. 有效性（Effectiveness）：步驟清楚可行，能使用紙筆計算求解

舉個例子：

下面是一個 Python 判斷是否可以投票的演算法（假設可以投票為 18 歲），仔細看下面的算法雖然簡單但有輸入也有輸出，且有明確有限步驟，步驟可行

```py
def check_can_vote(age):
    if age >= 18:
        return True
    else:
        return False

check_can_vote(20)
```

# 評估複雜度
事實上，解決一個問題不一定只有一種演算法。那我們怎麼評估演算法的好壞呢？一般來說會有兩種方式：時間複雜度和空間複雜度，比較常見的是使用時間複雜度

## 時間複雜度（Time Complexity）
想要評估一個演算法執行速度快慢，最直覺的方式是測量演算法計算的時間。但由於執行時間會受不同電腦/計算機機器硬體規格與實作方式影響，很難放諸四海皆準，因此學術上傾向於統計演算法步驟數目，當做時間複雜度可考量。

最常見的評估演算法好壞就是時間複雜度，時間複雜度是指運用概量（漸近分析 asymptotic analysis，例如：當 f(n) = n^2 + 3n 這個函數 n 很大時，3n 會比 n^2 小很多，可以忽略不計。當 n 趨近無限大時，f(n) 等價於 n^2）而非絕對時間（因為會牽涉到電腦/計算機環境變因，所以絕對時間不容易準確），通常我們使用 `Big O notation` [大 O 符號](https://zh.wikipedia.org/wiki/%E5%A4%A7O%E7%AC%A6%E5%8F%B7)來表示時間複雜度。假設算法函式所需執行時間為 `T(n)` ，則我們將其時間複雜度表示為 `O(f(n))`。`f(n)` 又稱為執行時間的成長率，是影響速度最大的變數。

首先我們先來看 O(1) 的例子，這個演算法執行的步驟是固定的，跟輸入的值無關：

```py
# 不管 n 輸入為多少，這個程式永遠只會執行一次
def print_num(num):
    print(num)

print_num(10)
```

下面這個例子就是 O(n) 的例子，時間複雜度跟輸入的次數有關，隨著 num 變大所需要跑 num 次，是線性時間的成長。這邊 `f(n)` 等於 `n`，所以 `O(f(n))` 就是 `O(n)`

```py
def sum_number(num):
    total = 0
    for n in num:
        total += num
    return total

sum_number(10)
```

O(nlog(n))

一般常見的時間複雜度如下圖表是：
![電腦科學概論：演算法入門概要](/img/kdchang/cs101/algorithm/big-o.jpeg)

## 空間複雜度
演算法的空間複雜度是指演算法所需要消耗的儲存記憶體資源。其計算和表示方法與時間複雜度類似，一般都用複雜度的漸近性來表示（asymptotic analysis）。

例如下面這個函式，不管程式跑了多少遍，都不會影響使用的變數數量，故該函式的空間複雜度為 O(1)：

```py
def sum_number(num):
    total = 0
    for n in num:
        total += num
    return total

sum_number(10)
```

但下面這個函式，會隨著丟進去的數字而影響變數的量，例如：

輸入為 n，就換產生 n 個變數空間需要儲存，故該函式空間複雜度為 O(n)

```py
def sum_number(num):
    total = []
    for n in num:
        total.append(num)

sum_number(10)

```

# 總結
以上簡單介紹了演算法入門教學。隨著資訊科技發展，演算法已經無所不在存在我們的生活當中。舉凡上網 google 搜尋資料、下載檔案的壓縮方法、檔案的加密傳輸等，都可以看到演算法運作的蹤跡，所以值得我們細細品味。那我們下回見囉！


# 參考文件
1. [台師大演算法筆記](http://www.csie.ntnu.edu.tw/~u91029/)
2. [演算法與資料結構](http://alrightchiu.github.io/SecondRound/mu-lu-yan-suan-fa-yu-zi-liao-jie-gou.html)
3. [大O符號](https://zh.wikipedia.org/wiki/%E5%A4%A7O%E7%AC%A6%E5%8F%B7)
4. [Algorithm Analysis](http://www.csie.ntnu.edu.tw/~u91029/AlgorithmAnalysis.html)
5. [All you need to know about “Big O Notation” to crack your next coding interview](https://medium.freecodecamp.org/all-you-need-to-know-about-big-o-notation-to-crack-your-next-coding-interview-9d575e7eec4)

（image via [pandorafms](https://blog.pandorafms.org/what-is-an-algorithm/)、[freecodecamp](https://cdn-images-1.medium.com/max/1600/1*KfZYFUT2OKfjekJlCeYvuQ.jpeg)）


關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
