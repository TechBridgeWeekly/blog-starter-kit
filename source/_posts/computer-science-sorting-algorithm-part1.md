---
title: 基礎電腦科學：排序（sorting）演算法入門上
date: 2019-03-10 22:18:20
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

![電腦科學概論：演算法入門概要](/img/kdchang/cs101/algorithm/sorting.jpg)

# 前言
排序（sorting）和搜尋（search）是演算法（algorithm）中最常見的入門知識。雖然我們在一般程式開發的場合中較少會需要自己實作排序和搜尋演算法，但排序（sort）和搜尋（search）的觀念也常出現在其他的演算法當中，應用層面很廣。本系列文章將使用 Python 來實作幾個經典演算法。首先我們先來介紹：選擇排序、插入排序和氣泡排序法。

# 選擇排序法
選擇排序法是一種十分直觀的排序演算法（就是選擇最小的值和第一個初始值互換），其基本原理如下：
1. 給定一個數字組合和初始最小值位值（一開始 index 為 0）
2. 經過第一輪每個數字和最小值比較，將取出的最小值和第一個數字位置對調（選擇最小的值）
3. 接著除了第一個已排序好的數字外，其餘數字持續最小值比較（index 為 1），若有更小值則和 index 為 1（第二個）值互換
4. 持續進行比較到能比較的值只剩下一個，則由小到大的排序完成

舉例來說；

有一個串列 `[9, 4, 11, 2, 7]` 希望由小到大排序。

初始最小值為 index = 0 的 9

1. 第一次排序後：2 [4, 11, 9, 7]：2 比 9 小，所以互換位置

2. 第二次排序後：2, 4, [11, 9, 7]，4 為初始最小值，沒有可以互換的值

3. 第三次排序後：2, 4, [11, 9, 7]，11 為初始最小值，7 是右邊比 11 小的最小值，兩者互換

4. 第四次排序後：2, 4, 7, [9, 11]，9 為初始最小值，沒有可以互換的值

5. 第五次排序後：2, 4, 7, 9, [11]，11 為初始最小值，沒有可以互換的值

6. 最後排序結果：2, 4, 7, 9, 11

使用 Python 程式實作：

```py
def selection_sort(num_list):
    """
    :param num_list: a list want to sort
    """
    num_list_length = len(num_list)
    for i in range(0, num_list_length):
        min_num_index = i
        for j in range(i + 1, num_list_length):
            if num_list[min_num_index] > num_list[j]:
                min_num_index = j
        num_list[min_num_index], num_list[i] = num_list[i], num_list[min_num_index]

    return num_list

if __name__ == '__main__':
    num_list = [9, 4, 11, 2, 7]
    print('選擇排序前：{}'.format(num_list))
    print('選擇排序後：{}'.format(selection_sort(num_list)))
```

選擇排序法是一種不穩定排序方法，且需要迭代多次，其最好和最壞以及平均時間複雜度皆為 `O(n^2)`，所需儲存空間不因輸入個數改變。僅儲存一個 index 值，故空間複雜度為 `O(1)`

# 插入排序法
插入排序法就像是玩大老二撲克牌一開始整理手牌一樣，拿到新發的牌就插入已排序好的手排對應的位置，其基本原理如下：
1. 給定一個數字組合並假設第一個值成為一已排序數列
2. 經過第一輪每個數字和已排序數列比較，將值插入已排序數列對應位置
3. 直到最後一個數字也已插入已排序列中則排序結束

舉例來說：

有一個串列 `[9, 4, 11, 2, 7]` 希望由小到大排序。

1. 第一步插入 9 後：[9], 4, 11, 2, 7

2. 第二步插入 4 後：[4, 9], 11, 2, 7

3. 第三步插入 11 後：[4, 9, 11], 2, 7

4. 第四步插入 9 後：[2, 4, 9, 11], 7

5. 最後排序結果：[2, 4, 7, 9, 11]

使用 Python 程式實作：

```py
def insert_sort(num_list):
    """
    :param num_list: a list want to sort
    """
    num_list_length = len(num_list)
    for i in range(1, num_list_length):
        insert_value = num_list[i]
        j = i - 1
        while j >= 0:
            if num_list[j] > insert_value:
                num_list[j + 1] = num_list[j]
                num_list[j] = insert_value
            j -= 1

    return num_list

if __name__ == '__main__':
    num_list = [9, 4, 11, 2, 7]
    print('插入排序前：{}'.format(num_list))
    print('插入排序後：{}'.format(insert_sort(num_list)))
```

插入排序法是一種穩定排序方法，其最好情況下時間複雜度為 `O(1)`（一開始已經排序好），最壞以及平均時間複雜度為 `O(n^2)`。所需儲存空間不因輸入個數而改變，故空間複雜度為 `O(1)`

# 氣泡排序法
氣泡排序法顧名思義就是在排序的過程中，排序的值兩兩比較，大的值就像氣泡一樣由水底往上升（往一邊靠攏）。單向氣泡排序基本思路如下（假設由小到大排序）：

對於給定 `n` 個數值，一開始由第一個值依序和之後每個值兩兩比較，若目前值比記錄值大時則交換位置。當完成一輪比較後最大值將於第 n 位，然後前 n - 1 位進行第二輪比較，重複進行過程到只剩下一位為止。

舉例來說；

有一個串列 `[9, 4, 11, 2, 7]` 希望由小到大排序。

1. 第一輪交換後：4, 9, 2, 7, 11

2. 第二輪交換後：4, 2, 7, 9, 11

3. 第三輪交換後：2, 4, 7, 9, 11

4. 最後排序結果：2, 4, 7, 9, 11

使用 Python 實作：

```py
def bubble_sort(num_list):
    """
    :param num_list: a list want to sort
    """
    list_length = len(num_list)
    for i in range(0, list_length):
        for j in range(i + 1, list_length):
            if num_list[i] > num_list[j]:
                num_list[i], num_list[j] = num_list[j], num_list[i]
    return num_list

if __name__ == '__main__':
    num_list = [9, 4, 11, 2, 7]
    print('氣泡排序前：{}'.format(num_list))
    print('氣泡排序後：{}'.format(bubble_sort(num_list)))
```

氣泡排序法是一種穩定排序方法，其最好情況下時間複雜度為 `O(n)`（一開始已經排序好），最壞以及平均時間複雜度為 `O(n^2)`。所需儲存空間不因輸入個數而改變，故空間複雜度為 `O(1)`。


# 總結
以上簡單介紹了選擇排序、插入排序和氣泡排序法並使用 Python 來實作幾個經典演算法，下回我們將持續介紹更多經典且常用的演算法，那我們下次見囉！

# 參考文件
1. [台師大演算法筆記](http://www.csie.ntnu.edu.tw/~u91029/)
2. [演算法與資料結構](http://alrightchiu.github.io/SecondRound/mu-lu-yan-suan-fa-yu-zi-liao-jie-gou.html)
3. [大O符號](https://zh.wikipedia.org/wiki/%E5%A4%A7O%E7%AC%A6%E5%8F%B7)
4. [Algorithm Analysis](http://www.csie.ntnu.edu.tw/~u91029/AlgorithmAnalysis.html)
5. [All you need to know about “Big O Notation” to crack your next coding interview](https://medium.freecodecamp.org/all-you-need-to-know-about-big-o-notation-to-crack-your-next-coding-interview-9d575e7eec4)

（image via [pandorafms](https://blog.pandorafms.org/what-is-an-algorithm/)、[brilliant](https://brilliant.org/wiki/sorting-algorithms/)）


關於作者：
[@kdchang](https://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
