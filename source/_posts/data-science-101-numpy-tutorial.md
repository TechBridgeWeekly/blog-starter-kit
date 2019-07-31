---
title: 從零開始學資料科學：Numpy 基礎入門
date: 2017-07-28 09:54:49
tags:
    - Python
    - Machine Learning
    - 機器學習
    - AI
    - Artificial Intelligence
    - NLP
    - Data Mining
    - 人工智慧
    - 監督式學習
    - Supervised learning
    - 從零開始學資料科學
    - Numpy
    - 資料科學
    - data science
    - data scientist
author: kdchang
---

![ 從零開始學資料科學：Numpy 基礎入門](/img/kdchang/numpy.jpg)

# 前言
本系列文章將透過系統介紹資料科學（Data Science）相關的知識，透過 Python 帶領讀者從零開始進入資料科學的世界。這邊我們將介紹 Numpy 這個強大的 Python 函式庫。

# 什麼是 Numpy？
Numpy 是 Python 的一個重要模組（Python 是一個高階語言也是一種膠水語言，可以透過整合其他低階語言同時擁有效能和高效率的開發），主要用於資料處理上。Numpy 底層以 C 和 Fortran 語言實作，所以能快速操作多重維度的陣列。當 Python 處理龐大資料時，其原生 list 效能表現並不理想（但可以動態存異質資料），而 Numpy 具備平行處理的能力，可以將操作動作一次套用在大型陣列上。此外 Python 其餘重量級的資料科學相關套件（例如：Pandas、SciPy、Scikit-learn 等）都幾乎是奠基在 Numpy 的基礎上。因此學會 Numpy 對於往後學習其他資料科學相關套件打好堅實的基礎。

# Numpy 基礎操作

![ 從零開始學資料科學：Numpy 基礎入門](/img/kdchang/arrays.png)

1. Numpy 陣列
    Numpy 的重點在於陣列的操作，其所有功能特色都建築在同質且多重維度的 `ndarray`（N-dimensional array）上。`ndarray` 的關鍵屬性是維度（ndim）、形狀（shape）和數值類型（dtype）。 一般我們稱一維陣列為 `vector` 而二維陣列為 `matrix`。一開始我們會引入 `numpy` 模組，透過傳入 `list` 到 `numpy.array()` 創建陣列。

    ```
    # 引入 numpy 模組
    import numpy as np
    np1 = np.array([1, 2, 3])
    np2 = np.array([3, 4, 5])

    # 陣列相加
    print(np1 + np2) # [4 6 8]
    
    # 顯示相關資訊
    print(np1.ndim, np1.shape, np1.dtype) # 1 (3,) int64 => 一維陣列, 三個元素, 資料型別

    np3 = np.array([1, 2, 3, 4, 5, 6])
    ```

    從檔案取資料：

    ```
    npd = np.genfromtxt('data.csv', delimiter=',')
    ```

    改變陣列維度：

    ```
    np3 = np3.reshape([2, 3])
    print(np3.ndim, np3.shape, np3.dtype) # 2 (2, 3) int64
    ```

    改變陣列型別（bool、int、float、string）：

    `bool` 可以包含 True、False，`int` 可以包含 int16、int32、int64。其中數字是指 `bits`。`float` 可以包含 16、32、64 表示小數點後幾位。`string` 可以是 string、unicode。`nan` 則表示遺失值。

    ```
    np3 = np3.astype('int64')
    np3.dtype
    # dtype('int64')
    ```

2. 建立陣列
    建立填滿 0 或 1 的陣列：

    ```
    np1 = np.zeros([2, 3]) # array([[ 0.,  0.,  0.], [ 0.,  0.,  0.]])
    np2 = np.ones([2, 3]) # array([[ 1.,  1.,  1.], [ 1.,  1.,  1.]])
    ```

![ 從零開始學資料科學：Numpy 基礎入門](/img/kdchang/numpy_indexing.png)

3. 陣列索引與切片
    一維陣列操作和 Python 原生 list 類似：
    
    ```
    np3 = np.array([1, 2, 3, 4, 5, 6])
    print(np3[2]) # 3    
    ```

    二維陣列：
    ```
    np3 = np3.reshape([2, 3])

    print(np3[1, 0]) # 4
    ```

![ 從零開始學資料科學：Numpy 基礎入門](/img/kdchang/axis.png)

4. 基本操作
    使用布林遮罩來取值：

    ```
    np3 = np.array([1, 2, 3, 4, 5, 6])
    print(np3 > 3) # [False False False  True  True  True]
    print(np3[np3 > 3]) # [4 5 6]
    ```

    加總：

    ```
    np3 = np3.reshape([2, 3])
    print(np3.sum(axis=1)) # 將 axis=1 橫向加總 [6 15]
    ```

# 總結
以上介紹了 Numpy 的基礎知識，建立了基本的 array 和 ndarray 的觀念。相信在熟悉 Numpy 之後 Pandas 的學習將會比較容易 （Pandas 的資料容器 DataFrame、Series 事實上是奠基在 Numpy 的陣列上）

# 延伸閱讀
1. [7 Steps to Mastering Machine Learning With Python](http://www.kdnuggets.com/2015/11/seven-steps-machine-learning-python.html)
2. [[python] numpy axis概念整理筆記](http://changtw-blog.logdown.com/posts/895468-python-numpy-axis-concept-organize-notes)

（image via [berkeley](https://bids.berkeley.edu/sites/default/files/styles/400x225/public/projects/numpy_project_page.jpg?itok=flrdydei)、[codingeek](http://www.codingeek.com/wp-content/uploads/2017/01/2D-arrays.png)、[cornell](http://pages.physics.cornell.edu/~myers/teaching/ComputationalMethods/python/anatomyarray.png)、[scipy-lectures](http://www.scipy-lectures.org/_images/numpy_fancy_indexing.png)）


關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 