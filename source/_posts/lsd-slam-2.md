---
title: 深入學習 LSD-SLAM - 2
date: 2017-04-15 18:50:58
tags:
    - ROS
    - Robotics
    - SLAM
    - LSD SLAM
author: pojenlai
---

## 前言

上回我們在[深入學習 LSD-SLAM - 1](http://blog.techbridge.cc/2017/03/18/lsd-slam-1/)中提到，LSD-SLAM 的一大特色是不計算特徵點、直接使用所有 pixel 來計算相機的姿態。但是，每一幀都有 640x480 個 pixel (作者[建議](https://github.com/tum-vision/lsd_slam#316-general-notes-for-good-results)使用 640x480 解析度的相機)，該怎麼處理這些資料呢？今天就來學習 direct method 的基礎。

在開始之前，我們可以從下圖清楚地看出 Feature-Based Method 跟 Direct Method 的差別，其實就是差在有沒有先抽取特徵點。

![feature-based-vs-direct](/img/pojenlai/lsd-slam-2-1.JPG)

## Direct Method 的基本概念

### 使用條件與假設

Direct method 是直接根據 pixel 值來計算相機姿態，這個方法要成立，需要建立在一個假設 - **灰階程度不變**之上。

```
灰階程度不變假設: 同一個空間點在不同影像中的亮度值相同
```

就是因為有灰階程度不變的假設，我們才能夠直接使用 pixel 值來計算相機姿態，雖然這個假設在現實中常常不成立，但是我們還是可以在這假設之上計算出漂亮的地圖。

### 怎麼讓電腦來解這個問題?

若灰度不變假設成立，那我們就可以開始進行下一步了！但如果你摩拳擦掌地想要開始來寫程式，會發現一件事 - 要怎麼開始寫程式咧?

這時候，就是數學派上用場的良機了。請看看下面這個投影片的 Direct method 部分:

![reprojection_err_vs_photometric_error](/img/pojenlai/lsd-slam-2-2.JPG)

在世界中的一點 $p_i$，會投影在兩個時刻 ($k-1$ 與 $k$) 相機拍到的畫面上，形成 $u_i$ 以及 $u'_i$，我們要做的事情，就變成找出讓 $ \sum_{i} || I_k(u\prime_i)-I_k-1(u_i)||^2 \ $  最小的 Transform - $T_k,k-1$。

於是我們得到了一個可以被最佳化的東西，稱為 Photometric error，有了這個 error，我們就知道寫程式的時候要優化的值是什麼、什麼叫做好的 pose estimation，也才有機會進行下一步。

如果想要更加體會最佳化是什麼樣的動態過程、增加腦海中的對最佳化的想像畫面，我很推薦你去看看[這篇文章](http://www.cnblogs.com/gaoxiang12/p/5689927.html)的 "6.直接法的討論"，裡面的舉例應該非常好懂，也可以加深體會。

其實這一塊要繼續討論下去才能夠真正接到程式碼，不過要再繼續深入，需要不少數學基礎，我會在之後的系列漸漸補上，然後再回來 revisit。

## 總結

這次跟大家介紹了 LSD-SLAM 的基礎，如果不懂 Direct method，就無法再深入體會 LSD-SLAM 的奧妙，我們下回見！

## 延伸閱讀

1. [直接法](http://www.cnblogs.com/gaoxiang12/p/5689927.html)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人跟電腦視覺有>少許研究，最近在鍛鍊自己的執行力
