---
title: Object Recognition Kitchen 透明物體辨識(演算法概念)
date: 2016-12-24 20:10:08
tags:
    - 機器人
    - ROS
    - OpenCV
    - 物體辨識
    - Object Recognition Kitchen
    - 演算法
    - algorithm
author: pojenlai
---

# 前言

這次的文章想要討論一個有趣的題目 – 透明物體辨識，這次的介紹先把題目限定在找出透明物體的位置，並把透明物體的輪廓找出來。

# 演算法功能簡介

我們的演算法目的是要找出影像中的透明物體，並把輪廓圈出來，就像下面這張圖一樣。

![Object Recognition Kitchen 透明物體辨識(演算法概念篇)](/img/pojenlai/glasssegmentation.jpg)

其實如果要更精確，應該把想要辨識哪些透明物體、在那些場景、辨識成功率希望有多高、如何定義辨識成功等都說清楚，不過這邊想先帶給大家一個初步的概念，就不討論得太過瑣碎。

# 演算法概念

為了達成這個功能，我們勢必要先收到彩色影像，所以肯定需要一台相機。不過下一個問題是，我們只需要用彩色影像資訊就能夠抓到透明物體的輪廓嗎? 還是我們需要其他的資訊 (也就是要不要考慮使用其他的 sensor) ?

要回答上面的問題，首先要來想想這個問題會什麼會困難。直覺上應該很容易發現，透明物體的顏色會隨著背景顏色不同而變，所以一般用顏色、紋理特徵來辨識的演算法都不太適用；此外，因為透明物體的邊界常常是模糊不清的，所以跟旁邊的物體很容易混在一起，造成輪廓抓錯。所以核心的問題就是，除了顏色、邊緣這些常用的特徵，我們還可以用什麼資訊來抓透明物體? 有一篇論文提出一個滿有趣的做法，他是使用 Kinect 的深度資訊，而且利用透明物體在 Kinect 的深度圖裡面會有破洞的特性，當成一個辨識的起點:

![Object Recognition Kitchen 透明物體辨識(演算法概念篇)](/img/pojenlai/ork1.png)
![Object Recognition Kitchen 透明物體辨識(演算法概念篇)](/img/pojenlai/ork2.png)

這個概念主要來自這篇論文 – Recognition and Pose Estimation of Rigid Transparent Objects with a Kinect Sensor，他提出的做法是，利用深度圖裡面有大塊破洞的位置，猜測這塊位置是透明物體，接著再根據這個線索找出透明物體的輪廓。

所以下一個問題就是，怎麼根據這個粗略的位置，把透明物體的輪廓切出來，論文中使用的方法是 Grabcut 演算法，這個演算法需要先有一個初始的前景跟背景線段，然後根據這個線段區分出前景跟背景，例如在下圖中，紅色線段是前景、藍色線段是背景，接著 Grabcut 演算法會根據顏色的近似程度還有距離關係，算出綠色框框內那些是前景、哪些是背景，然後把前景切出來。

![Object Recognition Kitchen 透明物體辨識(演算法概念篇)](/img/pojenlai/ork11.png)

切出來之後，目的就算是達成了！當然後續還有很多問題待討論（例如背景的複雜度、怎麼對深度影像中的破碎點做一些基本的處理等），不過今天先把範圍限縮在最上層的概念。

# 總結

演算法的應用真的是千奇百怪，但不管是簡單的演算法題目還是困難的演算法題目，解決的方法都是要看出問題的本質，然後再根據這個問題的特性去使用特定的步驟來解決。像這個範例就是巧妙地利用 Kinect 特性來處理透明物體的特徵很難抓的這個問題，畢竟會在 Kinect 的深度圖中固定呈現出破洞也算是一種特徵，至於更詳細的演算法流程或是程式碼的部分，就留待未來再介紹囉！

# 延伸閱讀

[演算法介紹投影片](http://www.slideshare.net/ssuser54fe9a/seminar20150520)
[ORK 的 transparent object recognition page](http://wg-perception.github.io/transparent_objects/index.html#transparent-objects)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人跟電腦視覺有少許研究，最近在鍛鍊自己的執行力