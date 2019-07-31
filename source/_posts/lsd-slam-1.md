---
title: 深入學習 LSD-SLAM-1
date: 2017-03-18 00:00:43
tags:
    - ROS
    - Robotics
    - SLAM
    - LSD SLAM
author: pojenlai
---

## 前言

SLAM 是近年來很火紅的一個技術，也有不少文章在介紹，不過我很少看到針對一些頂級演算法的深入介紹，所以才想透過這篇文章來開始深入很久以前玩過的[LSD SLAM](http://vision.in.tum.de/research/vslam/lsdslam)，也跟大家一起學習這個迷人的演算法。建議有興趣的讀者先讀過[這篇平易近人的 Visual SLAM 簡介](http://www.leiphone.com/news/201607/GLQj0wrjKD4eHvq5.html)，然後再繼續往下看，會更有感覺！

先放一張 LSD-SLAM 建出來的地圖，讓大家感受一下帥度。

![lsd-cool](/img/pojenlai/lsd-slam-1-1.JPG)

## 安裝方式

安裝方式可以到 LSD-SLAM 的 [github page](https://github.com/tum-vision/lsd_slam#2-installation) 看看，筆者很久以前跑過 Ubuntu 12.04 上面的版本，建議可以看看作者的[溫馨提示](https://github.com/tum-vision/lsd_slam#316-general-notes-for-good-results)，比較容易建出好的地圖。

如果想看中文的安裝教學，也可以看看這一篇 [LSD-SLAM 編譯過程 (Ubuntu 14.04 + ROS Indigo)](http://blog.csdn.net/xueyinhualuo/article/details/48490939)

## 演算法簡介

![lsd-algo](/img/pojenlai/lsd-slam-1-2.PNG)

從上圖中我們可以看到，LSD-SLAM 演算法的一個特色就在於它不需要計算特徵點，這個特色在 real-time 的應用中滿重要的，畢竟一般影像是以 30 fps
的速率進來，也就是一張影像只能處理 33 ms，要怎麼在這麼短的時間內就處理完一幀，是一個問題。有些計算特徵的演算法雖然厲害(例如 SIFT、ORB)，但計算時間太長，佔掉 30 ms 的一大部分，就很難在 real-time 應用中派上用場。

LSD-SLAM的基本想法就是利用所有 pixel 資訊不斷計算目前相機所在的位置，等到目前的位置離前一個 keyframe 足夠遠，就儲存一張新的 keyframe。並且在過程中不斷計算 depth map 形成 point cloud map，也不斷對 map 做 optimization 計算。

如此一來，就可以形成一個完整的 map，整個 map 就是一個巨大的 pose graph，graph 裡面的 vertice 是 keyframe，edge 就是連接 keyframe 的 3D similarity transform。

## 總結

這篇很簡單地介紹了 LSD-SLAM，算是一個起頭，讓大家對這個演算法有基本的認識，之後我會慢慢深入裡面的細節，也會逐漸變得好玩！

## 延伸閱讀

1. [雷鋒網 SLAM 相關文章](http://www.leiphone.com/tag/SLAM)
2. [相機位姿估計0：基本原理之如何解PNP問題](http://www.cnblogs.com/singlex/p/pose_estimation_0.html)
3. [SLAM Tutorial@ICRA 2016](http://www.dis.uniroma1.it/~labrococo/tutorial_icra_2016/)
4. [高翔大神的blog](http://www.cnblogs.com/gaoxiang12/tag/%E8%A7%86%E8%A7%89SLAM/)
5. [CMU Designing Computer Vision Apps - lecture 19](http://16623.courses.cs.cmu.edu/slides/Lecture_19.pdf)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人跟電腦視覺有少許研究，最近在鍛鍊自己的執行力

