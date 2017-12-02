---
title: Guided Policy Search 環境安裝
date: 2017-12-02 22:37:01
tags: ROS, Guided Policy Search, Reinforcement Learning
---

## 前言

Guided Policy Search (GPS) 是一個滿酷的 policy search 方法，接下來我們會來介紹這個 package 的安裝還有啟動方法，讓大家好入個門。如果你不太了解什麼是 policy 跟 policy search，可以看看 [這篇文章](https://pojenlai.wordpress.com/2017/10/26/what-do-policy-and-policy-search-mean/)。

因為核心的安裝方法在網頁上已經寫得滿清楚，所以這篇文章主要是側重在幫大家整合一些可能需要的資源（因為安裝過程中可能會遇到一些問題）。

## 安裝方法

首先，要先安裝一些基本的 [dependency](http://rll.berkeley.edu/gps/#dependencies)，這個可以直接參考 GPS 的網頁。需要有 python, caffe/Tensorflow 等等東西，網路上都有很多資源了，就不再贅述。

接下來，要 clone GPS 的 package：

```
git clone https://github.com/cbfinn/gps.git
```

然後就是去執行設定：

```
cd gps
./compile_proto.sh
```

如果你在安裝完畢後有遇到其他問題，你可以看看 [這個 issue](https://github.com/cbfinn/gps/issues/61)，或者也可以看 [這個 issue](https://github.com/cbfinn/gps/issues/91)，應該就可以 cover 大部分遇到的問題了！

## 啟動 GPS 

這個 package 裡面已經提供了幾種測試環境，Box 2D、Mujoco 跟 Gazebo PR2，因為個人覺得 PR2 比較大隻，可以做到更複雜的任務，所以就講一下怎麼啟動 PR2 的範例。

一開始要先啟動 Gazebo 跟弄出一隻 PR2：

```
roslaunch gps_agent_pkg pr2_gazebo.launch
```

這一步你如果碰到問題，可以參考 [這篇文章](https://pojenlai.wordpress.com/2017/10/28/how-to-solve-resourcenotfound-gazebo_worlds-error-when-running-simulated-pr2-for-gps/)。

接下來是啟動 Guided Policy Search：

```
python python/gps/gps_main.py pr2_example
```

跑起來之後應該就會看到如下的視窗，然後 Gazebo 裡面的 PR2 會開始動，Window 中也會開始顯示訓練時的軌跡資料：

![gps-window](/img/pojenlai/ros-gps-window.png)

接下來就會看到 PR2 的左手一直在進行嘗試，這些嘗試是為了讓 PR2 的左手可以到達目標位置：

![gazebo-pr2](/img/pojenlai/ros-gps-pr2-fin.png)

達到目標之後，GPS 的視窗就會顯示如下：

![gps-window-2](/img/pojenlai/ros-gps-window-2.png)

## 下一步是什麼？

下一步就看你想要拿 GPS 來做什麼實驗，你可以[套用自己的機器人](http://rll.berkeley.edu/gps/index.html#learning-with-your-own-robot)、[設計自己的實驗](http://rll.berkeley.edu/gps/#running-a-new-experiment)等等，或是以目前的環境為起點完全弄懂 GPS 的實作細節，有滿多東西可以延伸的。

## 總結

今天跟大家介紹了 Guided Policy Search 的環境安裝跟基本概念，主要是想幫助有興趣的讀者可以減輕入門的環境安裝門檻，更詳細的演算法概念可以參考 [深入淺出 End-to-End Learning on Robotics](https://pojenlai.wordpress.com/2017/12/02/%E4%B8%80%E8%B5%B7%E8%AE%80-end-to-end-training-of-deep-visuomotor-policies/)，或你也可以去看 [Guided Policy Search 的論文](https://people.eecs.berkeley.edu/~svlevine/papers/mfcgps.pdf)，會幫助你對演算法本身更加了解。

## 延伸閱讀

1. [深入淺出 End-to-End Learning on Robotics](https://pojenlai.wordpress.com/2017/12/02/%E4%B8%80%E8%B5%B7%E8%AE%80-end-to-end-training-of-deep-visuomotor-policies/)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人跟電腦視覺有少許研究，最近在學習[看清事物的本質與改進自己的觀念](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)
