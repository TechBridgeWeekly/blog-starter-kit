---
title: 深入學習 LSD-SLAM 番外篇 - RDS X RTAB-Map
date: 2017-06-10 21:49:02
tags:
    - ROS
    - Robotics
    - SLAM
    - LSD SLAM
    - RTAB-Map
author: pojenlai
---

## 前言

因為前陣子因緣際會發現了 [Robot Ignite Academy](http://www.theconstructsim.com/construct-learn-develop-robots-using-ros/robotigniteacademy_learnros/) 跟 [ROS Develop Studio（RDS）](http://www.theconstructsim.com/rds-ros-development-studio/)，很希望可以推薦給我們的讀者。所以這期專欄我延伸一下 LSD SLAM 系列，先跟大家介紹這個很酷的學習資源，並介紹跟 LSD SLAM 目的相似的 RTAB-Map 演算法。

## 為什麼要介紹 RDS？

學習的方法有許多種，借引用這一週很紅的 [英文學習 repository](https://github.com/byoungd/english-level-up-tips-for-chinese-programmers-and-designers) 的圖：

![multi-dimension-learning](https://camo.githubusercontent.com/939994430f33c07bd2c4a1da0e1cfceebcd501fb/68747470733a2f2f6f6f6f2e306f302e6f6f6f2f323031372f30362f30382f353933386233626133396539382e706e67)

學習越難的東西，越不容易只靠單一學習資源就學通透，舉例來說，你很難只看文章不動手實作就真的學會寫程式。

同樣地，學習機器人如果不碰機器人，很難真正弄懂機器人。Gazebo 的出現確實幫助許多無法輕易取得機器人的人，使用模擬機器人來學習。但是，如果對 Linux 不熟，很可能在環境安裝的階段就會碰到障礙導致學習中斷。

今天介紹的 RDS 就可以先排除學期初期的環境設定問題，直接使用 Web UI 來呈現模擬器、command window、檔案系統，讓使用者可以直接切入機器人的開發。除此之外，[Robot Ignite Academy](http://www.theconstructsim.com/construct-learn-develop-robots-using-ros/robotigniteacademy_learnros/) 直接提供了跟 RDS 整合在一起的線上課程，等於是你完全不需要有任何基礎準備，你只要有一個瀏覽器，就可以連上 RDS 開始學習機器人。

就是因為他們提供了一個很方便的學習環境，所以我才想寫一篇文章來跟大家分享，或許可以幫助更多人更容易感受到學習機器人的滋味。

## RDS 簡介

RDS 是 [The construct](http://www.theconstructsim.com/) 這間公司開發出來的工具，下圖是我去上他們的課程截的圖，你可以看得出來，最左邊是課程內容，右邊有檔案系統的 UI、模擬器畫面，下方還有 command window 以供輸入指令。視窗的安排也有幾種配置方式，可以隨自己喜歡去編排。

![LSD-SLAM-side-1-1](/img/pojenlai/lsd-slam-side-1-1.JPG)

想要使用 RViz 等工具也不用緊張，可以額外開一個分頁顯示桌面，個人覺得應有盡有了。

![LSD-SLAM-side-1-2](/img/pojenlai/lsd-slam-side-1-2.JPG)

## Robot Ignite Academy

看完上面的介紹，你可能會問，現在有哪些東西可以用 RDS 學？

基本上都在這個截圖裡了，是說未來也還會再增加，所以有興趣的讀者可以持續關注。

![ria](/img/pojenlai/lsd-slam-side-1-3.JPG)

## RTAB Map 簡介

RTAB-Map (Real-Time Appearance-Based Mapping) 是一種 RGB-D SLAM 的方法，跟 LSD SLAM 的目的很像，都是為了建出 3D 地圖並定位感測器位置，最主要的差別在於，LSD SLAM 使用的感測器是一個 RGB camera，而 RTAB-Map 使用的是 Kinect 這種 RGB-D camera。

這個番外篇沒有要詳細介紹這個演算法的設計概念，所以就直接從使用的角度出發啦。

因為 RTAB-Map 本身是一種 SLAM 的演算法，有人寫了 [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) 這個 package，要在 ROS 中跑起 RTAB-Map 直接靠這個 package 就行了。不過要使用，還是需要一些基本的認知，接下來就簡短地跟大家介紹一下！

RTAB-Map 基本上有兩個模式 - Mapping mode 跟 Localization mode，Mapping mode 顧名思義就是建地圖，RTAB Map 會把地圖資料（例如影像、2D 地圖、3D 地圖）存起來，可以見下圖。

![db_visualization](https://github.com/introlab/rtabmap/wiki/doc/Tools/database_viewer.png)

Localization mode 就是根據已經建好的地圖資料庫，來定位機器人現在的位置，一旦可以將機器人感測到的資料對應到地圖，就算是定位成功。

## 實際操作 RTAB-Map

如果你去試玩看看他們的課程，直接用下面幾個指令就可以準備好所有東西，然後用你的鍵盤操控 turtlebot 就可以開始建地圖了。

```
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start"
roslaunch rtabmap_ros demo_turtlebot_rviz.launch
roslaunch turtlebot_teleop keyboard_teleop.launch
```

如果你有自己的 RGB-D sensor 想要玩，那你就直接參考[安裝方法](https://github.com/introlab/rtabmap_ros#ros-distribution)跟[RGB-D Hand-Held Mapping With a Kinect](http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping#Mapping_mode)也可以很快上手！

然後你可以用這一個指令啟動 Localization mode，那機器人收到的感測器資料就會被拿來跟地圖比對：

```
roslaunch rtabmap_ros rtabmap.launch localization:=true
```

哈不過之所以能這麼容易就啟動，也是因為 launch 檔把這個黑盒子包得很好，你可以去看看 [rtabmap.launch](https://github.com/introlab/rtabmap_ros/blob/master/launch/rtabmap.launch)。

## 總結

這週跟大家介紹了 [Robot Ignite Academy](http://www.theconstructsim.com/construct-learn-develop-robots-using-ros/robotigniteacademy_learnros/)、[ROS Develop Studio](http://www.theconstructsim.com/rds-ros-development-studio/) 跟 [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)，希望大家學習愉快！

## 延伸閱讀

1. [[ROS tutorial] RTAB-Map in ROS 101](https://www.youtube.com/watch?v=gJz-MWn7jhE&feature=youtu.be)
2. [RTAB-Map 官方 tutorial](http://wiki.ros.org/rtabmap_ros#Tutorials)
