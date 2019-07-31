---
title: 使用 Gazebo 模擬器控制機器人建立 2D 地圖
date: 2016-08-06 22:18:57
tags:
  - 機器人
  - Robot
  - ROS
  - Gazebo
author: pojenlai
---

# 前言

ROS 很吸引人的一個地方在於，他跟模擬器 Gazebo 很好地結合在一起，讓使用者可以使用一台筆記型電腦就開始撰寫機器人程式，而且可以在筆電上看到程式執行的結果。模擬器裡的機器人不會撞壞、不會沒電，所以就算在程式設計過程中有錯誤，也不會有任何硬體成本的損失 (時間當然是會損失)，而且很容易大量測試演算法，因為可以在模擬器中設置各種場景，而不需要一個很大的空間和昂貴的機器人。這篇文章會簡單介紹怎麼在 Gazebo 裡面啟動 PR2，並遙控他來建立一個 2D 的地圖。

# 啟動 Gazebo 跟 PR2

我的測試環境是 Ubuntu 14.04 + ROS Indigo，不過使用的指令都很 general，即便你使用其他版本，應該也不會有什麼問題。

首先我們確定已經安裝所需要的套件：

```
sudo apt-get install ros-indigo-gazebo-*
```

接下來在終端機輸入 gazebo 應該就可以成功啟動 Gazebo 模擬器。然後我們可以用現成的 package 啟動一個有 PR2 的空白模擬環境。

```
roslaunch pr2_gazebo pr2_empty_world.launch
```

Gazebo 裡面已經有很多現成的模型，選擇左邊的 Insert 頁籤，裡面就可以找到一些現成的模型，在這邊我先加入了 Cube 跟 Dumpster 的模型，加入後的結果如下圖：

![gazebo](/img/pojenlai/gazebo1.png)

對於機器人來說，最重要的第一個環節就是感知功能，他必須要可以感知環境中的資訊，才能做出相對應的動作。所以我們會先關心的就是 Gazebo 裡面的 PR2 機器人可不可以接收到環境中的資訊。

答案當然是可以的，我們可以透過 ROS 提供的 visualizer Rviz 來觀看 PR2 接收到的資訊，如果是已經熟悉 ROS 的讀者，想必對 Rviz 不陌生，只要開一個新的終端機，輸入 rviz 就可以啟動了。啟動之後，可以 Add 一個 Image 的顯示框，然後選擇現在可以看到的 topic，就可以看到 Gazebo 裡面的 PR2 看到的畫面。

![gazebo](/img/pojenlai/gazebo2.png)

# 啟動 gmapping package 建地圖

環境都已經啟動了，接下來就要開始建地圖啦! 這邊我們使用的工具是 gmapping ，首先我們在隨便一個路徑新增一個 launch 檔 –  vim pr2_build_map.launch 。

然後輸入這一串文字，儲存檔案後離開。

```
<launch>

  <!-- dynamic map generation -->
  <node name="gmapping_node" pkg="gmapping" type="slam_gmapping" respawn="false" >
    <remap to="base_scan" from="scan"/>
    <param name="odom_frame" value="odom_combined" />
  </node>

</launch>
```

然後可以用 roslaunch pr2_build_map.launch啟動。
啟動 gmapping 之後，我們還是透過 Rviz 來觀看地圖建立的狀態，這邊需要加入 Map 的顯示，然後 topic 選擇 /map 就好。地圖中的黑色線就是障礙物，淺灰色區域表示是可以走的無障礙物區域，深灰色則是未知區域。

![gazebo](/img/pojenlai/gazebo3.png)


讓我們近看一下 map 的 topic 要怎麼選：

![gazebo](/img/pojenlai/gazebo4.png)


但如果機器人不能移動的話，就只能顯示眼前看到的這一塊地圖。所以我們先使用簡單的遙控程式來控制 Gazebo 裡面的 PR2 移動。

這邊只要啟動 teleop_keyborad 就可以用鍵盤控制機器人移動：

```
roslaunch pr2_teleop teleop_keyboard.launch
```

WASD 四個按鍵分別代表前左後右四個方向的平移、QE兩個按鍵是原地旋轉，這邊要注意必須選到啟動 teleop_keyboard的視窗，按按鍵才有用。使用 teleop 來控制 PR2 走一走之後，就可以看得出我們已經使用 gmapping 建立了一個看起來有模有樣的地圖了。

![gazebo](/img/pojenlai/gazebo5.png)

如果你想要把地圖存起來，提供未來要做 Navigation 使用，或只是單純想留個紀念，可以用一個指令把地圖存起來 – rosrun map_server map_saver 。


# 總結

這篇文章跟大家簡單介紹怎麼使用 Gazebo 模擬器來模擬機器人，並利用他實際操作了一個小小的 SLAM 功能，也建出了自己的一張地圖。我們能利用 Gazebo 來做的事情當然遠不止於此，不過時間有限，我們下次見。

# 延伸閱讀
1. [gmapping 演算法原理簡介](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa11/slides/gmapping.pdf)
2. [SDF 網站中文版](http://sdformat.cn/)


關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人跟電腦視覺有少許研究，最近在鍛鍊自己的執行力
