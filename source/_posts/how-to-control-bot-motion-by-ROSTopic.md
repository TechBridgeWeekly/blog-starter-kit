---
title: 如何用 ROS Topic 控制機器人移動
date: 2016-09-03 22:14:01
tags:
    - ROS
    - Robot
---

筆者今年在 COSCUP 2016 開了一個工作坊，教大家 ROS 的基礎以及 Gazebo 的基本概念，因為是第一次辦工作坊，還在學習怎麼拿捏時間跟內容，自覺並不是講得很完整，所以希望可以再提供一個簡單的範例，讓大家對 topic 傳輸要怎麼應用在控制機器人上會更有概念。

## 在工作坊講的內容 – Node, Topic and Service

<iframe src="//www.slideshare.net/slideshow/embed_code/key/n0KThs66mGONye" width="595" height="485" frameborder="0" marginwidth="0" marginheight="0" scrolling="no" style="border:1px solid #CCC; border-width:1px; margin-bottom:5px; max-width: 100%;" allowfullscreen> </iframe> <div style="margin-bottom:5px"> <strong> <a href="//www.slideshare.net/ssuser54fe9a/coscup-2016-ros-gazebo" title="COSCUP 2016 - ROS + Gazebo機器人模擬器工作坊" target="_blank">COSCUP 2016 - ROS + Gazebo機器人模擬器工作坊</a> </strong> from <strong><a href="//www.slideshare.net/ssuser54fe9a" target="_blank">Po-Jen Lai</a></strong> </div>

這份投影片裡面主要就是帶大家實際操作建立 package、寫 ROS Node、寫 ROS Topic 和 Service 傳輸資料的步驟，讓大家熟悉怎麼撰寫 ROS 程式，並帶到一點點使用 Gazebo 的基本概念，不過比較可惜的是沒有一個範例讓大家實際體會怎麼裡利用投影片裡教的機制來控制機器人，所以補充下面的章節，讓大家可以透過 Topic 簡單地控制機器人移動。


## 如何使用 Turtle_sim 模擬一隻可以接收 geometry_msgs/Twist 型態

接下來，來講一下怎麼透過 Topic 傳輸機制來控制 Turtlesim 裡面的機器人。首先讓我們啟動 turtlesim 的程式，使用下列指令：

```sh
roscore
rosrun turtlesim turtlesim_node
```

啟動 Turtlesim 後，就會看到一個 GUI 介面，裡面有一隻可愛的烏龜。

![turtle_sim_1](/img/pojenlai/turtle_sim_1.png "turtle_sim_1")

```sh
ros@ros-K401UB:~$ rostopic list
/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

接下來就讓我們開始寫程式吧，首先建立一個 package 並開始寫後續的程式：

```sh
cd ~/catkin_ws/src
catkin_create_pkg coscup_follow_up rospy geometry_msgs
cd coscup_follow_up
vim src/circle_turtle.py
```

circle_turtle.py 的程式碼也很簡單，只是單純地送出一個 geometry_msgs/Twist 的 command，所以我們只要定期送出一個 command 就可以讓這隻可愛的烏龜移動了。

```py
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
 
def circle_walker():
 pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
 rospy.init_node('cmd_publisher_node')
 rate = rospy.Rate(10) # 10hz
 
cmd = Twist()
 count = 0;
 
while not rospy.is_shutdown():
 cmd.linear.x = 1
 cmd.angular.z = 0.5
 
pub.publish(cmd)
 rate.sleep()
 
if __name__ == '__main__':
 try:
 circle_walker()
 except rospy.ROSInterruptException:
 pass
 ```

 最後只要把程式跑起來，就可以看到結果了。

 ```sh 
chmod +x src/circle_turtle.py
cd ../..
catkin_make
source devel/setup.bash
rosrun coscup_follow_up circle_turtle.py
```

![turtle_sim_2](/img/pojenlai/turtle_sim_2.png "turtle_sim_2")

這邊要介紹一個簡單的機器人模型，也就是一般的兩輪機器人模型。在這種模型底下，其實你只能夠控制機器人的前後移動以及左右旋轉，所以程式中就只有指定 linear.x 跟 angular.z 的值 (因為就算其他值不給 0，也不會有任何反應)。

## 總結

這篇文章算是將 COSCUP 工作坊的內容再稍微延伸一下，讓大家比較有見樹又見林的感覺，不會寫了一堆 node ，卻都還沒有碰到機器人，雖然內容對於已經熟悉 ROS 的讀者不難，但希望透過這篇文章可以幫助更多想上手的讀者降低學習的難度。


關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人跟電腦視覺有少許研究，最近在鍛鍊自己的執行力
