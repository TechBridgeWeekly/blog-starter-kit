---
title: ROS SMACH 簡介
date: 2017-02-18 21:52:22
tags: 機器人, Robot, ROS, SMACH, Finite State Machine
author: pojenlai
---



## 前言

這次想跟大家介紹一個好用的工具 - SMACH (唸法同 smash)。SMACH 的存在意義是為了讓大家可以用系統化的方式來撰寫機器人的行為。這個系統化的方式是 Finite State Machine，概念上跟之前介紹過的 ecto 的 graph 概念有點相似。

簡單來說，就是讓我們可以很方便地定義機器人的各種狀態，所以機器人就不再只是根據程式碼裡面寫定的一連串行為去行動，而可以根據各種不同的條件進行狀態的切換。舉例來說，一個掃地機器人平時處於掃地狀態時，他要做的事情就是移動跟吸塵，而當機器人看到障礙物的時候，就可以切換到避開障礙物的狀態，等到障礙物消失的時候，再切換到掃地狀態。

想要對 SMACH 有更具體的概念嗎? 可以看看這個影片:

[![video](https://img.youtube.com/vi/F3slROzVNbc/0.jpg)](https://www.youtube.com/watch?v=F3slROzVNbc)

雖然 SMACH 跟 ROS 可以一併使用，他的核心其實是一個跟 ROS 沒有關係的 Python library，是透過 smach_ros 來跟 ROS 的 topics, services 跟 actions 串接。

## 使用 SMACH 的幾大好處

我們雖然知道 Finite State Machine 可以建立由狀態來決定行為的程式，但除了這個之外還有幾個特點，讓我們可以在適當的時候使用 SMACH。

* Task Hierarchy: state 可以由更多小 state 組成，建立複雜的行為
* Task Priority: state 之間可以有優先權，有高優先權的 state 可以中斷低優先權的 state 
* Concurrency: 如果想要同時執行多項行為，也可以用 SMACH 來實現

## SMACH 基本小程式

這邊我們用個簡單的範例來介紹要怎麼創造一個 state machine，首先呢，最重要的東西就是 state，在 SMACH 的語法規定中，每個 state 用一個 class 來寫，其中要包含 init 和 execute 兩個函式，實際執行的行為會被寫在 execute 函式裡面。

當每個 state 的行為都被定義好之後，剩下要寫的就只需要在 main 中，將各個 state 加入 SMACH 的 container (其實 state machine 就只是一個包含很多 state 的 container 而已) 就好。

在下面的範例中，只有兩個簡單的 state - FOO 和 BAR，他們的關係如下圖:

![img](/img/pojenlai/simple_smach.png)

```python
#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'
        
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
```

## 總結

今天跟大家簡介了 SMACH 這個 package，也介紹了要怎麼寫一個非常小的 state machine，SMACH 可以做到相當複雜的行為，如果有興趣的話，可以去看看延伸閱讀中提供的教學，裡面有相當完整詳細的說明。

其實要撰寫比較複雜的機器人行為，除了 finite state machine 之外，behavior tree 跟 teer 都是常見的選擇，根據應用的不同可以選擇自己想要的方法。連結可以在延伸閱讀的地方找到喔！

## 延伸閱讀

1. [SMACH Tutorials](http://wiki.ros.org/smach/Tutorials)
2. [ROS behavior tree package](http://wiki.ros.org/behavior_tree)
3. [ROS teer package](http://wiki.ros.org/executive_teer)
