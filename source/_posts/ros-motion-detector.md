---
title: 接收來自 ROS Topic 的影像並偵測畫面中的動作
date: 2016-11-26 20:10:08
tags: 機器人, ROS, 動作偵測, OpenCV, Mixture of Gaussian
author: pojenlai
---

## 前言

這次來帶大家玩個基礎的應用 - 動作偵測,用自己筆電的相機再加上 OpenCV 的 API，就可以做到動作偵測並把在動的地方框起來。可以透過這篇學習怎麼使用 ROS Topic 來接收影像並做後續的處理。

## 開 package 寫程式

首先來開一個 motion_detector package:

```
catkin_create_pkg motion_detector rospy sensor_msgs cv_bridge usb_cam\
cd motion_detector
vim src/motion_detector.py
```

程式碼長這樣，裡面主要是用到 OpenCV 的 [MOG2](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_bg_subtraction/py_bg_subtraction.html#backgroundsubtractormog2) 這個前背景分類的工具，切出來的前景就用一個框框來表示。

```python
#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

kernel_elliptic_7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
kernel_elliptic_15 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
area_threshold = 2000

class MOG2:
  def __init__(self):
    self.fgbg = cv2.BackgroundSubtractorMOG2(history=150, varThreshold=500, bShadowDetection=True)

  def detect(self,image):
    fgmask = self.fgbg.apply(image)

    cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel_elliptic_7, dst=fgmask)
    cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel_elliptic_15, dst=fgmask)

    contours = cv2.findContours(fgmask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area_box = ((cv2.contourArea(contour), cv2.boundingRect(contour)) for contour in contours[0])
    area_box = [(area, box) for (area, box) in area_box if area > area_threshold]
    area_box.sort(reverse=True)

    bounding_boxes = [((x, y), (x+w, y+h)) for _, (x, y, w, h) in area_box[:5]]
    for p1, p2 in bounding_boxes:
        cv2.rectangle(image, p1, p2, (0, 255, 0), 2)

    return image
    #return fgmask #for param tuning

class Motion:
    def __init__(self):
        rospy.init_node("motion_detector_node")
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('camera/visible/image', Image, queue_size=2)
        rospy.Subscriber("usb_cam/image_raw", Image, self.imageCallback)
        self.motion_detector = MOG2()
        rospy.spin()

    def imageCallback(self, image):
        if self.motion_detector:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            result_img = self.motion_detector.detect(cv_image)
            image = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
            #image = self.bridge.cv2_to_imgmsg(result_img, "mono8") #for param tuning

        self.pub.publish(image)

if __name__ == '__main__':
    detector = Motion()
```

## 執行程式

先把程式變成可執行的 node，接着來寫個 launch file 方便執行：

```
chmod +x src/motion_detector.py
mkdir launch
vim launch/motion_detection.launch
```

```xml
<launch>

  <!-- Launch the motion detector node for image processing. -->
  <node pkg="motion_detector" name="MotionDetector" type="motion_detector.py"/>

  <!-- Launch the driver node for our usb camera. -->
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
   <param name="video_device" value="/dev/video0" />
   <param name="image_width" value="640" />
   <param name="image_height" value="480" />
   <param name="pixel_format" value="yuyv" />
   <param name="camera_frame_id" value="usb_cam" />
   <param name="io_method" value="mmap"/>
  </node>

  <node name="rviz" pkg="rviz" type="rviz" required="true" 
      args="-d $(find motion_detector)/rviz/motion.rviz" />
</launch>
```

這邊想順便教大家一個技巧，你可以把 rviz 的檔案先寫好，這樣每次重新啓動 Rviz 就很方便，不用重新選想要看的資料類型。

```
vim rviz/motion.rviz
```

其實這個檔案可以直接從 Rviz 存出來，或是你也可以很 hardcore 地自己寫XD

```xml
Panels:
  - Class: rviz/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Image1
      Splitter Ratio: 0.5
    Tree Height: 387
  - Class: rviz/Selection
    Name: Selection
  - Class: rviz/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.588679
  - Class: rviz/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: Image
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz/Image
      Enabled: true
      Image Topic: /camera/visible/image
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Image
      Normalize Range: true
      Queue Size: 2
      Transport Hint: raw
      Unreliable: false
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
    - Class: rviz/SetInitialPose
      Topic: /initialpose
    - Class: rviz/SetGoal
      Topic: /move_base_simple/goal
    - Class: rviz/PublishPoint
      Single click: true
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.785398
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 668
  Hide Left Dock: false
  Hide Right Dock: false
  Image:
    collapsed: false
  QMainWindow State: 000000ff00000000fd00000004000000000000016a00000212fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000006400fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000002800000212000000dd00fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f00000212fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000002800000212000000b000fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b2000000000000000000000002000004b0000000a9fc0100000002fb0000000a0049006d006100670065030000014b0000009700000287000001a3fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004b00000003efc0100000002fb0000000800540069006d00650100000000000004b0000002f600fffffffb0000000800540069006d006501000000000000045000000000000000000000022b0000021200000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 55
  Y: 14
```

## 執行結果

```
roslaunch motion_detector motion_detection.launch
```

用上面的指令跑起來之後，就可以看到在動的東西被框框圈起來啦！

![1](/img/pojenlai/motion_detector_rviz.png)

## 延伸閱讀

1. [ROS 跟 OpenCV 串接的工具 - cv_bridge](http://wiki.ros.org/cv_bridge)
2. [用 Optical Flow 來偵測動作](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_lucas_kanade/py_lucas_kanade.html#lucas-kanade)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人跟電腦視覺有少許研究，最近在鍛鍊自己的執行力
