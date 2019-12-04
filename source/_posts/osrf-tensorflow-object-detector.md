---
title: 一起來玩 OSRF 的 TensorFlow Object Detector
date: 2018-02-24 21:56:33
tags:
    - ROS
    - TensorFlow
    - Object Recognition
author: pojenlai
---

## 前言

上次我們一起 [使用 TensorFlow 來做簡單的手寫數字辨識](https://blog.techbridge.cc/2018/01/27/tensorflow-mnist/)，雖然我們可以自己使用這些 API 兜自己的模組，但我們也可以利用現成的工具，節省開發時間。

## 安裝

安裝步驟可以參考官方的 [README文件](https://github.com/osrf/tensorflow_object_detector/blob/master/README.md)。我自己是在 Ubuntu 14.04 + Indigo 測試過，也 OK，只要記得將一些安裝 kinetic package 的地方更新成 indigo 就好。例如第二步的 camera dependencies：

```
sudo apt-get install ros-indigo-usb-cam ros-indigo-openni2-launch
```

## 真實環境測試

安裝成功之後，立刻就來試試看真實環境下的測試結果，執行 launch 檔之後，應該就可以看到跳出如下視窗，顯示出辨識的物體。

![image](/img/pojenlai/obj_detector.png)

我們都知道，要做 object recognition 有很多種不同的 neural network 可以用，這個 package 預設是使用 [Single Shot Detector](https://github.com/weiliu89/caffe/tree/ssd)，SSD 主要的優點在於速度，根據他們在 PASCAL VOC、MS COCO、ILSVRC 等 Dataset 測試的結果，SSD 的準確度跟 region proposals 的方法差不多，但速度快很多。詳細介紹可以參考[這篇文章](http://blog.csdn.net/u011534057/article/details/52733686)。

## 換一個不同的 model 

接下來讓我們來嘗試換換，其實只要編輯 detect_ros.py 裡面的 MODEL_NAME 就好。首先，我們先到 [tensorflow 的 detection model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md)，我下載了 ssd_inception_v2_coco 的 tar.gz 檔。把裡面的 model 解壓縮到 tensorflow_object_detector/data/models 後，再去改 detect_ros.py 裡面的 MODEL_NAME：

```python
# What model to use
#MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'
MODEL_NAME = 'ssd_inception_v2_coco_2017_11_17'
```

![change model](/img/pojenlai/change_model.png)

因為筆者在換 model 的時候，已經離拍上圖好一小陣子，所以上圖的香蕉已經被筆者吃掉，場景也變了。

那如果你是想要更新辨識物體的種類，那就更新 detect_ros.py 裡面的 LABEL_NAME，這就不贅述了。

## 總結

今天我們玩了一個 OSRF 的 Object Detector，如果有想要做自己的機器人應用，需要用到物體辨識功能，不妨就直接拿這個 package 來用用，就算你覺得現有的 model 都不太好用，想自己重新 train 一個或 finetune 一個來符合自己的使用場景，也還是可以利用這個 package，抽換掉 model 就好，可以省下不少時間。

## 延伸閱讀

1. [為什麼 SSD(Single Shot MultiBox Detector) 對小目標的檢測效果不好？](https://www.zhihu.com/question/49455386)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
