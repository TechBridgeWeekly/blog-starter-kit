---
title: 一起來讀論文 - Robot Learning via Human Adversarial Games
date: 2019-04-12 16:51:34
tags:
    - Human-Robot Interaction
    - Deep Learning
    - Computer Vision
    - Grasping
author: pojenlai
---

## 前言

這次要跟大家介紹一篇論文 - Robot Learning via Human Adversarial Games。這篇論文主要想研究的問題是，在機器人透過 Reinforcement Learning 學習抓東西的過程中，如果有人類來干擾機器人、試圖讓機器人抓到的東西被弄掉，是不是有辦法讓機器人學習得更有效率？

![img](https://i.imgur.com/PsZiva5.jpg)

這篇論文透過一個簡單的實驗場景，透過讓一些測試者參與實驗，證明人類的干擾可以讓機器人學習得更好，就讓我們繼續看下去。

## 過去相關研究

機器人用 Deep Reinforcement Learning 學習抓東西是這幾年才比較火熱的研究領域，之前有一些研究 - [Supersizing Self-supervision: Learning to Grasp from 50K Tries and 700 Robot Hours
](https://arxiv.org/pdf/1509.06825.pdf)是用很長的時間，讓機器人大量學習，做到可以抓取東西的能力。

但這種方法非常曠日廢時，所以有一些後續的研究 - [Supervision via Competition: Robot Adversaries for Learning Tasks](https://arxiv.org/pdf/1610.01685.pdf) 就開始探討如果讓機器人 A 去嘗試干擾機器人 B 的學習過程，是不是可以讓機器人 B 學習得更好。結果還真的可以。

所以這篇論文就再進一步探究，如果對於抓取都沒什麼概念的機器人 A 去干擾都可以幫助機器人 B 學習得更好，那用擁有許多物理知識的人類取代機器人 A 來干擾，是不是能夠更有效率？

雖然乍看之下覺得干擾這件事有點奇怪，但如果再好好思考一下，就會覺得有道理，因為在現實生活中，人類本來也就不一定一直是協助機器人的角色，有時候可能也會不小心影響到機器人的工作。而且如果這些干擾會讓機器人要做的工作失敗，剛好就可以是最佳的教材，因為機器人就知道原本他那樣做可能不夠 robust，就會再調整自己的策略。

## 方法介紹

### 問題描述

![img](https://i.imgur.com/KFX5XdV.jpg)

這篇論文把他們的問題表示成一個 two-player game，其中一個玩家是機器人，另一個玩家是人類，一開始整個遊戲的狀態是 $S$，機器人做了一個 action $a^R$ 後，會讓遊戲狀態變成 $S^+$，然後人類會再做一個 action $a^H$，讓遊戲狀態變成 $ S^{++} $。

然後把 reward 設定成

![img](https://i.imgur.com/Rddgu3B.jpg)

就可以將問題變成，機器人要想辦法學習到能最大化 reward 的策略：

![img](https://i.imgur.com/U2GVMpl.jpg)

### 學習演算法

這篇論文的核心演算法就在下面的 pseudo code 中：

![img](https://i.imgur.com/blItVZp.jpg)

主要概念都是，讓機器人先做一個 action，然後再觀察人類做完干擾的 action 之後會讓 reward 變成怎麼樣，進而學習到能讓人類干擾最小化的演算法。

### 如何判斷要抓哪邊

他們使用一個簡單的 end-to-end convolutional neural network 來判斷：

![img](https://i.imgur.com/3eZrplB.jpg)

一開始，機器人會用 camera 得到一張影像，然後從這張影像中 sample $N_g$ 張比較小的影像輸入上圖的 network，然後對於每張比較小的影像，會再產生出 $N_A$ 個角度的抓取分數，然後機器人就可以從 $N_g \times N_A$ 個可能的抓取位置和角度中選出最高分的。

## 測試環境

測試的環境是在 Mujoco，不過他們有自己修改一下環境，讓 Mujoco 可以接受人類的 input。

他們使用了 5 種不同的物體，讓機器人先抓取後，人類受試者可以提供六個方向的力(上、下、左、右、前、後)來嘗試讓機器人抓到的物體掉落，透過這個簡單的 task 來驗證機器人能否在有人類干擾的情況下學習得更好。

## 實驗結果

論文裡面有提供詳細的測試結果和受試者的問卷調查結果，有興趣的讀者可以自己去閱讀，不過從下面的圖片，應該可以很明顯地看出這篇論文提出來的學習方法是成功的：

第一排的圖片是呈現機器人在人類干擾前，自己學到的抓取位置(紅色的 bar 是指機器人的夾爪張開時的位置，黃色點是夾爪關起來時的位置)。第二排則是在人類干擾後學習到的抓取位置。

![img](https://i.imgur.com/MqbHUTE.jpg)

## 總結

今天跟大家分享了 Human Robot Interaction 領域中的一個有趣方向，隨著這些研究的持續進展，希望未來有一天機器人能夠大幅度地幫助人類生活！

## 延伸閱讀

1. [Robot Learning via Human Adversarial Games 的測試環境 code](https://github.com/davidsonic/Interactive-mujoco_py)
