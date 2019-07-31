---
title: 一起來讀 CRAM - A Cognitive Robot Abstract Machine for Everyday Manipulation in Human Environments
date: 2018-06-09 21:54:21
tags:
    - ROS
    - CRAM
    - robotics
author: pojenlai
---

## 前言

很久以前，我就寫過一篇 [CRAM 簡介](https://pojenlai.wordpress.com/2012/12/19/%E7%B0%A1%E4%BB%8B%E4%B8%80%E5%80%8B%E5%BC%B7%E5%A4%A7%E7%9A%84%E5%B7%A5%E5%85%B7-cram/)，但是一直沒有把這個工具用起來，深感可惜。

最近又因緣際會寫了 [一起讀 Knowledge-Enabled Robotic Agents for Shelf Replenishment in Cluttered Retail Environments](https://blog.techbridge.cc/2018/05/19/intro-to-robot-shelf-replenishment/)，所以想要再把 [CRAM 的原始 paper](https://www.open-ease.org/papers/beetz10cram.pdf) 拿起來看一次，然後將我所了解的分享給大家。

## 為什麼發展 CRAM？

CRAM 被發展出來的原因，就是因為機器人在做日常生活中會出現的 manipulation task 時，需要做大量的決策。舉例來說，機器人若要從桌上拿起一本書，他就要決定：

1. 要站在哪邊才拿得到書？
2. 要用哪隻手？
3. 手臂要怎麼靠近書？
4. 要怎麼抓？（是說之前弄了一個 [awesome-grasping](https://github.com/Po-Jen/awesome-grasping)，陸續會再整理有用的東西進去，對這塊有研究的讀者也可以一起來編修）
5. 要出多少力氣來抓？
6. 要怎麼把書拿起來？
7. 要怎麼拿穩？

稀鬆平常到不能再平常的一件任務，仔細去觀察，居然可以包含這麼多的決策，而且每一個都有夠複雜，都可以做很多研究。所以，讓機器人在做事時得以 **依照眼前狀況進行決策**，CRAM 就是為此而生。

## 怎麼用 CRAM 來制定計畫？

首先，我們來看看，若要讓機器人拿起 ?obj 這個物體，讓 CRAM 執行的 plan 會長得如下：

```
(def-goal (achieve (object-in-hand ?obj))
(with-designators
(pickup-place ...)
(grasp-type ...)
(pickup-reaching-traj ...)
(lift-trajectory ...)
(when (and (holds-bel (object-in-hand ?curr-obj) now)
(obj-equal ?curr-obj ?obj))
(succeed (object-in-hand ?obj)))
(at-location pickup-place
(achieve (arm-at pickup-reaching-traj))
(achieve (grasped grasp-type))
(achieve (arm-at lift-trajectory))
(succeed (object-in-hand ?obj))
```

### 特色一 - 用 state 當作目標

在這個範例中，我們希望機器人達到的目標是 (object-in-hand ?obj)，這種做法比起直接把目標定成 (pick-up ?obj) 有一些好處。例如，機器人在接收到目標時，他會先檢查手裡面是不是已經有 ?obj 了，而不是不管三七二十一就直接去想辦法抓起來；此外，如果抓取 ?obj 失敗了，機器人也可以發現並沒有達到 (object-in-hand ?obj)，而不會自以為已經執行完 (pick-up ?obj) 就表示任務已經完成。

當然，我們可以很人工地在 pick-up 這個 action 中加上前檢查跟後檢查，但這樣就不是單純的 pick-up 了，以描述的精確度來說，還是 (object-in-hand ?obj) 較好。

### 特色二 - 用 [first-class object](https://stackoverflow.com/a/245208/1128197) 表示 trajectory、object、location

first-class object 是 paper 裡的原話，其實簡單理解就是 object 啦！舉例來說，在我們計劃中的 pickup-reaching-traj 可以先初始化成下面的樣子（初始化成這些 attribute 不是憑研究者自己的經驗而來，而是會去 query KnowRob 來得到，至於 KnowRob 怎麼得到，那就要去看 KnowRob 的 paper）：

```
(a hand-trajectory
(purpose (pick-up ?obj))
(type motion-plan)
(objective (minimize torque)))
```

這個 object 裡面有幾個 attribute（就像是 object 的 data member），我們可以用這些 attribute 來更清楚描述 pickup-reaching-traj，在這邊就是指定 purpose 是要拿起物體、objective 是要最小化使用的 torque。

厲害的地方來了，在機器人執行任務的過程中，如果機器人看到 ?obj 是杯子，而且裡面有水，那他就可以跟自己說，矮額，我水杯要拿好拿正，才不會被主人罵。所以他就可以加上一個 constraint - (motion-constraint keep-obj-upright)，使 pickup-reaching-traj 變成：

```
(a hand-trajectory
(purpose (pick-up ?obj))
(type motion-plan)
(objective (minimize torque)))
(motion-constraint keep-obj-upright)
```

這種設計讓機器人要完成的動作、看到的物品、所處的地方都能被表示成 first-class object，在實質意義上就是讓機器人對他所在的 location、要操作的 object 跟要執行的 trajectory 都有 **更深的認識**，因為這每一個 first-class object 都可以有許許多多的 attribute（例如 location 還可以加上 (memory where-I-and-EVE-first-met)，有看過瓦力的讀者朋友應該懂 XD）。

如果大家跟我一樣，看到這邊應該會有點霧煞煞，講起來好像很厲害，但是要怎麼做到呢？例如機器人看到有水的杯子，怎麼知道要加上 (motion-constraint keep-obj-upright)，這究竟怎麼實作的？

所以接下來，我們來看看 CRAM 的架構。

## CRAM 的架構

蹦！一開始先上架構圖，不過一開始不容易看懂，沒關係，讓我們繼續看下去。

![CRAM](https://i.imgur.com/o5gUl3v.jpg)

### 重點一 - CPL（CRAM Plan Language）

CRAM 的可執行計畫，就是用 CPL 寫的。CPL 是這篇論文的研究者們自己發展出來的一套 plan language，他們說當時存在的一些方法（例如 [3T architecture](https://pdfs.semanticscholar.org/presentation/37a9/e1df800f6e52d9c940e298e0dab5f273b15e.pdf)），對於 Concurrency、action synchronization、failure handling、loops 跟 reactiveness 的支援都不到位，但這些功能都是機器人應用必須考慮的。

以我目前的程度，要討論 plan language 的設計還太早，所以就讓我們直接來看看 CPL 可以怎麼用吧：

![CPL usage](https://i.imgur.com/daX2H61.jpg)

這個 table 裡面列了一些 CPL 的 control structure，前兩個應該滿簡單就可以理解。__**in parallel do**__ 就是讓機器人可以同時 navigate 跟建地圖；__**try in parallel**__ 就是讓機器人同時用兩種方法去嘗試完成某個任務。

__**with constraining plan**__ p b 的意思就是，執行 primary activity B 並且讓執行過程滿足 constraining plan P。

__**plan**__ 這一類的意思是可以利用 __**order**__ 來指定各個 subplan 執行的順序。

### 重點二 - KnowRob

KnowRob 的核心是 Prolog 寫的，主要就是可以儲存 knowledge、對這些 knowledge 做簡單推理、還有跟 CPL 相接。

跟 CPL 相接的概念也滿簡單的，是用 query-answer 的方式相接。例如 CPL 可以問 KnowRob 說桌上的杯子在哪裡，或可以問說桌上有沒有杯子。

那為何 KnowRob 可以回答這類問題呢？

詳細可以參考 [一起讀 Knowledge-Enabled Robotic Agents for Shelf Replenishment in Cluttered Retail Environments](https://blog.techbridge.cc/2018/05/19/intro-to-robot-shelf-replenishment/) 裡面的 Knowledge 小節。

但是，要回答這麼多種類的問題，僅僅靠一個核心 KnowRob 是不夠的。例如當 CPL 問 KnowRob 桌上有沒有杯子時，KnowRob 必須要有能力辨識物體，並且知道什麼是上下關係。而且，物體辨識的演算法還一直在進步中，所以必須要保有彈性。而 KnowRob 保有彈性的方式也是利用可以跟多個 module 相接的方式，這個對應到架構圖中的KnowRob Extension Modules。

### 重點三 - COGITO

COGITO 在 CRAM kernel 之上又加了一層推理的功能。核心精神是要可以去檢視 CRAM 制定的 plan 好不好、執行是否遇到失敗、怎麼改進等等。

論文裡面對於詳細運作方式沒有著墨太深，僅僅點到：A plan is not only a piece of compiled code that is executed, but also a data structure that can be accessed during run-time or recorded to create a persistent execution trace.

### 回顧

現在，我們有辦法回答剛剛的問題了嗎？

```
機器人看到有水的杯子，
怎麼知道要加上 (motion-constraint keep-obj-upright)，
這究竟怎麼實作的？
```

雖然論文上沒有明講，但我們稍微推敲一下就會知道，一旦機器人辨識出有水的杯子，一調用 KnowRob 儲存的知識，就可以在這個物體的 attribute 加上 (motion-constraint keep-obj-upright)。然後，如果執行任務時，?obj == 剛剛辨識出來的有水杯子，那自然就會把這個 object 裡跟 trajectory 有關的 attribute 加入要執行的 trajectory 條件中。（雖然想起來很簡單，不過沒有接到實作還是覺得虛虛的，之後有機會可以實作看看）

## CRAM 的可擴充性

從 CRAM 的架構圖中，我們可以看到左邊有一排 CPL Extension Modules，基本上就是要提供更多功能，讓 perception、learning、adaptation 等等都做得更好。舉例時間，我們可以搭配一個 module，讓機器人可以辨識 human action。

這個擴充彈性算是一個先留下來的空缺，看你的應用是什麼，就可以擴充自己想要的 module。有興趣更深入了解的話，可以去看 paper 裡面的 VI. COGNITIVE EXTENSIONS。

## 總結

這次跟大家介紹了 CRAM 這篇論文，也算是智慧機器人領域一大厲害的架構，希望可以幫助更多台灣的讀者朋友一起踏入認知機器人的領域。在 Deep Learning、Reinforcement Learning 開始風起雲湧的這個時代，CRAM 這種較從 rule、logic 著手的方法，跟 machine learning based 的方法應該會激盪出新的火花吧。

## 延伸閱讀

1. [A Reactive Plan Language](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.51.7438&rep=rep1&type=pdf)
2. [CRAM tutorials](http://cram-system.org/tutorials)

關於作者：
- [@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
