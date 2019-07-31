---
title: 一起讀 Knowledge-Enabled Robotic Agents for Shelf Replenishment in Cluttered Retail Environments
date: 2018-05-19 22:44:03
tags:
    - CRAM
    - KnowRob
    - RoboSherlock
    - Cognitive Robot
author: pojenlai
---

## 前言

今天來跟大家一起看一篇論文，可以讓機器人將貨架整理成指定的樣子，算是比較進階的機器人應用。大家可以先看個影片，對整個應用會比較有概念。

[![Demo](https://img.youtube.com/vi/xFwinZAHrnA/0.jpg)](https://www.youtube.com/watch?v=xFwinZAHrnA)

## 系統 overview

有 overview，就比較能夠繼續往下看。所以首先讓我們來看看整個系統架構：

![overview](https://i.imgur.com/dBdVbjK.jpg)

整個系統的運作流程如下：

1. CRAM 先產生最初的任務 - 重新整理貨架，整理成跟 KNOWROB 中儲存的樣貌一樣
2. CRAM 向 ROBOSHERLOCK 提出 query ，以偵測有哪些物品
3. ROBOSHERLOCK 調出儲存在 KNOWROB 裡的 semantic map，了解只需要偵測貨架上的物品即可
4. 偵測出有哪些物品後，回傳給 CRAM
5. CRAM 規劃出一個可以把貨架整理好的 plan
6. 執行剛剛規劃好的 plan（若中途失敗，就重新規劃）

這一系列的步驟中，有三個極為重要的黑盒子 - CRAM、KNOWROB 跟 ROBOSHERLOCK。接下來我們會分別簡介。

## Perception

Perception 這個功能主要是交給 ROBOSHERLOCK 這個黑盒子，他比較特別的地方在於，他是把 perception 這個任務用 query-answer 的角度來切入。

什麼意思呢？

舉例來說，我們可以送給 ROBOSHERLOCK 下面這種 query：

```
(detect (an object
            (category container)
            (capacity (≥ 1 liter))))
```

ROBOSHERLOCK 就會想辦法回傳給你大於 1 公升的容器。看到這邊是不是覺得很酷呢？因為這種 perception 的機制有點像是人類要完成一件事情時，心中會有的想法。例如我們如果要念書，我們就會想「我的課本在哪裡呢
？」，然後就會有相對應的知識讓我們可以正確地知道課本在哪裡。

而之所以能夠回答這種問題，在於 ROBOSHERLOCK 在設計之初就將 knowledge 跟物體綁在一起，可以看看下面這張圖，裡面的每個物體都有一些相對應的 knowledge - color, shape, text, size 等等，這些 knowledge 是利用 KNOWROB 這個 knowledge base 來儲存，看完下一個部分就會知道怎麼儲存。

![ROBOSHERLOCK](https://i.imgur.com/v9FO59r.jpg)

## Knowledge

Knowledge 相關的東西，都是儲存在 KNOWROB 裡面。一言以蔽之，KNOWROB 就只是一個 knowledge base，只是這個 knowledge base 裡面的知識，都是用 [Web Ontology Language (OWL)](https://en.wikipedia.org/wiki/Web_Ontology_Language) 來儲存。

用 OWL 儲存的好處是，所有的知識都可以用 [description logic](http://www.wikiwand.com/zh-tw/%E6%8F%8F%E8%BF%B0%E9%80%BB%E8%BE%91) 來表示，比如說

![DL](https://i.imgur.com/TwOTVTa.jpg)

換句話說，我們可以用嚴謹的邏輯來儲存類別的知識（男人、人），可以儲存個體的知識（張三）。這種知識表達的方法不僅僅是儲存了機器人所需要用到的知識，也讓機器人易於使用。舉個例子：

- 瓶子是容器
- 桌上的瓶子.size: medium

假設 medium 大於 1 公升，那當 ROBOSHERLOCK 被 query 要找大於 1 公升的容器時，就可以利用 KNOWROB 的知識，很容易推論出目前辨識出的 medium size bottle 符合需求。

而實際上，機器人要做的整個任務都是可以用知識來表示的，看看這張圖就很清楚了：

![knowrob](https://i.imgur.com/w57Qpd7.jpg)

## Plan and Execution

有了 perception 的能力，也有相關的 knowledge 可以使用，接下來要進場的就是實際規劃可以把貨架收拾好的計畫，還有執行。這個就交給 CRAM 來處理。

CRAM 的核心精神是為了讓機器人可以更有彈性地完成任務，如果想看稍微詳細一點的介紹，可以參考這篇 [CRAM 簡介](https://pojenlai.wordpress.com/2012/12/19/%E7%B0%A1%E4%BB%8B%E4%B8%80%E5%80%8B%E5%BC%B7%E5%A4%A7%E7%9A%84%E5%B7%A5%E5%85%B7-cram/)。

因為 CRAM 的設計有跟 manipulation 等 module 相接，所以可以用來執行任務，可以參考這個架構圖：

![CRAM](https://i.imgur.com/o5gUl3v.jpg)

## 總結

今天跟大家一起簡單地看過了這篇很 high level 的論文，因為裡面用到的三大黑盒子都太高級了，其實只看這篇論文沒有辦法對整個系統有透徹的了解。不過，機器人這麼複雜的系統，有一篇較大的 paper，這篇 paper 再包含一些厲害的 paper 也不為過吧。

## 延伸閱讀

1. [CRAM Tutorials](http://cram-system.org/tutorials)
2. [KnowRob 2.0 — A 2nd Generation Knowledge Processing Framework for Cognition-enabled Robotic Agents](https://ai.uni-bremen.de/papers/beetz18knowrob.pdf)
3. [ROBOSHERLOCK: Unstructured Information Processing for Robot Perception](https://pdfs.semanticscholar.org/ca4a/1ece9ba958040266199d739c351ad041a0bf.pdf)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
