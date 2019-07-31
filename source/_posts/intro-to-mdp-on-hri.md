---
title: 如何用 Markov Decision Process 描述 Human-Robot Interaction 問題
date: 2018-11-24 14:31:45
tags:
    - Human-Robot Interaction
    - Markov Decision Process
author: pojenlai
---

## 前言

上次我們介紹了 [Markov Decision Process 的基本概念](https://blog.techbridge.cc/2018/10/27/intro-to-mdp-and-app/)，雖然上次有簡單提到 MDP 的應用，但並不是很詳細，所以今天我們想嘗試把 MDP 跟 [Human-Robot Interaction](https://github.com/Po-Jen/awesome-human-robot-interaction) 問題連結起來，一起來看怎麼從一個簡單的 insight 出發，加上 MDP 的數學基礎，最後形成一篇嚴謹的論文 ( Human-Robot Interactive Planning using Cross-Training: A Human Team Training Approach ) 吧！

## 這篇論文要講什麼

Human-Robot Interaction 是一個探討機器人跟人如何互動的學科，而目前最常討論的議題之一就是，如何讓機器人和人可以更有效率地一起完成工作。這篇論文嘗試讓機器人跟人進行 job rotation，讓彼此都處在對方的處境中，進而讓合作的效率提升。

## 這篇 paper 是源自於什麼 insight？

Insight 1：同一個 team 裡面的成員，處理事情的方法（mental model）越接近，他們越容易理解彼此的需要，進而提升團隊的效率。

![img](https://i.imgur.com/P0AzJ5N.jpg)

Insight 2: 進行 Cross-Training，也就是讓機器人和人輪流交換工作，讓他們都去做對方會做的事，可以讓他們的 mental model 更接近。

![img](https://i.imgur.com/z1LqDGZ.jpg)

## 這個 insight 怎麼用數學描述？

看了上面的 insight，感覺頗符合直覺的，因為我們可以想像自己跟其他人合作，如果可以做做看對方的工作，就更容易體會對方做事情所感受到的需求，在合作時也更容易滿足對方的需求。

但問題是，這樣的直覺怎麼跟數學接起來？

這時候，就要用到我們上週學習的 [MDP](https://blog.techbridge.cc/2018/10/27/intro-to-mdp-and-app/) 了，首先，這篇論文假設，人類的 mental model（或說白話一點，行為的偏好）可以用 MDP 來描述。

1. **MDP 跟欲描述問題的初步關係**

若我們仔細看 MDP 裡面的所有變數，真正需要我們煩惱的有兩件事 - reward 要怎麼定義？Transition probabilities 要怎麼定義？

這兩個地方就是這篇論文要動手腳的地方了。首先，作者假設機器人已經有一點 prior knowledge，也就是有第一版的 reward 跟 transition probabilities，於是就可以用 value iteration 的演算法算出第一版機器人的 policy。

這一版 policy 可以想成是機器人原本認為該怎麼完成任務的方法。但可能跟人類夥伴的工作方法不一樣，所以就需要來做 cross training。

2. **怎麼把 cross training 跟 MDP 接起來**

接下來就是重點，我們先假設，機器人跟人是一起要鎖螺絲：
![img](https://i.imgur.com/Rgzyp1U.jpg)
人負責放螺絲到三個要鎖的位置，機器人負責把螺絲鎖緊。
那 Cross training 有兩個階段，首先是 forward phase，也就是機器人跟人都負責自己該做的地方；接著是 rotation phase，也就是機器人跟人互換，這時人要負責鎖螺絲，機器人負責放螺絲。

於是，雖然一開始機器人有第一版 policy，但機器人在 forward phase 可以觀察，從某個 state s，自己做完某個 action 後，人會讓 state 變成怎麼樣的 s'（這邊用 s' 表示經過機器人和人的 action 後，state s 的下一個 state）。然後就用這樣的狀態變化來更新 MDP 的 transition function。
接著，在 rotation phase，這時機器人是要擺放螺絲，所以就可以由人來給予機器人 reward 的 feedback，讓機器人可以更新自己對各 state 的 reward 認知，所以就可以算出新的、較接近人類夥伴的 policy。（大家應該可以看出，即使不同人的 policy 偏好不同，機器人都可以透過這套演算法，去適應自己夥伴的偏好）

## 這篇論文的限制

1. 首先，人類的 mental model 相當複雜，用 MDP 來 model 有點像是把人類的偏好用 transition probabilities 跟 reward 表示，但這樣比較像是知其然不知其所以然。就像我們如果跟其他人合作，我們可以去猜想、甚至理解為什麼夥伴會有他的偏好。但用 MDP 顯然沒有這樣的描述能力。

2. 如果今天要讓機器人融入人類社會，要做更多的事，這樣的 model 能夠更 general 嗎？目前的做法看起來是，一旦要做的事情變得很多，就得訓練很多套 MDP。

## 這篇論文的貢獻

雖然上面講了一些限制，但這篇論文其實是很讚，110 次的引用數不是掛著好看的。

1. 跳脫用單純 Reinforcement Learning 的方式來學習跟人合作，而是將合作問題巧妙地用 MDP 來表示人類 mental model，然後用 cross training 來改進合作的效率。

2. 用嚴謹的數學方法精準地描述這過程，雖然若仔細看 paper，我們還是會看到不少假設，讓推導可以進行下去，但已經算是相對頗嚴謹的 paper 了。

## 我這篇介紹沒 cover 到的問題

1. Transition probabilities 在上周介紹的 MDP 定義中，提到 MDP
是用來處理當採取一個 action，因為環境中有一些不確定的因素，讓我們採取完 action 會到達的 state s' 有多種可能的問題。那這篇論文的意思是要機器人把人類 action 當做一個環境中的變因，然後機器人認知的 transition probabilities 盡量往人類的偏好靠近？（因為從 current state s 開始，機器人採取某個 action，人類也採取某個 action，讓 state 變成 s'）

2. 實作上要怎麼定義 state 有多少？action 又有多少？

## 總結

今天跟大家介紹了一篇論文，來講怎麼使用 MDP 到 HRI 的問題上面，我覺得這個領域很有趣，也很值得投入，希望對想要入門的朋友有幫助。

## 延伸閱讀

1. [Mathematical Models of Adaptation In Human-Robot Collaboration](http://stefanosnikolaidis.net/papers/snikol_review_2017.pdf)
2. [Github repository: awesome-human-robot-interaction](https://github.com/Po-Jen/awesome-human-robot-interaction)


關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
