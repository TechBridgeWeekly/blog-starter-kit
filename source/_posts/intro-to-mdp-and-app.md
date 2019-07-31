---
title: 簡介 Markov Decision Process 與其應用
date: 2018-10-27 22:37:56
tags:
    - Markov Decision Process
    - Reinforcement Learning
author: pojenlai
---

## 前言

今天想跟大家介紹一個在 Reinforcement Learning 中相當重要的基礎 - Markov Decision Process（MDP）。比起一般的 search，MDP 能夠 model 更複雜的問題，今天就讓我們來介紹 MDP 的基礎觀念，還有他的應用跟限制。

## MDP 想要 model 什麼問題？

MDP 想要處理的問題是：當你採取的 action 不會完全如你所想的達到你想要的 state，你該如何採取 action？

舉例來說（這例子出自延伸閱讀 1 裡面的 MDP），假設有一隻機器人，在一個迷宮裡面，他從最左下角出發，要走到終點（+1 or -1 都是終點），而且要盡量讓 reward 最大：

![img](https://i.imgur.com/1dgFbSg.jpg)
(This image comes from [UC Berkeley CS188 Lecture 8](https://www.youtube.com/watch?feature=player_embedded&v=wKx4MuLfe0M))

但當這隻機器人想要往上走，他只有 80% 的機率會真的走到上面那一格，另外有 10% 的機率會走到右邊那格、10% 的機率會走到左邊那格。

![img](https://i.imgur.com/KQgzThC.jpg)
(This image comes from [UC Berkeley CS188 Lecture 8](https://www.youtube.com/watch?feature=player_embedded&v=wKx4MuLfe0M))

這時候就尷尬了，因為當這隻機器人還沒實際開始走，他其實不知道接下來會發生什麼事。他不知道當他要往上走，會不會真正往上。

這種問題，我們稱做 non-deterministic search problem，也就是你無法事先做好所有規劃，因為你採取某個 action，無法 determine 會產生什麼結果。

在這種情況下，機器人就不能只是用一些搜索的演算法，得到 "上上右右右" 這種可以無腦照做的步驟，因為只要其中有一步不如預期，那就到不了~~葛來分多~~ +1 分的終點。

在這種情況下，我們希望機器人可以**有一套想法**，他可以知道在什麼情況下該怎麼做。假設他第一步想往上，卻因為那 10% 的機率向右了。那沒關係，就走 "右上上右" 就是了，反正一樣到得了終點。

機器人的**這一套想法**，通常被稱做 **Policy**。比起一個可以直接照做的 plan，policy 比較像是一套規則，告訴機器人在什麼 state 要做什麼事。

延伸一下，我們可能遇到過某些人，或自己也發生過，還不太會變通，講一動才做一動，這就像是 [你還在抱怨老闆「幹嘛不一次說清楚」嗎？新鮮人該知道但很少人告訴你的職場五件事](https://www.thenewslens.com/article/13041) 裡面提到的印表機例子。

## 怎麼用數學來描述 MDP？

首先，我們需要定義一些基本的東西，例如有：

- 總共有哪些狀態（機器人可以在迷宮裡的每一格）
- 起始狀態是什麼（機器人在最左下角的那一格）
- 可以採取的行動（往上、往下、往左、往右走）
- 轉移的機率（例如在某一格 s_k，採取的 action 是往上走，那只有 0.8 的機率會讓 s' 真的是 s_k 的上方那一格；各自有 0.1 的機率會讓 s' 落在 s_k 的左邊或右邊）
- 到達某個狀態獲得的獎賞（例如走到 -1 那格就會被扣一分）

寫成數學符號變成下面這樣：

![img](https://i.imgur.com/rzLEQCP.jpg)
(This image comes from [UC Berkeley CS188 Lecture 8](https://www.youtube.com/watch?feature=player_embedded&v=wKx4MuLfe0M))

另外，考慮到我們想要解的東西，我們還需要定義

- reward（下面用 value 來表示）
- Q-state（Q(s,a) 表示你已經從 state s 選擇了 a action，但因為 a action 產生的結果是什麼還不確定，所以我們把這個還沒決定的 state 稱作 Q-state）
- policy（就是我們希望學到的東西，讓機器人知道在什麼 state 底下該採取什麼 action）

![img](https://i.imgur.com/nPTNz0j.jpg)
(This image comes from [UC Berkeley CS188 Lecture 8](https://www.youtube.com/watch?feature=player_embedded&v=wKx4MuLfe0M))

## 怎麼解 MDP？

定義完上面這些數學符號之後，接下來我們來講解一下該怎麼解。

1. 假設我們在 state s，我們希望可以挑到讓 Q-state value 最大的 action；

   ![img](https://i.imgur.com/Fp4RICL.jpg)

2. 而 Q-state value 要最大，就看這個 Q-state 會落到哪些 s'，這些 s' 所帶來的 value 會最大：

   ![img](https://i.imgur.com/RsDW1xt.jpg)

   為何式子會長成上面這樣？因為我們用 $$T(s, a, s')$$ 來表示從 state s，採取 action a，會變成 state s'的機率。這個機率再乘上 s' 的 reward，還有 s' 會得到的 value，就是我們預期 Q-state 會帶來的好處。

3. 把 1. 的 $$V^*(s)$$ 用 2. 展開：

   ![img](https://i.imgur.com/gTHnLGA.jpg)

最後，把這些步驟合起來，就會得到一個可以慢慢收斂的演算法。

![img](https://i.imgur.com/zhQjACa.jpg)

以一開始的迷宮例子來說，我們經過 100 次 iteration，就會知道在每個 state，往哪邊走會得到最大的 value，這些 value 跟 action 方向顯示在下圖：

![img](https://i.imgur.com/7kOCcSR.jpg)

而這其實就是讓機器人知道他在不同 state 該怎麼選擇 action 的 policy。

## MDP 的應用

基本上只要是連續決策、而且在決策過程中的行動具有不確定性，就可以用 MDP 來處理。

比如說該要怎麼投資股票、怎麼跟人對話等等，有很多有趣的應用。有興趣的讀者可以參考 [這篇文章 - Real Applications of Markov Decision Processes](http://www.it.uu.se/edu/course/homepage/aism/st11/MDPApplications1.pdf) 或 [這個答案](https://stats.stackexchange.com/a/178393)。

## MDP 的限制

雖然我們已經學會怎麼用 MDP 來獲得一個 policy，但 MDP 有一些限制是我們需要知道的，才能夠在對的地方正確使用：

1. 無法處理非上帝視角的問題：我們生活的世界中，有很多東西是我們還無法觀測到的（比如人內心的想法、比如宇宙中的暗物質），所以我們無法描述這世界的真實狀態，這種問題就由更進階的 [Partially Observable Markov Decision Processes](https://stefanosnikolaidis.blogspot.com/2018/08/pomdps-intro.html) 來嘗試 model。
2. 只考慮到 reward，沒考慮到採取 action 的 cost：我們在採取行動時，除了會考慮獲得的 reward，也會考慮付出的代價，有興趣的讀者可以參考這篇 [The naïve utility calculus: Joint inferences about the costs and rewards of actions](http://jjara.scripts.mit.edu/cdl/docs/jaraett_NUC_cogsci2015.pdf)。

## 總結

今天跟大家介紹了 MDP 的基本觀念跟數學解法，希望大家在學習 Reinforcement Learning 時有更紮實清楚的底子。

## 延伸閱讀

1. [UC Berkeley CS188 Intro to AI -- Course Materials](http://ai.berkeley.edu/lecture_videos.html)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
