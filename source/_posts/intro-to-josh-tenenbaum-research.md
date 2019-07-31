---
title: 一起來看 Joshua B. Tenenbaum 教授有趣的認知科學研究 - Building Machines that Learn and Think Like People
date: 2018-09-29 23:38:19
tags:
    - artificial intelligence
    - cognitive science
author: pojenlai
---

## 前言

[Joshua B. Tenenbaum](https://scholar.google.com.tw/citations?hl=zh-TW&user=rRJ9wTJMUB8C&view_op=list_works&sortby=pubdate) 教授的研究很有趣，他從小嬰兒的行為出發，研究人們是如何透過無數次跟世界的互動，建立起簡單的物理概念（e.g. 物質不滅、物質不會浮在半空中、物體會撞到彼此等等）、還有簡單的心理學（e.g. 看到別人去抓門把，就可以猜到那個人想要開門）。

今天就想要來介紹一下他的研究和未來的藍圖，讓有興趣鑽研人工智慧的讀者一起看看認知科學跟人工智慧的最前端研究。

## 我們是從什麼基礎開始建構起一切的能力?

這個問題，就是 Tenenbaum 教授研究中最核心的問題。

讓我們看看下面這部影片，在這部影片中，我們可以看到一個 18 個月大的小嬰兒，就已經有能力了解 "書會被櫃子們擋住，得把櫃子打開才能把書放進去"、"別人一直拿書去撞櫃子，是因為他想把書放進去，而且他沒有手開門" 等等稍具複雜度的物理概念和心理學概念。

[![video](https://img.youtube.com/vi/Z-eU5xZW7cU/0.jpg)](https://www.youtube.com/watch?v=Z-eU5xZW7cU)

在我們還無法自己生存、還無法說話、還無法做我們長大後能做的很多事之前，我們就已經學會基本的物理和基本的心理學概念。

這件事情提供的想法是，我們如果可以打造出有能力自己學懂這些基礎物理知識和心理學知識的機器，那這部機器是否就能夠再學會各式各樣的技能？就如同有能力學會基礎物理和心理學知識的小嬰兒，就有能力學會長大後需要學的各種技能。

## 腦海中的遊戲引擎

雖然嬰兒的學習過程提供了我們一些 insight，讓研究有了切入方向，但有能力學習的基礎到底是什麼？

以 Tenenbaum 教授的想法來說，大概就是一個 **能夠自我修正的、programmable 的遊戲引擎**。

上面這句話到底在講什麼東西？讓我們繼續往下看。

大家可能都有經驗，當你要找一件東西，你會開始回想起你上次還看見那個東西的場景，回憶你走過的每個地方，直到你不在記得那件東西還在，你就可以回到那個地方尋找。

小小的一件事情，包含了你的記憶能力、場景想像能力，就好像你的腦海中有一個可讀出檔案的遊戲。

再舉一個例子，當你要拿一個很珍貴的花瓶，你是不是會很小心翼翼地拿？因為你甚至可以在腦海中想像如果沒拿好花瓶，可能會摔到地上破掉？

這跟我們在電腦模擬中看到的現象簡直一模一樣，你可以模擬上千次，但不會直接改變現實世界；就像你可以在腦海中模擬無數次上台演講的畫面。

這個概念很重要，因為在處理訊息跟思考的時候，我們幾乎是時時依賴這個腦海中遊戲引擎。雖然有很多習慣因為已經被重複太多次，在使用時幾乎不需要調用遊戲引擎來模擬，但是這些習慣必然是符合我們腦內遊戲引擎的規則。

什麼意思呢？

就像我們每個人都拿過很多次杯子，所以幾乎是想都不用想就可以拿起來。但如果有一天，有個愛整人的朋友把你的杯子用強力膠黏在桌上，於是你要拿杯子發現它穩如泰山完全不動，你就會感到驚訝。因為在你的腦海中，杯子的重量是有一定程度的，而一般情況下，不可能重到你拿不起來。

所以這時候你就會開始研究，為什麼你看到的現象跟你 **腦海中的遊戲引擎** 預測得不一樣？直到你發現原來是被黏在桌上，你腦海中的遊戲引擎就會發生變化，讓它未來更有可能預測到杯子會被黏在桌上（似乎也有點強化學習的味道，對吧？）。

這也是為什麼魔術很有趣，因為魔術常常推翻我們遊戲引擎具備的規則；

[![video](https://img.youtube.com/vi/LFlstuiUQzo/0.jpg)](https://www.youtube.com/watch?v=LFlstuiUQzo)

## 怎麼打造像人一樣學習的機器 ?

Tenenbaum 教授的短期研究藍圖如下圖，他的想法，是從視覺著手：

![blue-print](https://i.imgur.com/yKcC8KS.jpg)

為什麼是從視覺，我想這也很合理，畢竟在我們還不太會走路、不太會抓東西的時候，我們就已經不斷地在看這世界中的各項事物，開始學習世界裡的一切規則，而這些規則都是我們可以正常走路、抓好東西的重要基礎。

有了藍圖之後，接下來就是方法，於是他們也提出了一個大的架構：

![architecture](https://i.imgur.com/YUehkv5.jpg)

而撐起這個大架構的技術細節，就是我們耳熟能詳的各種技術：

![tech](https://i.imgur.com/8OvUB6y.jpg)

但這些技術如何交互使用？如何構建成具備人類認知能力的基礎？都還是剛起步的研究題目，所以就留待大家去發掘。

## 總結

今天跟大家簡單介紹了 Tenenbaum 教授的研究，個人覺得他的研究出發點非常有趣，也完全可以跟我們的成長經驗串連，希望未來可以看到更多有趣的研究成果。

## 延伸閱讀

1. [MIT AGI: Building machines that see, learn, and think like people (Josh Tenenbaum)](https://www.youtube.com/watch?v=7ROelYvo8f0)
2. [Building Machines that Learn & Think Like People - Prof. Josh Tenenbaum ICML2018](https://www.youtube.com/watch?v=RB78vRUO6X8)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
