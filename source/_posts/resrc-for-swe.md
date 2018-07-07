---
title: 軟體工程師面試資源最簡整理與技巧分享
date: 2018-07-07 13:46:50
tags: software engineer, data structure, algorithm, interview
author: pojenlai
---

## 前言

只要是在 CS 產業，想要當軟體工程師，基本的資料結構和演算法功力是必須要時時打磨的，這次就整理一些好用的工具，而且盡量把資源最小化，讓大家在準備面試時有足夠資源，又不會被太多資料弄得暈頭轉向。

## 線上練習寫 code 網站

- [Hackerrank](https://www.hackerrank.com)

如果是剛開始刷題，可以透過寫 Hackerrank 的題目幫忙複習基本的資料結構和演算法。
![DS](https://i.imgur.com/nqhF4tQ.jpg)
![ALG](https://i.imgur.com/LGFssTb.jpg)

- [Lintcode](https://www.lintcode.com)

Lintcode 有一些 ladder 也還不錯，可以幫助循序漸進地刷題。比如說下圖的 bit manipulation ladder：
![lint ladder](https://i.imgur.com/8KrTi9B.jpg)

- [Leetcode](https://leetcode.com/)

Leetcode 應該就不用多說，很多人應該都知道，只要你有了一定基礎，需要大量練習題目，那 Leetcode 絕對是不二選擇。

而且 Leetcode 也可以按照不同的 topic，區分難易度，甚至可以花錢看到各公司常出的題目，也是很不錯。

![leetcode](https://i.imgur.com/KfR4pp6.jpg)

## 線上課程

如果想要有系統地學習，花點錢從線上課程學習也是很有效率的方法。

### 演算法與資料結構

- [九章算法課程](https://www.jiuzhang.com/course/)

九章算法是中國有名的演算法教學網站，他們現在也推出了許多課程，如果是喜歡有人幫忙把資料整理好、並包含講解的，也可以先來上這個課，再好好刷題。

### 系統設計

- [Grokking the System Design Interview](https://www.educative.io/collection/5668639101419520/5649050225344512)

![sys-website](https://i.imgur.com/QECH6ST.jpg)

## 技巧

### 各項能力分開練習

基本上要寫出一道題目，而且要做到 bug free，必須要有下列能力：
1. 釐清問題（展現細心程度，可以跟面試官溝通清楚問題範圍再下手）
2. 想出演算法（展現資料結構和演算法的應用能力）
3. 寫出 pseudo code（展現把初步想法變成可運行程式的能力）
4. 寫出 code（展現程式語言的掌握能力）
5. 人體 testing & debugging（展現 unit test、integration test 的基本能力）
6. 溝通能力（問清楚問題、把自己的想法講清楚、在必要時尋求協助）

如果把所有能力都混在一起練習，就很容易混亂，因為每一道題目讓你卡的點可能都不同；反之，如果有意識地發現自己容易卡在什麼地方，就容易專項加強（例如你常常可以想出演算法並實作，但常常掛在 edge case，那你應該先加強 1，讓自己在釐清問題時就把 test case 考慮得更完善，再開始想演算法；然後也要加強 5，讓你在測試自己程式時，可以完整測試自己的程式是否可以 handle 在第 1 步考慮到的所有 test case）。

### 記錄過自己寫過的題目還有解法
這一點 **非常非常非常** 重要！如果能夠把想過的解法清楚地寫下來，之後想要複習就會事半功倍，畢竟有很多很 tricky 的題目，如果每次都要重想，會花非常多時間。
此外，如果想要準備得更充足，寫完問題想一想 follow-up question 會怎麼出也是很重要的，如果沒有文件累積，每次看到題目連以前想懂的東西都要重來，那又怎麼有時間準備更進階的問題呢？
紀錄的工具很多種，像是寫個 blog、gitbook 或是 github repository 都是不錯的選擇。可以參考 [這本 gitbook - leetcode note](https://legacy.gitbook.com/book/shanyuc/leetcode-note/details)。

### 資源不用多，足夠就好

說真的，網路上的面試準備資源超級多，整理得很用心的 github repository 如 [awesome-interview-questions](https://github.com/MaximAbramchuck/awesome-interview-questions) 也很多，但是，重點還是在於能否把題目寫出來。所以與其收集一堆資源但造成大腦超載，不如就專注在解題目上面。
不過，我不是要否定這些資源的價值，因為每個人喜歡準備的方法不同，不同偏好就是和不同的資源（最簡單說就是有人喜歡文字學習，有人喜歡圖像學習，那同樣遇到想不出來的題目，喜歡圖像學習的人就可以去 youtube 找 [解題的影片](https://www.youtube.com/watch?v=CfNRc82ighw) 看，而不必執著硬要看懂 leetcode 討論串）。
## 總結

以上幫大家整理了一些好用的面試準備資源，如果你想要上手，可以從 hackerrank 跟 lintcode 開始找回資料結構和演算法的基本手感。要練習各種變形題目的話，就上 leetcode 狂刷題。如果看到某個知識想要更加了解，google 看看或是參考 [awesome-competitive-programming](https://github.com/lnishan/awesome-competitive-programming)、[awesome-algorithms](https://github.com/tayllan/awesome-algorithms) 之類的整理也很夠用了。

祝大家準備愉快！

## 延伸閱讀

1. [花花醬刷 Leetcode Youtube Channel](https://www.youtube.com/user/xxfflower/videos)
2. [軟體工程師面試資源整理](https://medium.com/hungys-blog/software-engineer-interview-resources-9cb5be8e451)
3. [Google Code Jam 賽制介紹](https://domen-blog.github.io/posts/2016-04-10/google-code-jam-introduction/)（如果沒要找工作，覺得自己刷到夠強了也可以衝一個看看拿 T-Shirt）

關於作者：
- [@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
