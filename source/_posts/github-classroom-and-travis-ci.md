---
title: 利用 Github Classroom 加 Travis CI 打造改作業系統
date: 2018-02-03 09:47:55
tags:
  - github
  - github classroom
  - travis
  - ci
author: huli
---

## 前言

這陣子開始了一個自己的 side project，叫做[程式導師實驗計畫](https://github.com/Lidemy/mentor-program)，希望能在四個月內把學生培養成找得到工作的工程師。

而 Git 身為工程師的必備技能之一，用 Git 來交作業也是合情合理的一件事，可以先培養學生們對 Git 的熟悉程度。

但問題來了：要怎麼用 Git 交作業？

之前有開過另外一個[前端的課程](https://github.com/aszx87410/frontend-intermediate-course)，那時候我採取的方式是讓學生自己開一個 Github repo 寫作業，並且設置 Github page，所以我可以看到學生的 source code 跟網頁顯示出來的結果。

接著自己設定好 Issue template，要學生寫完作業之後開 issue 提交，如下圖所示：

![](/img/huli/homework/p1.png)

這樣的好處是我可以把作業都集中在同一個地方管理，可以很方便看出誰交了哪些作業，以及每個作業的狀況：

![](/img/huli/homework/p2.png)

但缺點也很明顯，那就是身為老師，其實很難「改作業」，意思就是如果我要指正學生們哪邊寫錯了，我只能在 issue 裡面留言，複製它本來的程式碼，然後告訴他怎麼改應該會比較好：

![](/img/huli/homework/p3.png)

整體來說，其實改作業的體驗還算不錯，沒什麼太大的問題。只是這次既然開始了一個全新的課程，就在想說有沒有更好的方法可以優化這個流程。

## 新的交作業流程

我在設計課程的時候，都會先思考我在工作上運用了哪些東西，並且把我覺得好的、可以移植的制度搬到課程上，背後的目的是希望先讓學生理解這些東西，日後進入職場時可以無縫接軌。

但有時候我不會跟他們講這是你以後工作時可能會碰到的流程，期待他們真的碰到時驚呼：「哇，原來我在課程裡面做過的練習其實是工作上會用到的東西！」

舉例來說，因為這次新的課程是要求學生每天參與，在我沒有上課的時間自學，剛好公司有在跑 Scrum，每天早上都要開 Stand-up meeting，並且在開始前先在 slack 裡面送出簡短的 note，我就把這個制度引入到課程之中。

``` markdown
*昨天*
- 完成 git 安裝
- 解 codewar 題目：Opposite number

*今天*
- 解 codewar 題目：Opposite number
- 寫作業：好多星星
```

每天我都會要求學生在 slack 群組裡面 po 昨天跟今天做的事情，雖然跟實際的 Stand-up meeting 還是相差許多，但至少初衷是一樣的：「幫自己整理進度、讓大家了解你的進度」。

秉持著相同的理念，這次的交作業機制我決定採用[ Github Flow](https://guides.github.com/introduction/flow/)。

什麼是 Github Flow？可以先看一下我在官網上面截的圖：

![](/img/huli/homework/github-flow.png)

簡單來說就是如果你要做任何改變的話，你要 follow 以下原則：

1. 開新的 Branch
2. 送 Pull Request
3. 等 Review
4. 確定沒問題，merge 到 master

我們公司也是採用類似的工作流程，因此我自己本身對這個流程滿熟悉的。而這個流程的好處是什麼？就是在送 PR 的時候，你可以很方便地看到改動以及加上建議：

![](/img/huli/homework/review.png)

這樣的方式，豈不是最適合拿來改作業？直接加上註解，可以一行一行修正，合格的作業就直接 approve，不合格的要求修正，然後再送一次 review。

決定好採用 PR 的方式交作業以後，其實還有一個東西要決定，那就是要怎麼送 PR？意思是說，PR 要在哪一個 repo 底下開？大概有以下幾種做法：

1. 老師開一個 `hw` repo，開權限給所有學生，學生寫完作業以後對 `hw` 送 PR
2. 學生開一個 `hw-student` repo，把老師加成協作者，寫完作業送 PR 讓老師 review

前者的話，你必須要在`hw`底下開不同資料夾，這樣每個學生才有地方來放自己寫的作業。好處很明顯，就是都集中在同一個地方進行管理，但缺點就是這個 repo 會變得很大，因為你可能同時要放 10 個學生寫的作業。

後者的話，學生自己開 repo，然後把老師加進來讓老師能 review，比第一種分散，但是自由度高很多，而且結業以後，學生可以直接把他的 repo 當成作品集的一部分。這兩種比起來，我是比較偏好這個的。

除此之外，其實還有另外一個問題需要解決，那就是有時候作業有固定格式需要遵守，例如說我有一些簡答題，已經在`hw`下面開好回答的模板，學生只要照著格式寫答案就好，那學生就必須複製這個模板到自己的 repo 底下，其實也是滿麻煩的。

那更好的方式是什麼？

很簡單，就是結合了前面兩種：

> 老師開一個放作業模板的 repo，學生 fork 這個 repo 到自己帳號底下，並且利用這個 fork 的 repo 交作業

這樣子學生就不用從零開始，可以直接採用老師已經寫好的交作業模板跟格式，只要照著做就行了。而這種處理方式，其實就是我們等等會提到的 Github Classroom。

## Github Classroom

我一開始看到這個，還以為是什麼神奇的系統，可以自動幫你完成一堆跟改作業有關的事情。但很可惜，這並不是。

Github Classroom 的系統很簡單，首先你要先註冊一個 organization 才能使用。進去之後你可以建立一個 Classroom，就是一堂課的意思。

而每堂課底下，都有一個地方可以讓你新增作業，在新增作業時你可以關聯自己帳號底下的 repo，介面長這樣：

![](/img/huli/homework/gcr.png)

而這個被關聯的 repo，就是你要拿來出作業的 repo，所以你可以先把很多東西都寫好，例如說交作業的規則跟格式之類的。像我的話就是會先把檔案開好，學生只要在指定的檔案下面寫答案就好：

![](/img/huli/homework/hw.png)

在新增作業之後，會有一個自動產生的 invitation link，只要學生點擊並且加入之後，就會自動在你的 organization 底下產生新的 repo。

例如說我拿來關聯的 repo 叫做`mentor-program`，學生的帳號是 abcd，就會產生一個 `mentor-program-abcd`，而這個 repo 就是基於你原本的所產生的，所以東西都一模一樣。產生之後呢，他會自動把學生本人跟老師設為協作者，然後學生只有 developer 的權限，老師則是有 admin 的權限。

所以說呢，用了 Github Classroom 的好處就是有一個自動化的系統幫你 fork 一份你的 repo 給學生，然後自動設定權限，並且在後台可以看到每個學生的 repo：

![](/img/huli/homework/gcr2.png)

做到這裡，你就有一個很不錯的改作業系統了，流程非常簡單：

1. 學生透過 Github Classroom 產生的連結加入
2. 產生 `mentor-progam-student_username` 的 repo
3. 學生 clone 下來，開新的 branch，寫作業
4. 寫完作業送 PR
5. 老師 review，確認沒問題再 merge

## 結合 CI 自動改作業

剛剛有說過，你學生的 repo 都是以你提供的為基礎 fork 出去的，所以學生可以按照你訂的規則來寫作業。

舉我剛剛提到的例子，我就先幫學生開好`hw1.js`、`hw2.js`...他們直接在檔案裡面寫答案就好。如果你有注意到的話，我還幫他們開了`hw1.test.js`，拿來做 unit test 用的。

在第一週的作業裡面，他們被要求實作出幾個簡單的 function，例如說判斷質數、判斷回文等等。所以每個 js 裡面就只是 export 出一個 function 而已。那要怎麼驗證呢？跑測試！

既然是那麼簡單的一些 function，那就可以寫 unit test 來驗證結果是否正確，而這個時候，我就想到其實可以結合 CI 做出自動改作業系統。

流程很簡單：

1. 學生送 PR
2. 串上 CI，偵測到有 PR 自動跑 test
3. 在 PR 裡面顯示結果

完成後的結果會像這樣，可以直接在 PR 裡面看到 CI 跑測試之後的結果：

![](/img/huli/homework/ci.png)

我採用的系統是有名的[Travis CI](https://travis-ci.org)，其實應用上也很簡單，登入進去之後它會自動抓你的 repo，就可以看到一個列表，只要打勾就可以把 Travis 跟 Github 串起來：

![](/img/huli/homework/ci2.png)

不過在打勾之前，你要對你的 repo 做一些設定。其實 CI 的原理很簡單，就是你提供一些指令讓它幫你跑，以我的課程來說，就是跑`npm run test`而已。

只要在專案的根目錄底下新增`.travis.yml`，就可以指定你要跑的環境跟其他參數，以[我的專案](https://github.com/Lidemy/mentor-program/blob/master/.travis.yml)為例：

``` yml
language: node_js
node_js:
  - "node"
cache: yarn
before_script: 
  - wget $TESTCASE_URL
notifications:
  email: false

```

Travis 很聰明，所以預設就會執行`npm run test`，所以不必在這邊設定任何東西。可以注意到的是，我這邊額外設定了`before_script `，而後面接的參數就是你要執行的指令。

會這樣設定是因為我希望在 repo 裡面的 test 檔案可以讓學生自己練習，他們自由修改，而我真正拿來改作業的 test 我把它放在遠端了，在跑 CI 的時候才抓下來，確保學生不可能改到。

準備完成以後，你只要去 CI 的後台打勾，並且進去調一些設定（例如說只要幫 PR 跑測試、環境變數的調整等等），一切就大功告成了！

![](/img/huli/homework/ci3.png)

## 結論

結合了 Github Classroom 跟 Travis CI，就能夠很輕鬆的打造出一個方便讓學生交作業，也讓老師可以很輕鬆的批改作業，甚至是讓系統自動改作業的服務。

如果你想要更進一步，CI 那端還有很多延伸應用可以做，例如說跑測試失敗的話就自動把 PR 關掉，或者是自動回覆說哪一個作業是錯誤的，甚至也可以把這些訊息記起來，直接做一個學生的 scoreboard 之類的，有很多有趣的應用可以玩。

但若是你只是想要基本的東西，只要簡單設定一下就夠了。

這一篇整理了我最近課程的改作業流程，目前嘗試起來都覺得很不錯，一來是我可以方便改作業，二來是強迫學生熟悉 Git 的流程，而且他們會越來越熟練。

如果你有哪些更好的建議，歡迎在下面留言，文中如果有錯也麻煩不吝指出，感謝。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好