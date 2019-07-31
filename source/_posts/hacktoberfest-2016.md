---
title: Hacktoberfest：一起踏入 open source 的世界吧！
date: 2016-10-14 19:51:15
tags:
    - hacktoberfest
    - github
    - open_source
author: huli
---

![Hacktoberfest](/img/huli/hacktoberfest.png "hacktoberfest")
（圖片來自： [Hacktoberfest is back](https://github.com/blog/2260-hacktoberfest-is-back)）

# 前言

[Hacktoberfest](https://hacktoberfest.digitalocean.com/) 是由 `hack` + `october` + `festival` 組合而成的名詞，是由知名的 VPS 供應商 [DigitalOcean](https://www.digitalocean.com) 與大家所熟知的 [github](https://github.com/)  所舉辦的活動，目的在於推廣 open source，希望許多人能藉由這個活動踏進 open source 的世界。

這個活動的口號是：
> Support and celebrate open source. Earn a limited edition T-shirt.

只要在十月份送四個 Pull Request（任何在 github 上公開的 repo 都可以），就符合免費拿 T-shirt 的條件！

在 [活動網站](https://hacktoberfest.digitalocean.com/) 上，有一些 Featured Projects，你可以直接去看看有沒有 issue 是你可以幫忙解的。

當然，如果你跟我一樣，是個開源初心者，網站下方也有許多有用的文章，可以幫助你了解如何對開源專案做出貢獻。

# 心得

儘管現在幾乎所有的工程師都聽過 github，也偶爾會看到一些有趣的專案；甚至連很多公司在徵才的時候，都會要求附上 github 帳號。但是，我相信有很多人跟我一樣，「幾乎從來沒有送過 Pull Request（以下簡稱 PR）」。

所以這篇文章主要是想宣傳一下這個活動，希望大家都有機會體驗到送 PR 的感覺，順便簡單寫給新手一些教學，讓你知道什麼是 PR。

## PR 是什麼？

簡單來說，當你在 github 上面某個專案看到一些 issue 的時候，你就可以試著去解掉這個 issue。

這邊的 issue 指的不一定是程式發生的 bug，也有可能是文件不夠完整，需要加強，或者是要幫忙寫測試，或只是簡單的補上專案的 License 等等。

為了要修改這個專案，你必須要 fork 一份到自己的帳號底下，用這份專案當作基底來修改（一般來說，還會新增一個 branch），修改完以後，你要怎麼把這些修改好的 commit 發給原作者呢？

因為原本的那個專案不是你的，你沒有權限（fork 的那一份你才有權限），所以你又不能直接 push 到原本的上面，那原作者要怎麼收到你這份修改完成的 code 呢？

沒錯！這個時候就需要 Pull Request，提交一個 PR。

當你提交 PR 的時候，可以指定你自己的 branch 跟原作者那邊的 branch，並且把你剛剛新增的 commit 顯示出來，就可以提交一個新的 PR 給原作者，讓原作者去審核這個 commit。如果順利的話，就會直接把你這個 commit merge 進去原來的專案，這份專案就有你的貢獻在裡面了！

如果你需要更詳細地講解，可以參考：

[花20分钟写的-大白话讲解如何给github上项目贡献代码](https://site.douban.com/196781/widget/notes/12161495/note/269163206/)
[3.3 创建Pull Request](https://github.com/geeeeeeeeek/git-recipes/wiki/3.3-%E5%88%9B%E5%BB%BAPull-Request)
[Github 發 Pull Request & 貢獻流程速查](https://gist.github.com/timdream/5968469)
[6.2 GitHub - 參與一個專案](https://git-scm.com/book/zh-tw/v2/GitHub-%E5%8F%83%E8%88%87%E4%B8%80%E5%80%8B%E5%B0%88%E6%A1%88)

## 詳細流程簡介

透過 hacktoberfest 這個活動，我找到一些對新手非常友善的 issue，身為一個新手，我十分感動。例如說我們可以來看看 [這一個 Telescope 的 issue](https://github.com/TelescopeJS/Telescope/issues/1460)

![Telescope issue 1](/img/huli/telescope-issue-1.png "issue1")

![Telescope issue 2](/img/huli/telescope-issue-2.png "issue2")

這 issue 直接跟你講說你需要會什麼、要解決的問題是什麼，以及如果你是第一次 contribute 的話，你應該要注意哪些事項，提供 step by step 的步驟給你。

很友善對吧！

通常在專案裡面都會有一份 `CONTRIBUTING.md`，想要貢獻以前記得仔細閱讀！裡面會說明詳細的流程，一般來說，流程大致上都會長這樣：

1. 找到 issue，確認沒有其他人跟你同時在做（會有 label 顯示，通常沒人在處理的會寫 up for grabs，有人在做的會寫 in progress），並且在底下說明你想要解這個 issue。
2. 得到同意以後，你就可以開始解這個 issue 了，這 issue 的標籤也會變更成 in progress，有些專案會要求把這個專案 assign 給自己。
3. fork 這個專案到自己的帳號底下
4. 在你的電腦上把 fork 好的專案 clone 下來
5. 查看專案裡面的貢獻指南（一般都是 `CONTRIBUTING.md`），裡面會有詳細的流程，可參考我下面的說明
6. 新開一個 branch（`CONTRIBUTING.md` 裡面通常會提示你 branch 名稱應該取什麼，有的會取 issue 的編號，有的會取叫 dev，每個專案都不同）
7. 設定好本機的環境，順利讓專案執行，並且跑測試（你會在 `CONTRIBUTING.md` 裡面看到如何設定環境）
8. 開始解 issue 囉！

可是，如果你程式沒那麼強，是不是就沒辦法貢獻了呢？

不是的！有些 issue 會特別標明是專門給新手的，適合第一次 contribute 的人！
就算你還是覺得這些 issue 太難，有些 issue 與程式無關，可能只是幫忙寫一些文件之類的。

在解完 issue 以後，接著就是發 PR 了。但是在發 PR 之前，請務必確保自己改過以後的 code 有通過測試。因為你送 PR 上去以後，通常也會再跑一遍測試，自己先在本機測過，可以省掉很多時間，也減少要幫你 review 的人的麻煩。

確認測試通過以後，就可以送 PR 了。

在送 PR 的時候，內文跟標題通常也會有一定的規範，例如說要提到你解了哪一個 issue，用什麼方法去解的等等，總之就是大概說明一下即可。

最後，就等專案的作者們有時間的時候會來審核，審核通過以後，你就正式對這個專案產生貢獻了。

# 結論

[有標上 hacktoberfest 標籤的 issue](https://github.com/search?l=&q=state%3Aopen+label%3Ahacktoberfest&ref=advsearch&type=Issues&utf8=%E2%9C%93) 可以先嘗試解解看，或是從活動網站上面推薦的專案開始下手。

比較大的專案的好處就是文件很完整，把每一個步驟都寫得很清楚，讓你不會不知所措。

如果你跟我一樣是寫 JavaScript，推薦你 [Telescope](https://github.com/TelescopeJS/Telescope) 這個專案，裡面每一個 issue 都寫得很清楚要做什麼，contribute 的文件也非常齊全，有些甚至還會附提示呢！

希望大家趁著這個活動可以嘗試看看送 PR、拿衣服，如果沒有拿到也不用太傷心，至少你對開源的世界有了更進一步的瞭解，也是獲益良多了。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好