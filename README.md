# TechBridge 技術共筆部落格

[![Build Status](https://travis-ci.org/TechBridgeWeekly/blog-starter-kit.svg?branch=master)](https://travis-ci.org/TechBridgeWeekly/blog-starter-kit)

TechBridge 技術共筆部落格是由 TechBridge Weekly 技術週刊團隊維護的技術部落格。本技術共筆部落格初期專注於 Web前後端、行動網路、機器人/物聯網、數據分析與產品設計等正體中文原創文章、國外好文翻譯分享。This is TechBridge Weekly Team's Technology Blog, which focus on web, mobile, robotic, IoT, data analytics and product design sharing.

- 技術共筆部落格：[http://blog.techbridge.cc/](http://blog.techbridge.cc/)
- 技術日報：[http://www.techbridge.cc/](http://www.techbridge.cc/)
- 技術週刊：[http://weekly.techbridge.cc/](http://weekly.techbridge.cc/)
- 粉絲專頁：[https://www.facebook.com/TechBridge.Fans/](https://www.facebook.com/TechBridge.Fans/)

## Contribute 
0. TechBridge 技術共筆部落格使用 Github 和 Hexo 平台，若您對於 Hexo 並不熟悉，請先閱讀[官方文件](https://hexo.io/)
1. `$ npm install -g hexo-cli`
2. 請先 Fork 一份檔案後 git clone 至本地端，並開啟自己的 branch
3. `$ npm install`
4. 寫新文章 `$ hexo new post <title>`，在 source > _post 資料夾下開始使用 Markdown 撰寫文章（寫靜態檔案為 new page、草稿：new draft）
5. 重新 compile 一次，將 .md 檔案轉成 .html 等靜態檔案 `$ hexo g ` (若有圖片請存在/sources/img/your_name/下)
6. 在本地端 [http://localhost:4000](http://localhost:4000) 觀看效果 `$ hexo s`，（若遇到問題請多 `$ hexo g ` compile 幾次）
7. 與上游保持同步
	- `$ git remote add upstream https://github.com/TechBridgeHQ/blog-starter-kit`
	- `$ git remote update`  #更新所有Repository branch
	- `$ git fetch upstream`
	- `$ git rebase upstream/master`
8. 請持續於自己的 brach 開發，完成後 Push 到你的 Repo
9. 完成後，請發 Pull Request 到 [TechBridgeHQ/blog-starter-kit](https://github.com/TechBridgeHQ/blog-starter-kit)，負責編輯審核通過即會刊出。若有任何問題歡迎[來信](techbridge.cc@gmail.com)或是發 issue :)

## Format（文章格式）
- 文章內容請遵守[中文文案排版指北](https://github.com/sparanoid/chinese-copywriting-guidelines)
- 若為翻譯文章，用詞請先搜尋相關用法，或可以參考這個[網站](http://jjhou.boolan.com/terms.htm)

```
---
title: Caffe & GoogLeNet，怎麼幫助機器人更好地辨識物體
date: 2016-03-19 10:54:49
tags: 機器人, 深度學習, Deep Learing, Robot, GoogLeNet, Caffe
author: pojenlai
---
以下文章內容
XXXXX

以下請關於作者先自己加上，之後會轉成自動生成：
關於作者： 
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人跟電腦視覺有少許研究，最近在鍛鍊自己的執行力

```

## Documents
1. [使用GitHub和Hexo搭建免费静态Blog](http://wsgzao.github.io/post/hexo-guide/)
2. [Hexo常见问题解决方案](https://xuanwo.org/2014/08/14/hexo-usual-problem/)
3. [Hexo 安裝教學、心得筆記](https://wwssllabcd.github.io/blog/2014/12/22/how-to-install-hexo/)

## 目前負責的 Curator（歡迎有興趣朋友跳坑加入策展人團隊:P）
1. [@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校
2. [@arvinh](http://blog.arvinh.info/about) 前端攻城獅，熱愛數據分析和資訊視覺化
3. [@huli](http://huli.logdown.com) 野生工程師，相信分享與交流能讓世界變得更美好
4. [@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人跟電腦視覺有少許研究，最近在鍛鍊自己的執行力
