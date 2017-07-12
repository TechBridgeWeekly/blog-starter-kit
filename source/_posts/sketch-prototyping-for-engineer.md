---
title: 給工程師的 Sketch Prototyping 簡易入門教學 
date: 2016-06-04 12:26:00
tags: Sketch, Prototyping, Wireframe, Mockup
author: kdchang
---

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/sketchapp.png)

## 前言
[Sketch](http://www.sketchapp.com/) 是近來非常火紅的輕量級 UI 設計工具，簡易好上手的設計讓許多設計師紛紛從 PS 和以拉等其他工具跳槽過來，整個 Sketch 生態系也蓬勃發展。雖然筆者本業是工程師，但前陣子趁著買 Flinto （可以很輕易將 Sketch 的靜態頁面轉成 Prototype）送 Sketch 的行銷活動時同時入手了這兩套設計工具（雖然我把它當成買 Sketch 送 Flinto 啦：P），因此藉著機會分享一下給工程師的 Sketch Prototyping 簡易入門教學。當然有些讀者會疑惑：工程師學 Sketch 是要設計師搶飯碗嗎？（事實上很難啦）

一般來說，在整個產品開發的過程當中，工程師和設計師的愛恨情仇已經可以編成好幾檔連續劇了，然而溝通不良的原因除了設計流程的問題外，有很大一部分是因為對於彼此的世界不了解（一個住火星，一個住土星？）。因此工程師和設計師若是試著了解對方的語言，或許就可以降低彼此的溝通成本，從此相親相愛了呢！而天賦異秉的人甚至有機會成為新創團隊人見人愛的超全端工程師（super full-stack 
engineer）獨角獸，集設計、前後端、行動端、行銷業務技能於一身（咦）？好啦，言歸正傳，接下來就讓我們好好了解 Sketch 這個好用工具吧！

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/sketch-ui-kit.png)

## Sketch 基本介紹
Sketch 是一個 Mac 上的輕量級介面設計工具。2010 年由荷蘭公司 Bohemian Coding 所創建，目前提供 30 天試用版，正式版 99 鎂（教育價5折優惠），不過目前只有提供 Mac 版本，所以只能先跟使用其他作業系統的朋友說聲 Sorry 啦！

究竟 Sketch 有哪些優點，讓許多設計師因此琵琶別抱呢？
### Sketch 優點
1. 活躍的第三方套件生態系！
2. 專為 UI 設計而生（Artboard 設計可以方便 UI 設計） 
3. 輕量級，簡單好上手（工程師 OS：就像 Sublime 之於 Eclipse）
4. 內建和可擴充的豐富 UI 模版（iOS、Material Design 等）
5. 搭配 [Flinto](https://www.flinto.com/)、[Zeplin](https://zeplin.io/) 可讓工程師更開心，設計師提早下班
6. 可以取代大部分 PS、以拉、Fireworks 等功能
7. 知名網路、科技公司都在用

看完了 Sketch 優點，相信讀者們一定摩拳擦掌準備牛刀小試啦！

## 使用 Sketch 設計 Mobile App UI
一般設計流程我們通常會有以下三種階段：
1. Wireframe（線框圖）
保真度低（low fidelity ）、修改成本低。適合初期討論產品架構和基本功能。通常使用紙筆手繪，但建議於完成後電子化
2. Mockup（視覺稿）
保真度中（middle fidelity ）、修改成本中。於 Mockup 階段著重於整體視覺、排版的靜態討論和確認，而 Sketch 主要會是在 Mockup 階段進行設計
3. Prototype（原型）
保真度高（high fidelity ），接近最終產品、修改成本高。確認互動設計的流程並確認架構和視覺規劃是否有需要調整之處

先介紹一下 Sketch 工作空間，左邊是圖層區，右邊是屬性、中間是工作區、上面是工具欄：

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/sketch-0.png)

工具欄可以點擊右鍵自定義：

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/sketch-0-1.png)


由於 UI 設計是 Sketch 主要的強項（尤其是行動端），在 Sketch 中 Artboard 是基礎的設計單元（成果展示區域），我們可以在一個 Page 中定義多個 Artboard。根據 Sketch 的設計，我們可以一次瀏覽多個 Artboard（也可說是多個介面），這樣可以更清楚整個設計 flow ，這是 Sketch 蠻大的一個優點（一覽眾山小？）。

現在，我們終於開始設計我們的新 App UI 啦！首先，我們透過左上方的新增按鈕新增了一個 Artboard ，並選擇了 iPhone6 當做設計基底（有 iPhone、Android、RWD等可以選）。接下來我們要打開在 Sketch 中內建的元件模版，iOS 和 Material Design、icon設計等（網路上也有很多網友提供自己的設計可以下載），這邊因為是使用 iPhone6 的 Artboard，因此我們打開 iOS 元件模版。

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/sketch-7.png)

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/sketch-1.png)

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/sketch-2.png)

接下來進行複製貼上大法！將元件複製到 Artboard 上就可以了！現在小工程師也可以設計簡單的 UI 啦！可以透過右上角工具預覽或是分享的方式看到成果！

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/sketch-4.png)

## 從 Mockup 到 Prototype
![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/sketch-flinto.png)
透過 Sketch 設計完成靜態檔案，若我們想將靜態檔案轉成 Prototype 進行使用者測試，我們可以使用 Flinto 好工具！透過簡單的連結就可以讓你的 Sketch 動起來啦（目前支援了簡單的手勢和動畫）！

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/flinto-3.png)

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/flinto-4.png)

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/flinto-5.png)

## 結論
以上就是給工程師的 Sketch Prototyping 簡易入門教學。事實上透過整合 [Zeplin](https://zeplin.io/)（可以方便註解和查詢設計相對位置和色碼）、[Invision](https://www.invisionapp.com/)（支援變更記錄，可以當做版本控管使用）等工具進行協作，還可以更加加快整個產品設計流程。透過前期的完善溝通討論，不但可以降低後續的溝通成本更可以讓完成的產品更接近最初的設計，更重要的是讓工程師和設計師的感情更好了！若你對於 Sketch 有興趣可以[參考這份投影片](http://slides.com/kd-chang/sketch-prototyping-for-engineer)有更完整的教學介紹。另外也可以參考[官方教學影片](http://www.sketchapp.com/learn/)。最後再次強調工程師學 Sketch 並非要去搶設計師搶飯碗。而是工程師和設計師若是試著了解對方的語言，或許就可以降低彼此的溝通成本。同理也建議設計師們可以去試著學一些簡單的程式語言基礎概念。筆者才疏學淺，若讀者有任何學習心得也歡迎分享交流：）

![給工程師的 Sketch Prototyping 簡易入門教學](/img/kdchang/sketch-5.png)

## 常用外掛
0. [外掛管理工具 Sketch Toolbox](http://sketchtoolbox.com/)
1. Notebook 註解好幫手
2. Sketch Flinto 從 Mockup 到 Prototype
3. Content Generator Sketch Plugin 生成假資料
4. Sketch Measure 測量位置、大小
5. Icon Stamper 簡單生成不同大小 icon

## 延伸閱讀
1. [Sketch 簡體中文教學](http://www.sketchcn.com/)
2. [Sketch 問答](http://www.sketchchina.com/)

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
