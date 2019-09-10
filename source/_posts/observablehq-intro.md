---
title: 從製作 visfest 2019 badge 認識 ObservableHQ

date: 2019-09-09 02:01:00
tags:
  - d3.js
  - javascript
  - visualization
---

## 前言

軟體工程師應該很少沒參加過 Conference 吧，不管是社群或是學術性的，只要是與會者都會拿到一面主辦方製作的名牌，上面除了印著你的大名外，大多就剩下 Conference 名稱與 Logo 了，然而，由灣區的資料視覺化社群所舉辦的年度聚會 - visfest unconf 很是特別，他們提供與會者一個製作自己 badge 的機會，讓大家自行發揮創意，特別之餘也很符合整個會議的調性。今年八月是他們舉辦的第五屆 visfest unconf，這次他們在 [ObservableHQ](https://observablehq.com/) 這個平台上釋出了一個 Template，讓大家更方便的製作名牌，效果如同下方（因為用到一些手機尚未支援的 API，建議使用桌面版 Chrome）：

<div id="animation"></div>

<script type="module">
  import notebook from "https://cors-anywhere.herokuapp.com/https://api.observablehq.com/@arvinh/visfest-unconf-badge-builder-template.js";

  const renders = {
    "result": "#animation",
  };

  import {Inspector, Runtime} from "https://unpkg.com/@observablehq/notebook-runtime@2?module";
  for (let i in renders)
    renders[i] = document.querySelector(renders[i]);

  Runtime.load(notebook, (variable) => {
    console.log(variable)
    if (renders[variable.name])
      return new Inspector(renders[variable.name]);
  });
</script>

除了 Observable 與 visfest 的 Logo 外，你可以繪製任何你想呈現的東西，改變背景顏色等等，而主辦方會用 [gitpop](https://gifpop.io/) 這項服務，將你的動畫製作成 10 frames 的 gif，因此在製作時，可以根據 template 提供的繪圖函式傳入的 `frameNumbers` 來控制動畫的呈現。

我自己是覺得這樣的想法很酷，所以即便無緣參加 visfest unconf，也製作了一個自己的 Badge 玩玩，順便來試用已經想玩很久的平台 [ObservableHQ](https://observablehq.com/)，這篇文章就記錄一下使用的過程，來介紹一下大家可以如何利用它來製作視覺化專案！

## ObservableHQ

D3.js 的作者 [@mbostock](https://twitter.com/mbostock)，在 2017 年的時候發了一篇名為 [A Better Way to Code](https://medium.com/@mbostock/a-better-way-to-code-2b1d2876a3a0)) 的文章，介紹了他當時正在製作的專案 - `d3.express`，也就是現在的 [ObservableHQ](https://observablehq.com/)。

<!-- 介紹目的 -->
<!-- 從我的 badge 開始介紹語法 -->
<!-- 帶出一些相關介紹連結 -->
<!-- -->

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化
