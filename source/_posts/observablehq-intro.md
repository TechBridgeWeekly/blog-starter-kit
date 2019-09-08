---
title: 從製作 visfest 2019 badge 認識 observable
date: 2019-09-09 02:01:00
tags:
  - d3.js
  - javascript
  - visualization
---

## 前言


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

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化
