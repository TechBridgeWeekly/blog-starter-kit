---
title: 初探 Probabilistic Models of Cognition
date: 2018-03-24 23:28:47
tags:
    - Artificial Intelligence
author: pojenlai
---

## 前言

最近這幾年，AI 的發展又再次變得蓬勃，是一次很好的機會，讓人們可以打造出極棒的科技來為眾人服務。近年最紅的是深度學習已經有很多相關研究，TB 周刊中也一直會為大家放上一些最新的發展，但是，筆者一直對潛藏在我們心智中的基石很有興趣，而這些基石，就跟今天想要介紹的 [Probabilistic Models of Cognition](https://probmods.org/) 很有關係。

## Probabilistic Models of Cognition 的基本想法

如果我們將人的腦袋想成一個 function，這個 function 可以做好多好多的事。例如看到一本好書、遇到一個好老師、聽到一門好課，我們會想要去把握、想要分享；例如我們看到一幅美麗的畫、一個壯麗的景觀，我們會發出讚嘆、想跟自己珍視的人一起欣賞；遇到千千萬萬種不同的 input，我們會有千千萬萬種不同的想法。

雖然我們可以處理千千萬萬種情境，產生千千萬萬種想法，但是，我們處理這千千萬萬種情境時，使用的基石可能很少。例如，我們只要知道：

- 物體有重量（基石 A）
- 抓住物體、抵抗重力才能拿起物體（基石 B）
- 用力拋出物體，物體會飛行（基石 C）
就可以丟出數以千計種物體（已簡化，還需要看得到物體位置、能夠操控手..等等條件）。

如果能掌握這些基石，那我們是不是就可以用這些基石來**產生**行為？利用基石來產生行為，就表示這些基石是 generative 的 model。

但是，這些基石又各自潛藏了一些變數，例如拋出去的力要多大才對？是否沒考慮到空氣阻力？所以裡面有一些我們未完整理解（也就是我們理解這個世界，形成的 model，還不完整），所以加入隨機的成分，這也是 **Probabilistic** Models of Cognition 名稱的由來。

## WebPPL

為了用比較正規的程式語言來描述我們心智中的基石（也就是 Probabilistic model），作者推出了一種程式語言，叫做 probabilistic programming for the web （WebPPL）。

我們可以使用這個語言來為世界建立簡單的模型，甚至跑模擬，另外，也可以利用裡面實作的推理功能來做簡單的推理。

我們目前只要知道這樣就好，語法那些的有興趣可以看看 [這個網站](http://webppl.org/)，基本上是由 Javascript 延伸而來。

## Generative model

>One view of knowledge is that the mind maintains working models of parts of the world. ‘Model’ in the sense that it captures some of the structure in the world, but not all (and what it captures need not be exactly what is in the world—just useful).

上面這段話是什麼意思呢，我們來看個小小的範例（下面的範例可以在 [conditioning](https://probmods.org/chapters/03-conditioning.html) 這一章節中看到）：

```javascript
// makes a floor with evenly spaced buckets
var bins = function (xmin, xmax, width) {
return ((xmax < xmin + width)
// floor
? {shape: 'rect', static: true, dims: [400, 10], x: 175, y: 500}
// bins
: [{shape: 'rect', static: true, dims: [1, 10], x: xmin, y: 490}].concat(bins(xmin + width, xmax, width))
)
}
// add two fixed circles
var world = [{shape: 'circle', static: true, dims: [60], x: 60, y: 200},
{shape: 'circle', static: true, dims: [30], x: 300, y: 300}].concat(bins(-1000, 1000, 25))
var randomBlock = function () {
return {shape: 'circle', static: false, dims: [10], x: uniform(0, worldWidth), y: 0}
}
physics.animate(1000, [randomBlock()].concat(world))
```

模擬跑起來會長得像這樣：

[![Forward](http://img.youtube.com/vi/oirC9Ae9e_Y/0.jpg)](https://www.youtube.com/watch?v=oirC9Ae9e_Y
"Forward")
除了模擬去推得結果，也可以從結果反推出

[![Inverse](http://img.youtube.com/vi/AvFtKw4fXXA/0.jpg)](https://www.youtube.com/watch?v=v
"Inverse")
從上面的兩例中可以看出，我們可以用很簡單的幾行 WebPPL 程式，就讓電腦可以做到自己要用其他程式語言來做，是有點難度的推理（當然，背後也是有物理模擬的黑盒子已經被包起來，所以我們不需要管那邊的細節）。

## 總結

今天我們稍微探討了 Probabilistic Models of Cognition，也玩了裡面的一個模擬功能，有興趣的讀者可以直接去看書裡的更多內容或是作者的論文。

## 延伸閱讀

1. [Josh Tenenbaum - The cognitive science perspective: Reverse-engineering the mind (CCN 2017)](https://www.youtube.com/watch?v=Z3mFBEOH2y4&t=9s)
