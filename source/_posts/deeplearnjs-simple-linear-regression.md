---
title: 用 Javascript 進行簡單線性迴歸分析
date: 2018-03-10 15:09:02
tags:
  - deeplearnjs
  - machine learning
  - linear-regression
  - javascript
---

# 前言
約莫兩個月前，我利用 [p5-deeplearn-js](https://github.com/ITPNYU/p5-deeplearn-js) 這套工具在 Chrome extension 中玩了一下機器學習的應用 (請參閱[前端也能玩 Deepleaning - 以 p5-deeplearn-js 為例](https://blog.techbridge.cc/2018/01/13/p5ML-deeplearnjs/))，當時雖然有去看他們的 source code，但對於底層 deeplearnjs 的部分並沒有仔細去了解，畢竟還是要對 ML 有根本的了解，才能讀得懂。

而隨著 Google 發布 [Machine Learning Crash Course](https://developers.google.com/machine-learning/crash-course/)，再度讓我有了研究 deeplearn.js 的念頭，但萬事起頭難，一開始不能太勉強自己，所以本文就先從最簡單的 linear regression 實作開始，來熟悉一下基本操作！

照慣例先來張 Demo 成果圖：

![Demo](/img/arvinh/deeplearnjs-demo-img.png) 

範例是從 [這篇文章](https://ithelp.ithome.com.tw/articles/10186905) 中獲得的靈感，假資料也是取自該處。

## 先進入假想情境！

想像一下你是一間飲料店老闆，在經濟不景氣的情況下，你需要嚴格控管你的進貨成本，因此，聰明的你開始觀察各種可能，最後發現天氣的變化與你的飲料銷售量有很大的關聯！

然而，氣象資料很好取得，但你需要的是能夠預測在哪種天氣下，你需要進多少貨，以此來控管每一次的進貨量。

也就是說，你需要一個公式！

這個公式要能夠讓你輸入一個特徵 X（也就是氣溫度數），接著透過運算後產生一個目標變數 Y（也就是飲料銷售量）。

能當到老闆，想必數學不會太差，整理一下後發現，我們只有一個自變量 `X`，以及因變量 `Y`，因此可以嘗試用一個簡單的線性函數來逼近這公式：

`Y = aX + b`

問題就來了，要怎麼樣找出適合的係數 `a` 與 `b` 呢？

## 先別急，讓我們先把資料整理一下

資料分析的第一步一定是先取得資料，身為飲料店老闆，要取得銷售資量應該不難，再上網查個氣象資訊就好，因此可以得到如下資料：

```js
// 建構資料 （當然是假的）
const degrees = [29, 28, 34, 31, 25, 29, 32, 31, 24, 33, 25, 31, 26, 30];
const salesVolume = [77, 62, 93, 84, 59, 64, 80, 75, 58, 91, 51, 73, 65, 84];
```

資料是成對關係，`氣溫 29 度`，`冰紅茶銷售量 77`。

接著我們可以先利用 HighCharts 把資料先視覺化出來，你就會很有感覺我們該做什麼。

<p data-height="468" data-theme-id="dark" data-slug-hash="PRweww" data-default-tab="result" data-user="arvin0731" data-embed-version="2" data-pen-title="DeeplearnJS-simple-linear-dataset" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/PRweww/">DeeplearnJS-simple-linear-dataset</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

嗯...看起來這些點似乎真的可以用一個線性函數去逼近呢！而且更棒的是，透過這些過往資料，應該是有助於我們找出公式中最棒的係數的！

## 接著就該 Linear regression 就登場了！

在機器學習的領域中，像我們這樣依靠大量資料來輔助學習的，稱作監督式學習。

而當公式中的目標變數（也就是 y）是連續型（continuous）的變數時，這樣的學習問題會稱為回歸（regression）問題。

那一個線性函數的回歸問題，就是線性迴歸 (linear regression) 了。

關於 Linear regression 的一些數學公式推導等等，我推薦大家閱讀這篇文章：[史丹佛大學機器學習（Machine Learning）上課筆記（一）](https://blog.gtwang.org/statistics/standford-machine-learning-1/)

裡面解釋得淺顯易懂，絕對比我解釋得還好...

就算不清楚線性迴歸，光看公式應該也知道，我們可以透過大量的迭代測試，不斷輸入 `X`，然後調整 `a` 與 `b` 來求出 `Y`，然後找出在所有 input 中，能得到與所有真實資料差距最小的一對係數即可。

用線性回歸的概念來解釋，就是我們需要有一個 cost function（或稱 loss function），藉由不同係數的輸入，從公式得出的值來與實際資料比較，想辦法找出結果差距最小的係數。

一般來說，cost function 會用 LMS（least mean squares）演算法來處理，可以先猜一個初始係數值，然後藉由 LMS 來不斷更新係數，直到找出最小差距的情況。

但這是單一 training sample 的狀況，當有多個 training sample 時，通用的演算法稱為 stochastic gradient descent (sgd)，概念雷同，就像是根據每一筆 training sample 去跑一次 LMS。

一樣，還是請大家去閱讀 [史丹佛大學機器學習（Machine Learning）上課筆記（一）](https://blog.gtwang.org/statistics/standford-machine-learning-1/)，獲得更正確的概念。畢竟這次主要是練習如何用 deeplearnjs 來實作。

## 解法都有了，那就來看看到底怎麼用 Javascript 實作吧！

要使用 deeplearn.js，只需要從 cdn 載入即可：

```js
<script src="https://unpkg.com/deeplearn@latest"></script>
```

接著，我們要先初始化資料：

```js
// 建構資料
const degrees = [29, 28, 34, 31, 25, 29, 32, 31, 24, 33, 25, 31, 26, 30];
const salesVolume = [77, 62, 93, 84, 59, 64, 80, 75, 58, 91, 51, 73, 65, 84];
// 運用 Deeplearn.js 結構化資料
// dl 就是 deeplearn.js 的 global 變數
const degrees_data = dl.tensor1d(degrees);
const salesVolume_data = dl.tensor1d(salesVolume);
```

在 deeplearn.js 中，tensor 是最核心的資料結構，用來表示向量、矩陣或是多維度的資料。

有許多 utility function 可以輔助創建 tensor 資料結構，像是這邊用的是 `tensor1d`，也就是 1D (1-dimension) 的 tensor。

一個 tensor 其實包含三個成分，也是創建 tensor 時可以傳入的參數：

* values (TypedArray|Array): tensor 的值。可以是 nested array 或 flat array 的結構。

* shape（number[]）:基本上就是該 tensor 的維度。若創建 tensor 時沒有指定維度，就會繼承傳入的 values 維度。也可以像這邊的範例一樣直接使用 `tensor${1|2|3|4}d` 來創建

* dtype（float32'|'int32'|'bool）：值的型別，當然是 optional。

初始化好 deeplearn.js 的資料結構後，接著定義我們要 training 的係數，這邊取為 `aw` 與 `ba`：

```js
// 要 train 的參數 aw, ba
const aw = dl.variable(dl.scalar(Math.random()));
const ba = dl.variable(dl.scalar(Math.random()));
```

`dl.variable(initialValue, trainable?, name?, dtype?)` 用來創建 training 過程中需要的變數，也可透過參數指定該變數能否在 training 過程中被修改（trainable），預設是 `true`。

其中 `initialValue` 可以是一個 Tensor，也可以像我們這邊一樣，傳入 `dl.scalar`。

`dl.scalar` 是維度為 0 的 tensor，基本上 0 維就是一個點，由於我們只需要亂數產生一個初值給 `aw` 與 `ba`，因此用 `dl.scalar` 即可。

再來，我們定義目標函數與 loss function

```js
// 定義目標函數 與 loss function （最一般的 mean square）
// f = aw * X + ba
const f = x => aw.mul(x).add(ba);
const loss = (pred, label) => pred.sub(label).square().mean();
```

在 deeplearn.js 中，函數的建構蠻直覺的，`y = a * x + b` 可以直接寫成 ` y = x => a.mul(x).add(b)`

loss function 則是接收兩個參數的函式，`pred` 就是 training data 中的自變因 `x` 透過目標函數 `f`，計算出的值，而 label 是 training data 中的答案 `y`。

然後，定義我們的 Optimizer，也就是用來最佳化 loss function 結果的演算法，這邊採用 `stochastic gradient descent (sgd)`：

```js
// 採用 stochastic gradient descent 來做最佳化 
// learning rate 這邊不能設太大
const learningRate = 0.0005;
const optimizer = dl.train.sgd(learningRate);
```

`dl.train.sgd` 是 deeplearn.js 內建的 sgd 演算法模型，接受一個 `leanring rate` 參數。在每一次的迭代中，係數都會不斷被更新，以找出最佳的結果，而這個 `learningRate` 參數是用來控制每一次的更新幅度。因此不能夠設得太大，也不能設得太小。

利用定義好的目標函數 `f`、`loss function` 與 `optimizer`，我們可以就開始 training modal 了！

```js
// training
for (let i = 0; i < 30; i++) {
  const cost = optimizer.minimize(() => loss(f(degrees_data), salesVolume_data), true, [aw, ba])
  console.log('cost');
  cost.print();
  console.log('aw');
  aw.print();
  console.log('ba');
  ba.print();
}
```

最核心的就是這一行：

`const cost = optimizer.minimize(() => loss(f(degrees_data), salesVolume_data), true, [aw, ba])`

使用 `optimizer` 的方式如上所示，輸入執行 `loss function` 的函示，並可額外輸入兩個參數，分別控制 1. 是否回傳最後的 cost; 2. 限制只更新哪些變數。

我們簡單 for loop 30 次（因為資料量很少...你要 train 一百次也行，但這邊結果不會有什麼差別）

過程中若想要 debug，可以像我一樣用 `tensor.print()` 的方式把 traning 過程的係數變化 log 出來。

最後，當 training 結束後，透過 `dataSync()` 將最終係數從 Tensor 中讀出：

```js
// 利用 dataSync() 取出 training 後得到的係數
const awPredict = aw.dataSync();
const baPredict = ba.dataSync();
```

很容易看出 `dataSync()` 是 Synchronously 的，會 block Browser 的 UI thread，直到 data 被你讀出。另外還有個 Asynchronously 的 `data()` method，會回傳 promise，當讀取結束時再呼叫 `resolves`。

在我們的範例中，因為接下來要用 Highcharts 畫圖，我們需要 block 著 UI thread 等資料讀出後再繼續，因此採用 `dataSync()`。

取出係數的值後，就能夠畫圖啦～

根據算出的係數，畫出線條頭尾兩點：
```js
const dataLine = [
    [22, parseFloat(22 * awPredict + baPredict)],
    [35, parseFloat(35 * awPredict + baPredict)]
]
```

然後用 HighCharts 繪圖：
```js
const options = {
    title: {
        text: 'deeplearn.js  最高溫與紅茶銷售量'                 
    },
    xAxis: {
      title: {
        text: '氣溫'                 
      },
      min: 20,
      max: 40
    },
    yAxis: {
      title: {
        text: '銷售量'                 
      },
      min: 40,
      max: 100
    },
    series: [
        {
            type: 'line',
            name: 'predict line',
            data: dataLine
        },  
        {
            type: 'scatter',
            name: 'dataset',
            marker: {
                symbol: 'cross',  
                radius: 4         
            },
            data: dataset
        }
    ]
}
// 繪製圖表
const chart = Highcharts.chart('app', options);
```

## 最終成果

<p data-height="472" data-theme-id="dark" data-slug-hash="dmPZdj" data-default-tab="result" data-user="arvin0731" data-embed-version="2" data-pen-title="DeeplearnJS-simple-linear" class="codepen">See the Pen <a href="https://codepen.io/arvin0731/pen/dmPZdj/">DeeplearnJS-simple-linear</a> by Arvin (<a href="https://codepen.io/arvin0731">@arvin0731</a>) on <a href="https://codepen.io">CodePen</a>.</p>
<script async src="https://static.codepen.io/assets/embed/ei.js"></script>

## 結論

初次使用 deeplearn.js 其實主要門檻還是在於對 Machine Learning 的了解不夠，我有稍微比對一下用 python 與 R 的做法，其實都很雷同，包含資料向量的概念都是，加上很多演算法在這些語言的 library 中都有內建，相信對 ML 與 DL 有深入了解的人來學習 deeplearn.js 會更得心應手。deeplearn.js 畢竟有 Google 支持，也支援 TensorFlow 的 modal，還是很值得投資學習的，想想未來能用別人的 browser 來 training 你的 modal 有多好啊！喔不，是想想未來 Web 能因此有更多的發展潛力，多好呀！


<!-- 資料來源 -->
## 資料來源
1. [GitHub Deeplearnjs](https://github.com/PAIR-code/deeplearnjs)
2. [Deeplearn js API doc](https://deeplearnjs.org/docs/api/index.html#dl.train.sgd)
3. [機器學習 玩具資料與線性迴歸](https://ithelp.ithome.com.tw/articles/10186905)
4. [史丹佛大學機器學習（Machine Learning）上課筆記](https://blog.gtwang.org/statistics/standford-machine-learning-1/)
5. [deeplearn.js的線性迴歸](http://blog.csdn.net/ns2250225/article/details/79414790)
6. [8 大JavaScript 機器學習框架之探索](http://mp.weixin.qq.com/s/X0k2JPze7x8nkSxkoHtWnw)

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化