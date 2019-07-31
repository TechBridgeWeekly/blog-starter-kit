---
title: Nengo AI 入門
date: 2019-05-10 16:51:34
tags:
  - Nengo AI
  - Neural Network
  - Deep Learning
author: pojenlai
---

## 前言

今天要跟大家介紹一個叫做 Nengo AI 的 Python 函式庫，這個函式庫主要提供的功能就是讓人可以建立分散式的 AI 系統，可以想像成更加龐大、功能也更強大的 AI。大家可以看一下介紹影片：

[![video](https://img.youtube.com/vi/zGpwSeMtCQc/0.jpg)](https://www.youtube.com/watch?v=zGpwSeMtCQc)

## 安裝

安裝的步驟滿簡單的，一行 `pip install nengo` 就搞定。

```
(C:\Users\rosindigo\Anaconda3\envs) C:\Users\rosindigo\PycharmProjects\nengo>pip install nengo
Collecting nengo
Using cached https://files.pythonhosted.org/packages/f7/ce/e314e1176bfbbe6c3b6cf4e8fa0620cafad8f8bad0
4203c55881e9cb2fb0/nengo-2.8.0-py2.py3-none-any.whl
Collecting numpy>=1.8 (from nengo)
Downloading https://files.pythonhosted.org/packages/2e/11/f006363050b24fb19a235e5efd219e7ac549398d531
110d80b8f2ba3a909/numpy-1.16.3-cp36-cp36m-win_amd64.whl (11.9MB)
|████████████████████████████████| 11.9MB 233kB/s
Installing collected packages: numpy, nengo
Successfully installed nengo-2.8.0 numpy-1.16.3
```

安裝完成之後，你可以先 import nengo 看看是不是可以成功：

```python
import nengo
model = nengo.Network()
```

因為 nengo 除了 core 之外，還有不同的 module，如果也想要安裝的話，可以參考下面的 command template：

```
pip install nengo[optional] # Additional solvers and speedups
pip install nengo[docs] # For building docs
pip install nengo[tests] # For running the test suite
pip install nengo[all] # All of the above
```

## 簡單理解 Nengo 的架構

下面這張圖很清楚地展示了 Nengo 從 Model 層到 Hardware 層的各個 module，基本上你可以說只要用 Nengo，就可以開發出在各種硬體上執行的 AI 系統。

![img](https://i.imgur.com/uE2xACq.png)

## 程式範例

接下來就讓我們看一個非常簡單的範例小程式（我就直接把說明寫在註解裡面啦）：

```python
import nengo
import numpy as np
 
 
if __name__ == "__main__":
  # 創建一個新的 network
  # 在 Nengo 中，任何 model 都是被包含在 Network 中
  model = nengo.Network()
 
  # 建立一個有 40 個 neuron 的 Nengo object - my_ensemble
  # object 是 model 的一部分(你可以建多個 object)
  # 用 with model 是為了讓 Nengo 知道這個 object 屬於哪個 model
  with model:
    my_ensemble = nengo.Ensemble(n_neurons=40, dimensions=1)
 
    # sin_input_node 會輸出一個 sine 波訊號
    sin_input_node = nengo.Node(output=np.sin)
 
    # 將 sin_input_node 的輸出送給 my_ensemble
    nengo.Connection(sin_input_node, my_ensemble)
 
  # 一旦建立好 object，也指定了裡面的 data 要怎麼流動
  # 這時候就可以用 Probe 來指定要收集哪邊的 data
  my_probe = nengo.Probe(my_ensemble)
 
  # 要模擬之前，要先用 model 建立一個 Simulator
  sim = nengo.Simulator(model)
 
  # 跑模擬，並印出模擬結果
  sim.run(5.0)
  print(sim.data[my_probe][-10:])
```

如果想要對 Neural Engineering Framework 有更深入的了解，可以看看這篇介紹 - [A Technical Overview of the Neural Engineering Framework](http://compneuro.uwaterloo.ca/files/publications/stewart.2012d.pdf)。

## 總結

今天跟大家介紹了 Nengo 這個 AI 系統的開發框架，個人覺得這個框架的潛力還滿大的，有興趣的讀者可以參考延伸閱讀提供的更多教材，先學習建立複雜度更高的 model，再進一步做出自己想要的 model。

## 延伸閱讀

1. [Nengo Documentation](https://www.nengo.ai/documentation.html)
2. [NengoDL: Combining deep learning and neuromorphic modelling methods](https://arxiv.org/abs/1805.11144)
3. [Biospaun: a large-scale behaving brain model with complex neurons](https://arxiv.org/abs/1602.05220)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
