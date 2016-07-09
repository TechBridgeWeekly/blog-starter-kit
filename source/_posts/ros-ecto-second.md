---
title: ecto 簡介 (2) – tendrils 與 scheduler
date: 2016-07-09 09:34:49
tags: ROS, ecto
author: pojenlai
---

## 前言

上次的文章介紹了 ecto 的基本機制 – cell 與 plasm，讓大家對於這個函式庫的基本功能有了一個認識，但你可能會感到疑惑，如果只是可以用來建立一個如 DAG 方式來執行的 pipeline ，寫在 main 函式裡不就好了，何苦使用 ecto，還得把一個 class 寫成一個又臭又長的 cell。這篇文章就會帶出 tendril 和 scheduler 的機制，讓大家了解到事情並不像我們想得那麼簡單。

## tendril 是什麼 & tendril 好用在哪裡

tendril 這個 object 就是 cell 最基本的溝通媒介，舉凡 cell 的輸入、輸出以及 parameter 的傳遞都會使用到 tendril 。在上一篇我們也可以看到，cell 的函式定義裡面，都是使用 tendril 這個型態來傳遞參數。

```cpp
struct Example01 
{
  static void declare_params(tendrils& p)
  {
    p.declare<int>("value", "Some integer");
  }

  void configure(const tendrils& p, const tendrils& i, const tendrils& o)
  {
    int n = p.get<int>("value");
    std::cout << "Value of n is " << n << "\n";
  }
};
```

使用起來只要寫一小段 Python Script 就好：

```python
#!/usr/bin/python
 
import ecto
from ecto.ecto_examples import Example01
 
ecell = Example01(value=17)
ecell.process()
```

那統一寫成 tendril 的好處是什麼呢? 統一的好處很直觀，就是很容易被廣泛地使用，舉例來說，如果我們今天要寫一個 scheduler 來控制整個 plasm 的排程，那每個 cell 的參數、輸入和輸出都統一用 tendril 在實作上會比較簡單。舉個例子，cell 的 process(const tendrils&, const tendrils&)就一定是固定有兩個 const 的 tendrils 類型的引數，不會被使用者任意自訂。

這邊還有另一個容易讓人搞混的地方，就是 tendril 跟 tendrils 的差異，這兩個東西是不一樣的，tendrils 可以想成是一個裝著 tendril 的容器，可以給 tendril 的名字，這個名字會被當作一個 key （型態自然就是 string），用來對應到 tendril 的物件。

## Scheduler 是什麼

Scheduler 是幫忙控制 plasm 執行的工具，比較酷的是 ecto.MultiPlasmScheduler 這個類別，他可以讓我們輕鬆地平行執行多個 plasm ，例如我們有三個 plasm 要執行，且三個想要執行的頻率不一樣，那使用 MultiPlasmScheduler 可以透過設定 step 的大小來控制頻率。

![three_plasms](/img/pojenlai/three_plasms.jpg)

自己如果想實作，那就要跟 multi-threads 奮戰，中間細致的控制滿容易出錯的，如果 thread 一多，debug更是困難。

ecto 可以讓我們從比較上層的角度來控制多個 plasm ，有看到一個 test script 是使用 MultiPlasmScheduler 來執行兩個 thread，跟上面的範例圖不一樣，不過概念是相近的：

```python
#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
import ecto
import ecto.ecto_test as ecto_test
import sys
from ecto.test import test

def build_pipelines():

    left_plasm = ecto.Plasm()
    right_plasm = ecto.Plasm()
    accumulator = ecto_test.Accumulator("Accumulator", connected_inputs_only=True)
    left_generator = ecto_test.Generate("Left Generator", step=1.0, start=1.0, stop=10.0)
    right_generator = ecto_test.Generate("Right Generator", step=2.0, start=1.0, stop=10.0)
    left_plasm.connect(left_generator["out"] >> accumulator["left"])
    right_plasm.connect(right_generator["out"] >> accumulator["right"])
    plasms = {
          "left": left_plasm,
          "right": right_plasm
         }
    scheduler = ecto.MultiPlasmScheduler(plasms, disable_qt_management=True)
    return (accumulator, scheduler)


@test
def test_plasm():
    (accumulator, scheduler) = build_pipelines()
    scheduler.spin()
    print( "RESULT: %s" % accumulator.outputs.out)
    assert accumulator.outputs.out == 80.0

if __name__ == '__main__':
    test_plasm()
```

但是這一塊的 API 跟教學文章很不完整，要深入了解可能要看撰寫 ecto 的這些高手們怎麼把 ecto 用在 OpenCV、PCL 等開源的電腦視覺函式庫中，之後的文章會更深入探討。

## 總結

這一篇文章簡單跟大家介紹了 tendril 跟 scheduler 的概念，對於 ecto 的功能及彈性更加有感覺，不過看完這一篇可能還不太知道要怎麼活用 ecto，下一篇會帶大家實際來玩玩這個工具，會更通 ecto 可以做到的事跟實際應用的範例。

## 延伸閱讀

1. [ecto 官方網頁的 tendril 介紹](http://plasmodic.github.io/ecto/ecto/reference/tendril.html#tendril-overview)
2. [ecto 官方網頁的 scheduler 介紹](http://plasmodic.github.io/ecto/ecto/reference/schedulers.html#schedulers)
