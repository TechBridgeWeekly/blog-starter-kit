---
title: ecto 簡介 (1) – cell 與 plasm
date: 2016-06-08 20:30:13
tags: ROS, ecto 
author: pojenlai
---

## 前言

這次想要跟大家介紹 ROS ORK (Object Recognition Kitchen) 這個函式庫實作物體辨識 pipeline 的機制，其背後運用到的一個重要的函式庫叫做 [ecto](http://plasmodic.github.io/ecto/index.html) ，接下來會介紹 ecto 的基本觀念和用法。但因為我想把內容講解得比較詳細，所以不會只花一篇文章的篇幅就介紹完這個工具，這一篇會講到最基本的 cell 與 plasm，讓大家先有初步的認識，更進階的用法甚至是實例會在之後的文章介紹。

## ecto 是什麼 & 為什麼要使用 ecto

可以把 ecto 想成一個框架，這個框架可以讓你很方便地把程式用 DAG (Directed Acyclic Graph) 的方式來實作，這樣實作的兩大好處在於模組化跟彈性。

DAG的一個範例圖：

![DAG](/img/pojenlai/dag.png)

首先談到模組化，在 ecto 的世界裡，你首先可以用 C++ 或 Python 寫出一個個的 cell ，這個 cell 就是執行一個功能的單位 (所以命名為 cell，細胞的意思)，這種設計的方式讓你在撰寫 cell 的時候，比較不會把好多個功能硬寫在一起，增加了程式的可讀性、也讓後續的擴充彈性變強。

模組化所衍生出的好處就是彈性，因為你只要抽換某個模組，就可以改變整個程式的行為。

以 ORK 為例，在撰寫物體辨識的 pipeline 時，假設有三個步驟: (這邊只是為了方便理解舉例，不是真實情況)

1. 讀取 Kinect 影像
2. 使用 Linemod 演算法進行 template matching
3. 將辨識結果輸出

那用 ecto 實作就會寫成三個 cells，然後再建立一個 ecto 的 plasm，plasm 其實就只是 graph，把寫好的三個cell相連接。 所以，如果我想要實作另一個物體辨識的演算法，我只要改寫第二個 cell 就好，當然有個前提是兩個演算法的 input 和 output 要一致，才不會影響到第一個和第三個 cell。

## 基本中的基本 – Cells & Plasm 的簡單用法

為了讓大家有見樹又見林的感覺，我們先看一下 ecto 大致上要怎麼用。最簡單的用法大概就是建立兩個 cell ，再用一個 plasm 將這兩個 cell 串成 graph 。

現在我們只要先知道要寫一個 cell 需要在裡面定義四個函式: 

```cpp
static void
declare_params(tendrils&)
static void
declare_io(const tendrils&, tendrils&, tendrils&)
void
configure(const tendrils&, const tendrils&, const tendrils&)
int
process(const tendrils&, const tendrils&)
```

這四個函式顧名思義就是要定義這個 cell 有哪些參數可以設定、輸入跟輸出是什麼、怎麼設定參數以及 cell 運作時的功能，很符合直覺上的需求。

那假設我們已經定義了兩個 cell – MyAwesomeCell1 跟 MyAwesomeCell2，ru, 剩下的就是建立一個 Plasm 來串接這兩個 cell 並執行，他的程式碼會像這樣:

```python
#!/usr/bin/env python
import ecto
import my_awesome_cpp_ecto_module
import my_awesome_python_ecto_module
 
# create a plasm
plasm = ecto.Plasm()
 
# create some cells
cell1 = my_awesome_cpp_ecto_module.MyAwesomeCell1(param1=whatever1)
cell2 = my_awesome_python_ecto_module.MyAwesomeCell2(param2=whatever2)
 
# connect those cells in the plasm
plasm.connect(cell1['output'] >> cell2['input'])
 
# execute the graph
plasm.execute(niter=2)
```

重點其實只有 Plasm 的初始化、串接 cell 成 graph 與執行這三個部分:

```python
# create a plasm
plasm = ecto.Plasm()
 
# connect those cells in the plasm
plasm.connect(cell1['output'] >> cell2['input'])
 
# execute the graph
plasm.execute(niter=2)
```

## Cell 的機制詳解

從上面的例子，大家應該可以明顯的看出，plasm 因為只是要串連寫好的 cell ，所以設定相對單純(目前我們先不討論 scheduling 等複雜的狀況)，但 cell 就不太一樣，上面只提到需要寫四個函式，卻沒有實例讓大家了解怎麼實作，接下來就會介紹比較實際的例子，讓大家了解 cell 要怎麼寫。

首先我們看個簡單的例子，這個 class 是一個 Printer ，我們在產生這個 Printer 的 instance 時，可以設定裡面的兩個 data member – prefix_ 和 suffix_ (或可以理解成參數)

```cpp
struct Printer
{
  Printer(const std::string& prefix, const std::string& suffix)
      :
        prefix_(prefix),
        suffix_(suffix)
  {
  }
  void
  operator()(std::ostream& out, const std::string& message)
  {
    out << prefix_ << message << suffix_;
  }
  std::string prefix_, suffix_;
};
```

假設要改寫成 ecto 的 cell，首先我們來宣告參數，透過 declare_params 這個函式可以做到，在這個例子裡，只是先單純宣告有兩個 params，提供這兩個參數的說明(這個說明可以用來自動生成文件，不過我們先忽略)，以及預設值。

大家可能會疑惑的地方是，params.declare 這個用法怎麼突然就跑出來了。這是因為在 ecto 裡面，cell 之間的溝通是透過 tendrils 這個類別來處理，但這一篇先不提到 tendrils 的細節，所以才會有點混亂，不過如果去看 [tendrils 的 API](http://plasmodic.github.io/ecto/api/html/classecto_1_1tendrils.html#a89e7ed34481d3ed987ddfd5ba35174ae)，就會清楚這中間是怎麼一回事。

```cpp
static void
declare_params(tendrils& params)
{
  params.declare<std::string>("prefix", "A string to prefix printing with.", "start>> ");
  params.declare<std::string>("suffix", "A string to append printing with.", " <<stop\n");
}
```

接著我們來定義 IO 的介面，透過 declare_io 來做，因為這個 cell 只需要接收需要印出的 message ，所以只需要宣告一個 input ，不需要宣告 output 。

```cpp
static void
declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
{
  inputs.declare<std::string>("message", "The message to print.");
}
```

目前我們已經指定了對外的兩個重點 – 有哪些參數以及 IO 介面。接著該考慮內部使用的設定了，所以第一步是將 declare_params 裡面宣告的參數 (此例中是 prefix 跟 suffix) 跟類別裡面的 data member (此例中是 prefix_ 跟 suffix_) 連接。

```cpp
void
configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
{
  params["prefix"] >> prefix_;
  params["suffix"] >> suffix_;
}
```

最後的重點就是，實作這個 cell 的功能，我們要把實際做的事情寫在 process 這個函式裡面。

```cpp
int
process(const tendrils& inputs, const tendrils& outputs)
{
  std::cout << prefix_ << inputs.get<std::string>("message") << suffix_;
  return ecto::OK;
}
```

所以如果把四個函式合起來看，就會像這樣:

```cpp
#include <ecto/ecto.hpp>
#include <ecto/registry.hpp>
#include <iostream>
#include <string>
using ecto::tendrils;
namespace overview
{
  struct Printer01
  {
    static void
    declare_params(tendrils& params)
    {
      params.declare<std::string>("prefix", "A string to prefix printing with.", "start>> ");
      params.declare<std::string>("suffix", "A string to append printing with.", " <<stop\n");
    }
    static void
    declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
      inputs.declare<std::string>("message", "The message to print.");
    }
    void
    configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
      params["prefix"] >> prefix_;
      params["suffix"] >> suffix_;
    }
    int
    process(const tendrils& inputs, const tendrils& outputs)
    {
      std::cout << prefix_ << inputs.get<std::string>("message") << suffix_;
      return ecto::OK;
    }
    std::string prefix_, suffix_;
  };
}
ECTO_CELL(ecto_overview, overview::Printer01, "Printer01",
          "A simple stdout printer with prefix and suffix parameters.");
```

雖然整個類別被寫成很長，但其實只要熟悉 cell 的基本用法，就不會被這一堆程式碼嚇到。最後想跟大加補充說明一下，上面提到的 cell 寫法有個不直覺的地方是，沒有明顯的繼承關係，所以會覺得不符合我們的 cell 應該要繼承一個 base 的 cell class 的直覺。其實這邊是被 ECTO_CELL 這個 Macro 給處理掉了，所以才會看起來只有宣告幾個函式就寫完一個 cell 的感覺。

## 總結

這篇文章簡介了 ecto 的 cell 和 plasm，下一篇將會介紹 tendril 跟 scheduler 的機制，幫助大家更加理解 ecto ，並在未來能運用這個框架來建立自己的應用。

## 延伸閱讀

1. [When to use DAG (Directed Acyclic Graph) in programming?](http://programmers.stackexchange.com/questions/171671/when-to-use-dag-directed-acyclic-graph-in-programming)
2. [ecto 官方網頁的 plasm 介紹](http://plasmodic.github.io/ecto/ecto/usage/tutorials/plasms.html)
3. [ecto 官方網頁的 cell 詳細介紹](http://plasmodic.github.io/ecto/ecto/overview/cells.html)
