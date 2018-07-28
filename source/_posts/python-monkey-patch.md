---
title: Python Monkey Patch 入門教學
date: 2018-07-14 09:54:49
author: kdchang
tags: Python
      猴子補丁
      Monkey Patch
---

![ 如何使用 Python Monkey Patch）](/img/kdchang/python-tips/monkey-patching.png)

# 前言
Python 是近來十分火紅的程式語言，不管是網站開發、遊戲開發或是資料科學都可以看見 Python 的身影，本系列文章將透過複習 Python 小 tips，讓讀者可以重新認識 Python。這次我們先來認識一下 Monkey Patch 這個程式設計上的小技巧。

# 什麼是 Monkey Patch
簡單來說，Monkey Patch 就是在 run time 時動態更改 class 或是 module 已經定義好的函數或是屬性內容。實務上常見的使用在 test 上用來 mock 行為或是 gevent 函式庫等。

```
A MonkeyPatch is a piece of Python code which extends or modifies other code at runtime (typically at startup).
```

# 製作第一個 Python Monkey Patch
我們這邊開了一個簡單的 monkey.py 檔案，裡面放置 demo 用的 class：MyClass，裡面有個 method 叫 f，會列印出 `f()` 字串，

```py
# monkey.py
class MyClass:
    def f(self):
        print 'f()'
```

此時我們在另外一個 main.py 檔案引入 monkey module，並替換掉 f function 為 monkey_f，這樣定義的出來的 instance 呼叫方法 f() 印出的則為 `monkey_f()`

```py
# main.py
import monkey

def monkey_f(self):
    print 'monkey_f()'
 
monkey.MyClass.f = monkey_f
obj = monkey.MyClass()
obj.f()
# monkey_f()
```

# 總結
以上就是 Python Monkey Patch 簡單介紹，Python Monkey Patch 主要用於動態於 run time 改變 class 的屬性或是方法，實務上常見的使用在 test 上用來 mock 行為或是 gevent 函式庫等地方。

## 延伸閱讀
1. [源码分析之gevent monkey.patch_all实现原理](http://xiaorui.cc/2016/04/27/%E6%BA%90%E7%A0%81%E5%88%86%E6%9E%90%E4%B9%8Bgevent-monkey-patch_all%E5%AE%9E%E7%8E%B0%E5%8E%9F%E7%90%86/)
2. [What is monkey patching?](https://stackoverflow.com/questions/5626193/what-is-monkey-patching)
3. [Monkey-patching in Python: a magic trick or a powerful tool?](https://www.slideshare.net/ElizavetaShashkova/monkeypatching-in-python-a-magic-trick-or-a-powerful-tool)

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter, Software Engineer & Maker. JavaScript, Python & Arduino/Android lover.:)

（image via [semaphoreci](https://semaphoreci.com/community/tutorials/mocks-and-monkeypatching-in-python)）
