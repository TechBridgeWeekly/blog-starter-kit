---
title: Python Decorator 入門教學
date: 2018-06-15 09:54:49
author: kdchang
tags: Python
      Decorator
      裝飾器
---

![ 如何使用 Python 學習機器學習（Machine Learning）](/img/kdchang/python-tips/decorator_tutorial_code.png)

# 前言
Python 是近來十分火紅的程式語言，不管是網站開發、遊戲開發或是資料科學都可以看見 Python 的身影，本系列文章將透過複習 Python 小 tips，讓讀者可以重新認識 Python。這次我們先來認識一下 Decorator 這個看似怪異但卻常被使用的設計模式。

# 什麼是 Decorator
簡單來說 Decorator 程式語言的設計模式，也是一種特殊的 function（例如：把 function 當做參數傳入，再把 function 傳回），透過 Decorator 可以將加上 Decorator 的 function 加上更多能力，重用許多程式碼。而在 Python 中我們則是使用 `@` 當做 Decorator 使用的語法糖符號。

# 製作第一個 Python Decorator
透過一個簡單抽象化例子，我們可以一窺 Python Decorator 的樣貌：

```py
@my_decorator
def my_func(stuff):
    do_things()
```

我們可以看到 `@my_decorator` 這個 Decorator 語法糖被加在 `my_func` 之上。而上面的程式碼其實等於，將 my_func 當做參數傳入 my_decorator 中：

```py
def my_func(stuff):
    do_things()

my_func = my_decorator(my_func)
```

看起來 Decorator 好像蠻方便的。但讀者內心一定會開始思考究竟 Decorator 常用嗎？或是會有了使用在哪些地方？等問題。事實上在實務上，Python 應用程式有許多地方都可以看到 Decorator 使用的蹤影，舉凡登入驗證、日誌 logging 等地方。

下面是一個簡單範例，主要是讓每次使用者在瀏覽 payment 頁面時流量要檢查使用者是否有登入進行頁面權限管理，若沒有則回傳 403 沒有權限訪問，若有則繼續往後送去 render 出頁面。
由於很適合使用 Decorator 來撰寫的情境，所以可以建立一個 Decorator 來當做每次是否呈現頁面的權限管理機制。

```py
from flask import abort
# wraps 是一個製作 Decorator 好工具，也可以把參數傳入
from functools import wraps

def check_login():
    # 做一些檢查

def login_required(fun):
    """
    Required user to login
    """
    @wraps(fun)
    # wraps 可協助傳入參數做操作
    def wrapper(*args, **kwds):
        if check_login() is None:
            return abort(403)

        return fun(*args, **kwds)
    return wrapper
```

Python Flask Web 應用程式的 routing（payment 頁面加上了 `@login_required`），代表有驗證登入的使用者才能看到該頁面：

```py
@payment_blueprint.route('/payment')
@login_required
def get_payment_page():
    return render_template('payment/index.html')
```

# 總結
以上就是 Python Decorator 簡單用法介紹，事實上關於 Python Decorator 還有許多進階應用可以去發掘，而中文的裝飾器翻譯又常常會讓人有所誤解，需要自己動手實作才能比較理解相關概念。本系列文章接下來也將透過複習 Python 小 tips，讓讀者可以重新認識 Python。

## 延伸閱讀
1. [理解 Python 装饰器看这一篇就够了](https://foofish.net/python-decorator.html)
2. [Advanced Uses of Python Decorators](https://www.codementor.io/sheena/advanced-use-python-decorators-class-function-du107nxsv)

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter, Software Engineer & Maker. JavaScript, Python & Arduino/Android lover.:)

（image via [kleiber](https://kleiber.me/blog/2017/08/10/tutorial-decorator-primer/)）
