---
title: 如何使用 Python 進行字串格式化
date: 2019-05-03 20:23:23
author: kdchang
tags: 
    - Python
    - string format
    - how-to
    - tutorial
    - Formatted String Literal
    - string
    - Template String
    - 字串格式化
---

![如何使用 Python 進行字串格式化](/img/kdchang/python101/mac-cover.jpg)

# 前言
在開發應用程式時我們往往會需要把變數進行字串格式化，也就是說把字串中的變數替換成變數值。事實上，在 Python 中有許多方式可以進行，其中最常見的有四種方式（斯斯有三種，但 Python 字串格式化方式竟然有四種！）：

1. 舊式字串格式化
2. 新式字串格式化
3. 字串插值
4. 樣板字串

字串插值是在 Python 3.6 之後有支援的方法，若是你的版本是在 Python 3.6 之後的話建議可以使用。若是需要讓使用者可以輸入變數來轉換成字串格式化的話，建議可以使用樣板字串來避免一些資訊安全上的問題。

以下就上述提到的四種方法來各自說明其特色和使用方式：

# 舊式字串格式化（%）
相對於 Python 版本之後推薦使用的新式字串格式化，舊式版本使用 `%` 運算子來進行字串格式化，若是有 C 語言撰寫經驗的讀者或許會覺得的似曾相似（是不是有點像 printf？）。使用 `%` 格式是告訴 Python 直譯器要在那邊替換文字 text 並使用字串呈現。這就是所謂的舊式字串格式化（%s 是以字串輸出，%f 是以浮點數輸出、%d 是以十進位整數輸出）：

```py
text = 'world'
print('hello %s' % text)
# hello world
```

若是希望把內容轉成十六進位的話可以使用：

```py
print('%x' % 23)
# 17
```

若是有多個變數要替換則使用 tuple 傳遞需要替代的內容值：

```py
print('hello %s %s' % ('world', 'go'))
# hello world go
```

# 新式字串格式化（format()）
在 Python3 以後，開始引進新串格式化，也就是使用 `format()` 函式來讓字串格式化，其功能和舊式格式化相差無幾，但主要是捨去 `%` 讓字串格式化使用上可以更加正常、規律，可讀性也相對提升。

一般基本用法：

```py
text = 'world'
print('hello {}'.format(text))
# hello world
```

也可以使用名稱來指定變數變換順序：

```py
name = 'Jack'
text = 'world'

print('hello {name}, hello {text}'.format(name=name, text=text))
# hello Jack, hello world
```

若是希望把內容轉成十六進位的話可以使用 format spec 在 `{}` 新增 `:x`：

```py
print('{:x}'.format(23))
# 17
```

# 字串插值（Formatted String Literal）
雖然已經有了新式字串格式化，然而在 Python 3.6 又新增了格式字串字面值（Formatted String Literal）此一作法可以把 Python 運算式嵌入在字串常數中。
眼尖的讀者可能會發現，咦，怎麼跟隔壁棚的 JavaScript ES6 字串模版有點像呀？

現在我們來看一下一般的使用方式：

```py
text = 'world'
print(f'Hello, {text}')
```

新的字串插值語法相當強大的點是，可以在裡面嵌入任何 Python 的運算式，舉例來說，我們想要呈現整數相加：

```py
x = 10
y = 27

print(f'x + y = {x + y}')
# 37
```

同樣，若是希望把內容轉成十六進位的話可以使用 format spec 在 `{}` 新增 `:x`：

```py
print('{:x}'.format(23))
# 17
```

讀者可能會覺得很字串插值神奇，但事實上其背後原理是由 Python 語法解析器把 f-string 字串插值格式字串轉成一連串的字串常數和運算式，最後結合成最終的字串。

```py
def hello(text, name):
    return f'hello {text}, hello {name}'

# 實際上 Python 會把它變成字串常數和變數（過程中有優化）

def hello(text, name):
    return 'hello ' + text + ', hello' + name
```

# 樣板字串（Template String）
樣板字串（Template String）機制相對簡單，也比較安全。

以下是一般的使用情境，需要從 Python 內建模組 string 引入：

```py
from string import Template

text = 'world'
t = Template('hello, $text')
t.substitute(text=text)
# hello, world
```

然而若是希望把內容轉成十六進位的話需要自己使用 hex 函式自己轉換：

```py
from string import Template

number = 23
t = Template('hello, $number')
t.substitute(number=hex(number))
# hello, 0x17
```

由於其他的字串格式化功能較為強大，所以反而會造成惡意使用者輸入變數替換成字串時造成不可預期的錯誤（一般來說使用者的輸入都是不可信的，要進行過濾）。

舉例來說惡意使用者可能可以透過字串格式的惡意輸入來獲取敏感資訊（例如：密碼、token、金鑰等）；

```py
SECRET_TOKEN = 'my-secret-token'

# Error func
class Error:
    def __init__(self):
        pass

err = Error()
malicious_input = '{error.__init__.__globals__[SECRET_TOKEN]}'
malicious_input.format(error=err)
# my-secret-token
```

沒想到，透過字串格式的方式竟然可以透過 `__globals__` 字典取出我們的 SECRET_TOKEN，若是一不留神，很可能機密資料就洩漏出去。此時若是使用 Template String 則會發生錯誤，是比較安全的選項：

```py
from string import Template

SECRET_TOKEN = 'my-secret-token'

# Error func
class Error:
    def __init__(self):
        pass

err = Error()
malicious_input = '${error.__init__.__globals__[SECRET_TOKEN]}'
t = Template(malicious_input)
t.substitute(error=err)
# ValueError: Invalid placeholder in string: line 1, col 1
```

# 總結
雖然 Python 的信仰是能用簡單唯一的方式來完成任務，然而字串格式化卻有多種方式，也各有其優缺點，其主要原因或許在於版本不同變遷所致。所以你有可能在公司內部專案不同專案看到使用不同的字串格式化方式，若是看到同一個專案使用不同字串格式化方式也不要驚訝。

一般情況我們會根據不同 Python 版本和使用情境去使用不同字串格式化方式，例如：若是使用 Python 3.6 之後的話建議可以使用字串插值，若版本比 3.6 舊，則使用新式字串格式化（format()）。若是需要讓使用者可以輸入變數來轉換成字串格式化的話，建議可以使用樣板字串來避免一些資訊安全上的問題。

# 參考文件
1. [python string — Common string operations](https://docs.python.org/3/library/string.html)
2. [(那些過時的) Python 字串格式化以及 f-string 字串格式化](https://blog.louie.lu/2017/08/08/outdate-python-string-format-and-fstring/)
3. [字串格式化](https://openhome.cc/Gossip/Python/StringFormat.html)
4. [Python String Formatting Best Practices](https://realpython.com/python-string-formatting/)
5. [Python 字串格式化教學與範例](https://officeguide.cc/python-string-formatters-tutorial/)
6. [A Quick Guide to Format String in Python](https://www.techbeamers.com/python-format-string-list-dict/)

（image via [unsplash](https://unsplash.com/photos/HvYy5SEefC8?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText)


關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
