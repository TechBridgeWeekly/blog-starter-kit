---
title: Python Web Flask 實戰開發教學 - 簡介與環境建置
date: 2017-06-03 12:00:00
author: kdchang
tags: Python, Django, MVC, Web, MTV, Web Backend, Web Framework, 教學, Flask, 框架, 網站開發, Anaconda
---

![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/python-logo.png) 

# 前言
Web 開發涉及層面很廣，包含了前後端開發、資料庫優化、平行處理、負載平衡、高可用性、資訊安全、雲端伺服器部屬等議題，本系列文章將透過介紹 Python Web Flask 實戰開發來學習現代化網站開發的方方面面。一開始先從簡介和環境建置開始吧！

# 什麼是 Python？
[Python](https://zh.wikipedia.org/wiki/Python) 是一種物件導向、直譯式的跨平台電腦程式語言，它包含了一組功能完備的標準庫和豐富套件生態系，可以輕鬆完成很多常見的任務（例如：讀寫檔案、自然語言處理、網路爬蟲、網站開發、機器學習等），因為它可以很輕易整合其他底層語言（例如：C/C++ 等），所以又稱為膠水語言。它的語法簡單，與其它大多數程式設計語言使用大括弧不一樣，它使用空白縮進來定義語句塊。由於具備簡潔易學等特性，許多開發者推薦 Python 為初學者第一個學習的程式語言。由於版本更迭，我們接下來討論的主要是以 Python3 為主，若電腦沒有安裝的話，你可以在[官方網站下載](https://www.python.org/)，若你不是安裝 [Anaconda](https://www.continuum.io/downloads) 這個 all-in-one 版本的話（自帶許多套件和科學運算工具，也可以建立虛擬開發環境），記得要安裝 [pip](https://pypi.python.org/pypi/pip)、[IPython](https://ipython.org/)。 

接下來我們將以 Anaconda 這個開發環境為主要講解環境。由於我們假定讀者已經有一些 Python 基礎，所以我們會跳過有關 Python 語法的基本教學。

# 什麼是 Flask？

![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/flask-index.png) 

[Flask](http://flask.pocoo.org/) 是一個使用 Python 撰寫的輕量級 Web 應用程式框架，由於其輕量特性，也稱為 micro-framework（微框架）。Flask 核心十分簡單，主要是由 [Werkzeug WSGI 工具箱](http://werkzeug.pocoo.org/)和 [Jinja2 模板引擎](http://jinja.pocoo.org/docs/2.9/)所組成，Flask 和 Django 不同的地方在於 Flask 給予開發者非常大的彈性（當然你也可以說是需要思考更多事情），可以選用不同的用的 extension 來增加其功能。相比之下，Django 雖然完善但技術選擇相對不彈性，不論是 ORM、表單驗證或是模版引擎都有自己的作法。事實上沒有最好的框架，只有合適的使用情境，Django 相比之下適合需要快速的開發大型的應用程式，和 Ruby 中的 [Ruby on Rails](http://rubyonrails.org/) 相似，而 Flask 則是相對輕量彈性，更像是 Ruby 界的 [Sinatra](http://www.sinatrarb.com/)。若讀者想先體驗看看 Flask 的程式狀況，以下是 Flask 簡易運行的程式，啟動測試伺服器後，可以在瀏覽器中（http://localhost:5000/）印出 `Hello World!`。

```python index.py
from flask import Flask
app = Flask(__name__)

@app.route("/")
def hello():
    return "Hello World!"

if __name__ == "__main__":
    app.run()
```

# Python Web 開發環境建置
所謂工欲善其事，必先利其器，要開發好的應用程式必須先準備好開發環境才行。以下介紹我們在接下來開發 Python Flask Web 應用程式所需要安裝的開發環境工具（以下以 Mac OS 等 Unix-like 作業系統為示範，若是 Windows 使用者建議安裝 Virtualbox 並運行 Linux Ubuntu 作業系統，[參考安裝文件](http://blog.xuite.net/yh96301/blog/432341564-VirtualBox+5.1%E5%AE%89%E8%A3%9DUbuntu+16.04)）：

1. Microsoft VSCode 編輯器
    Microsoft VSCode 是 Microsoft 推出的編輯器（非整合開發環境 IDE），夾帶著 Microsoft 過去打造 Visual studio 整合開發環境的豐富經驗，相比 Sublime Text 和其他編輯器，VSCode 的優勢在於開源且活躍的開發社群、內建 debugger 框架、原生 Git 整合、套件整合容易等特性。所以對於初學者來說 VSCode 是一個蠻適合入門的開發環境。它的安裝方式也十分簡易，在官網下載後按照指示安裝完成即可。

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/vscode-index.png) 

    我們可以安裝 Python 語法和格式檢查的相關 Python 套件幫助除錯：

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/vscode-python.png) 

2. 終端機環境（iTterm/內建 terminal）
    terminal（終端機）是我們下指令的地方，許多時候我們進行程式開發時不會使用 GUI 的介面而是使用下指令方式請電腦做相對應的行為。在 Linux 和 Mac 裡面都有內建的 terminal 的應用程式，若你是 Mac 用戶想使用更便利的工具（分割視窗、熱鍵、搜尋、自動補完等）可以額外安裝 [iterm2](https://www.iterm2.com/index.html) 做使用。

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/iterm2.png) 

    下指令（$ 為提示字元，不用輸入）：

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/iterm2-example.png) 

    對於有志於從事程式開發相關工作的讀者建議可以多熟悉指令碼的輸入，更多指令碼可以參考鳥哥撰寫的 [Linux 基本指令介紹](http://linux.vbird.org/linux_basic/redhat6.1/linux_06command.php) 和 [Linux 學習資源](http://www.linux.org.tw/resource.html)。

3. Git 版本控制系統/註冊 GitHub 帳戶 
    Git 是一種分散式版本控制系統，可以讓我們可以更方便地管理我們的程式碼。在網路上有非常多優秀的 Git 教學文件（[連猴子都能懂的Git入門指南](https://backlogtool.com/git-guide/tw/)、[寫給大家的 Git 教學](https://www.slideshare.net/littlebtc/git-5528339)、[初心者 Git 上手攻略](https://www.slideshare.net/lkiral/git-34157836)）。安裝 Git 方式是到官網下載軟體，依照指示安裝。

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/git-index.png) 

    互動式語法學習：

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/try-git.png) 

    在介紹完 git 之後我們來了解一下 GitHub。GitHub 是一個可以存放 git 程式碼專案的平台，透過 GitHub 我們可以接觸到最新的開放原始碼資訊，也可以將我們的程式碼開源出來。
    
    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/github-index.png) 

    從 GitHub 上複製程式碼
    
    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/github-clone.png) 

    ```
    # 複製一份到本地端
    $ git clone https://github.com/kdchang/flask101.git
    # 移動到資料夾
    $ cd flask101
    ```

    常見 Git 指令：

    ```
    # 初始化專案
    $ git init
    # 查看狀態
    $ git status
    # 檢查差異
    $ git diff 
    # 將變更檔案放入暫存區
    $ git add index.py
    # 使用 commit -m 提交變更
    $ git -a -m 'init commit'
    # 查看歷史
    $ git log
    # 放棄已經 commit 的檔案重回暫存區
    $ git reset HEAD index.py
    # 放棄檔案變更
    $ git checkout index.py
    ```

4. Anaconda Python3 版本
    Anaconda 是一個 all-in-one 的 Python 開發環境，對於初學者來說是個十分合適的開發環境包。Anaconda 具備了幾項特點：

    - 便於安裝許多流行的科學、數學、工程、數據分析的 Python 模組  
    - 開源和免費
    - 跨平台支持：Linux、Windows、Mac
    - 支持 Python 版本切換，方便建立不同的虛擬開發環境
    - 內建 Spyder 編輯器和 Jupyter Notebook 環境 

    安裝流程也十分簡單，進入 Anaconda 首頁，選擇對應作業系統（這邊使用 Mac OS）：
    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/anaconda-index.png)

    選擇對應 Python 版本下載，我們使用 Graphical Installer（圖像介面安裝方式），接著在下載完成時按照預設的安裝方式完成安裝；
    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/flask101/anaconda-install.png) 

    若是完整安裝成功，可以打開終端機輸入，若是顯示 Python 3.6.0 :: Anaconda 4.3.0 (x86_64) 版本號即安裝成功：

    ```
    $ python -V
    Python 3.6.0 :: Anaconda 4.3.0 (x86_64)
    ```

    接著要建立我們專案虛擬環境，這樣在安裝操作套件時比較不容易被污染到 root 的環境，啟動後會出現（套件名稱）的提示字元：

    ```
    # 顯示目前虛擬環境列表
    $ conda info -e 
    # 創建虛擬環境
    $ conda create -n 套件名稱 python=3.6
    # 進入虛擬環境（若是 Windows cmder 環境不用加 source） ，成功後提示字元變成：（套件名稱）$
    $ source activate 套件名稱
    # 離開虛擬環境（若是 Windows cmder 環境不用加 source） 
    $ source deactivate 
    ```

# 建立第一個 Flask 開發專案

## 使用 pip 管理 Python 模組

```
# 安裝模組
$ pip install 模組名
# 移除模組
$ pip uninstall 模組名
# 搜尋模組
$ pip search 模組名
```

通常開發專案時我們會將已安裝模組名稱和版本號存成一個列表，以便下次安裝使用：

```
$ pip freeze > requirements.txt
```

根據 `requirements.txt` 列表安裝模組：

```
$ pip install -r requirements.txt
```

## 建立虛擬環境
在了解 pip 套件管理工具後我們正式來建立一個 Flask 的專案：

建立專案資料夾

```
$ mkdir python-flask-todo-app
$ cd python-flask-todo-app
```

建立獨立虛擬環境

```
$ conda create -n python-flask-todo-app python=3.6
$ source activate python-flask-todo-app
```

安裝 Flask 模組

```
$ pip install flask
```

## 設定 Config 設定檔案

建立設定檔 `config.py` 檔案在專案資料夾的根目錄中：

```python
class Config(object):
    pass

class ProdConfig(Config):
    pass

class DevConfig(Config):
    DEBUG = True
```

建立一個 `main.py` 檔案初始化 Flask App 和使用其 API，若是一切正常在終端機執行 `python main.py` 會在瀏覽器網址列輸入 `localhost:5000` 後看到 `Hello World` 了！

```python main.py
from flask import Flask
from config import DevConfig

# 初始化 Flask 類別成為 instance
app = Flask(__name__)
app.config.from_object(DevConfig)

# 路由和處理函式配對
@app.route('/')
def index():
    return 'Hello World!'

# 判斷自己執行非被當做引入的模組，因為 __name__ 這變數若被當做模組引入使用就不會是 __main__
if __name__ == '__main__':
    app.run()
```

## 使用 Flask Script
[Flask Script](https://flask-script.readthedocs.io/en/latest/) 是 Flask 常用的 extensions，可以讓我們使用指令列來操作 Flask 程式並在 shell 環境下操作 app context，使用方式如下：

安裝模組

```
$ pip install flask-script
```

建立 `manage.py` 於根目錄

```python manage.py
from flask_script import Manager, Server
from main import app

# 設定你的 app
manager = Manager(app)
# 設定 python manage.py runserver 為啟動 server 指令
manager.add_command('runserver', Server())

# 設定 python manage.py shell 為啟動互動式指令 shell 的指令 
@manager.shell
def make_shell_context():
    return dict(app=app)

if __name__ == '__main__':
    manager.run()
```

當執行 `$ python manage.py runserver` 即會執行測試伺服器，`$ python manage.py shell` 可以在對話指令列中中輸入 `app` 找到被引入的 `<Flask 'main'>`，可以讓我們在互動式指令對話框中測試操作使用。

# 總結
本文介紹了如何建置 Python Web 開發環境，我們也實際完成了我們第一個 Python Flask 程式。在接下來章節中我們將持續介紹 Python Web Flask 實戰開發，並學習現代化網站開發的方方面面。

# 延伸閱讀
1. [Wiki Flask (web framework)](https://en.wikipedia.org/wiki/Flask_(web_framework))
2. [Python Web 程式設計入門實戰線上課程](http://pics.ee/c34g)

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 

（image via [fedoramagazine](https://cdn.fedoramagazine.org/wp-content/uploads/2015/11/Python_logo.png)）
