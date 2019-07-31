---
title: 用 Python 自學程式設計：程式設計思維入門
date: 2017-10-05 22:00:00
tags:
    - Python
    - Django
    - MVC
    - Web
author: kdchang
---

![Python 自學程式設計：程式設計思維入門](/img/kdchang/learning-programming/coding.jpg) 

# 什麼是程式設計思維？
最近幾年全球刮起了一股爭相學習電腦科學（Computer Science）和程式設計（Programming）的風氣，上至總統、首相下至升斗小民都爭相開始學寫程式。事實上，學寫程式最重要的是學習電腦科學背後思考方式和邏輯，學習如何透過科技解決生活上的問題。更進一步說，程式設計與其說是一種科學，更偏向一種藝術人文的創作與科學的混合體。以前畫家或是作家是拿紙筆創作，現在只要有一部電腦，連上網路，就可以透過敲打鍵盤將自己的創意付諸實踐。

有人說：`程式設計 = 演算法 + 資料結構`

這樣的說法不能說錯，事實上，具備良好資料結構和演算法素養的程式設計師往往可以寫出品質較好的程式碼，但大部分人看到演算法和資料結構就已經滿臉問號了，更別提曾經在學校被這兩門課程心靈受挫的學生們。所以對於初學者來說，更應該讓他了解，事實上學程式設計思維就是在學習解決問題的能力（並非每個人都需要成為程式設計師）：

1. 尋找並發現問題
2. 釐清並定義問題
3. 分解問題
4. 尋找解決方法和資源
5. 驗證問題和解決方式

唯有具備這樣解決問題的能力，才能稱得上真正學會程式設計思維。而這樣的思考方式的訓練是不管將來到哪裡，是否有從事程式設計相關工作都受用，這才是對於學習電腦科學與程式設計思維正確的認知。

# 寫程式就是下指令請電腦做事情
剛剛我們在上面提到程式設計思維中有一個部分是分解問題，事實上，寫程式在定義問題後，我們必須把問題切分，就像是食譜一樣，不管是再豐盛的料理或是滿漢全席，都有一個個步驟去完成。舉例來說，我們今天要創作一道蔥花蛋或菜脯蛋，我的步驟會是：

1. 放點油
2. 打蛋
3. 如果喜歡蔥花可以加入蔥花，如果喜歡菜脯可以加入菜脯（程式術語：`if...else` 條件判斷）
4. 放入少許鹽巴
5. 中火快炒，翻五次面（程式術語：`for` 迴圈）
6. 當看到蛋面呈現金黃色時可以起鍋，結束料理（程式術語：`while` 迴圈）
7. 好吃的蔥花蛋或菜脯蛋上桌

再大程式切分後就變成一個個小程式和指令，將程式切分成一個個模組，再將它們都組裝起來就成我們的應用程式和系統。

# 第一次學寫程式，學哪一種語言好呢？
既然程式設計思維很重要，究竟第一次學寫程式，學哪一種語言好呢？

先講結論：`Python` 語法平易近人，學習曲線平緩，應用廣，可以很快做出一些解決生活上問題的應用，累積成就感，適合初學者。當然若你是國中小的學生，`Scratch` 之類的拖拉式圖象化程式語言或許是合適選擇，但成人一般對於這種玩具比較排斥些，認為不是真正在學寫程式。

一般而言除了區分高低階程式語言外，我們會把程式語言分為：

`靜態語言`：在程式語言中我們會使用變數（variable）儲存程式的值，在靜態語言中需要事先宣告變數型態（type），也宣告了它會在記憶體中佔有多少空間等資訊。電腦會透過這些資訊把程式編譯（compile）成低階的機器語言讓電腦可以執行。這樣的設計可以讓電腦執行起來更有效率，但對於開發者來說會比較繁瑣一些（例如：宣告字串變數要在變數前加 string 宣告），也由於類型被宣告後無法改變，所以被稱為靜態語言。常見的靜態語言包括：C/C++、Java、C#、Go、Swift 等）。

`動態語言`（又稱為 script language）：相對於靜態語言，動態語言不會強制變數類型宣告，它不是使用編譯器而是使用直譯器（interpreter）來解譯。動態語言雖然開發和撰寫程式上效率較快，但執行速度往往比靜態語言慢（現在差距已經慢慢變小）。一般常見的動態語言包括：Python、JavaScript、PHP、Ruby 等）。


以下就來先簡單介紹常見程式語言和它的簡單語法範例（可以透過 [repl.it](https://repl.it) 可以將程式碼貼在網頁上呈現所見即所得效果）：

1. Python
本系列文章的主角，[Python](https://zh.wikipedia.org/wiki/Python) 是一種物件導向、直譯式的跨平台動態程式語言，它包含了一組功能完備的標準庫和豐富套件生態系，可以輕鬆完成很多常見的任務（例如：物聯網應用開發、遊戲、讀寫檔案、自然語言處理、網路爬蟲、網站開發、機器學習等），因為它可以很輕易整合其他底層語言，所以又稱為膠水語言。它的語法簡單，與其它大多數程式設計語言使用大括弧不一樣，它使用縮進來定義語句塊。由於具備簡潔易學等特性，是許多開發者推薦 Python 為初學者第一個學習的程式語言。由於版本更迭，我們接下來討論的主要是以 Python3 為主。以下是使用 Python 印出最喜歡的語言：

    ```python
    language = 'Python'
    print('My favorite Language is {}'.format(language))
    ```

2. C
    經典的傳統主力程式語言，適用於需要效能重視速度的應用，可以操作許多小細節，但學習門檻稍微高一些，執行前需要事先編譯完成：

    ```c
    #include <stdio.h>
    int main(int atgc, char *argv[]) {
        string language = "C++";
        printf("My favorite Language is %s", language);
        return 0;
    }
    ```

3. C++
    屬於 C 家族成員，具備物件導向特性，同樣是適用於需要效能重視速度和操作底層韌體、硬體的好選擇：
    ```c++
    #include <iostream>
    using namespace std;
    int main() {
        string language = "C++";
        cout << "My favorite Language is " << language;
        return(0);
    }
    ```

4. Java
    常見於企業系統和 Android 行動應用開發的 Java 是物件導向程式語言，由於跨平台開發等特性讓 Java 一直是市場蠻熱門的語言。
    ```java
    public class CodeLanguage {
        public static void main(String[] args) {
            string language = "Java";
            System.out.format("My favorite Language is %s", language);
        }
    }
    ```

5. C#
    由微軟推出吸收了 C++/Java 等優點的物件導向程式語言，常見於開發微軟平台相關的應用程式。
    ```csharp
    using System;
    namespace CodeLanguage {
        class CodeLanguage {
            static void Main() {
                string language = "C#";
                Console.WriteLine("My favorite Language is {}", language);
            }
        }
    }
    ```

6. JavaScript
    隨著版本演進和 Node.js 的推出後從玩具語言到可以挑大樑的程式語言，應用範圍遍及網頁前後端開發、遊戲開發、物聯網程式開發、手機程式開發等。是程式設計師社群平台 Github 和程式問答平台 StackOverflow 上最受歡迎的程式語言之一。且由於應用範圍廣泛，程式江湖更傳言：可以使用 JavaScript 編寫的程式，最終都會出現 JavaScript 版本。

    ```js
    var language = 'JavaScript'
    console.log('My My favorite Language is ' + language);
    ```

7. PHP
    吸收了 C/Java/Perl 特點的網站開發主力語言，世界上有許多的網站都是使用 PHP 進行開發。

    ```php
    <?php
        $language = 'PHP';
        echo 'My favorite Language is ' + language;
    ?>
    ```

8. Ruby
    常見於網頁 Web 開發，以 Ruby on Rails Web 開發框架聞名於程式設計社群。

    ```ruby
    language = 'Ruby'
    puts 'My favorite Language is #{language}'
    ```

9. Swift
    由 Apple 推出的推出的程式語言，主要用於開發 iOS/Mac 系列產品，應用範圍也涉及到了伺服器端開發（Server）。

    ```swift
    let language = 'Swift'
    print("My favorite Language is ", language)

    ```

10. Go
    由 Google 推出的程式語言，適合用於平行化程式開發。

    ```go
    package main
    import "fmt"

    func main() {
        language := 'Go'
        fmt.Printf("My favorite Language is %s", language)
    }
    ```

看完了眾多語言是不是發現其實 Python 語法還蠻可愛平易近人的呢？事實上，世界上沒有最好的程式語言，只有最適合的使用地方。接下來我們將透過 Python 學習程式設計思維。

# Python 環境建置
在開始之前我們必須先建置相關的開發環境，讓自己從麻瓜（不會寫程式的人）變成擁有程式魔力的魔法師。

所謂工欲善其事，必先利其器，要開發好的應用程式必須先準備好開發環境才行。以下介紹我們在接下來開發 Python Flask Web 應用程式所需要安裝的開發環境工具（以下以 Mac OS 等 Unix-like 作業系統為示範，若是 Windows 使用者建議安裝 Virtualbox 並運行 Linux Ubuntu 作業系統，[參考安裝文件](http://blog.xuite.net/yh96301/blog/432341564-VirtualBox+5.1%E5%AE%89%E8%A3%9DUbuntu+16.04)）：

1. Microsoft VSCode 編輯器
    Microsoft VSCode 是 Microsoft 推出的編輯器（非整合開發環境 IDE），夾帶著 Microsoft 過去打造 Visual studio 整合開發環境的豐富經驗，相比 Sublime Text 和其他編輯器，VSCode 的優勢在於開源且活躍的開發社群、內建 debugger 框架、原生 Git 整合、套件整合容易等特性。所以對於初學者來說 VSCode 是一個蠻適合入門的開發環境。它的安裝方式也十分簡易，在官網下載後按照指示安裝完成即可。

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/learning-programming/vscode-index.png) 

    我們可以安裝 Python 語法和格式檢查的相關 Python 套件幫助除錯：

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/learning-programming/vscode-python.png) 

2. 終端機環境（iTterm/內建 terminal）
    terminal（終端機）是我們下指令的地方，許多時候我們進行程式開發時不會使用 GUI 的介面而是使用下指令方式請電腦做相對應的行為。在 Linux 和 Mac 裡面都有內建的 terminal 的應用程式，若你是 Mac 用戶想使用更便利的工具（分割視窗、熱鍵、搜尋、自動補完等）可以額外安裝 [iterm2](https://www.iterm2.com/index.html) 做使用。若是不想使用 Virtualbox，使用 Windows 讀者可以使用 [Cmder](http://cmder.net/) 這個軟體當做終端機環境。

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/learning-programming/iterm2.png) 

    下指令（$ 為提示字元，不用輸入）：

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/learning-programming/iterm2-example.png) 

    對於有志於從事程式開發相關工作的讀者建議可以多熟悉指令碼的輸入，更多指令碼可以參考鳥哥撰寫的 [Linux 基本指令介紹](http://linux.vbird.org/linux_basic/redhat6.1/linux_06command.php) 和 [Linux 學習資源](http://www.linux.org.tw/resource.html)。

3. Git 版本控制系統/註冊 GitHub 帳戶 
    Git 是一種分散式版本控制系統，可以讓我們可以更方便地管理我們的程式碼。在網路上有非常多優秀的 Git 教學文件（[連猴子都能懂的Git入門指南](https://backlogtool.com/git-guide/tw/)、[寫給大家的 Git 教學](https://www.slideshare.net/littlebtc/git-5528339)、[初心者 Git 上手攻略](https://www.slideshare.net/lkiral/git-34157836)）。安裝 Git 方式是到官網下載軟體，依照指示安裝。

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/learning-programming/git-index.png) 

    互動式語法學習：

    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/learning-programming/try-git.png) 

    在介紹完 git 之後我們來了解一下 GitHub。GitHub 是一個可以存放 git 程式碼專案的平台，透過 GitHub 我們可以接觸到最新的開放原始碼資訊，也可以將我們的程式碼開源出來。
    
    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/learning-programming/github-index.png) 

    從 GitHub 上複製程式碼
    
    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/learning-programming/github-clone.png) 

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
    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/learning-programming/anaconda-index.png)

    選擇對應 Python 版本下載，我們使用 Graphical Installer（圖像介面安裝方式），接著在下載完成時按照預設的安裝方式完成安裝；
    ![Python Web Flask 實戰開發教學 - 簡介與環境建置](/img/kdchang/learning-programming/anaconda-install.png) 

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

## 建立虛擬環境
在建立相關開發工具後我們正式來建立一個 Python 的專案：

打開終端機移動到桌面，建立專案資料夾

```
$ cd ~/Desktop
$ mkdir python_examples
$ cd python_examples
```

建立獨立虛擬環境，並進入虛擬環境：

```
$ conda create -n python_examples_venv python
$ source activate python_examples_venv
```

成功進入虛擬環境後

# 你的第一個 Python 程式
一般而言我們會使用編輯器或是整合開發環境（IDE）進行程式撰寫，然後在終端機下指令執行程式。當然你也可以在終端機上使用內建互動式介面或是 jupyter notebook 進行。以下是我們要介紹給大家的簡單範例，第一個是隨機印出不同的喜愛程式語言，第二個是使用第三方套件擷取政府公開資料。程式設計唯有動手實際操作和實踐才能學的好，希望讀者打開你的編輯器透過自己實作去熟悉 Python 程式撰寫：

隨機印出不同的喜愛程式語言：

1. 引入 `random` 套件
2. 定義 `language` 變數，並將儲存程式語言字串的串列（list）資料結構給定給變數
3. 產生 1 到 2 之間的隨機整數
4. 列印出最喜歡的程式語言字串（含根據隨機產生的 index 選取到的串列值）

```python favorite_language.py
import random

language = ['Python', 'JavaScript', 'Java']
rnd = random.randint(1, 2)
print('My favorite Language is' + language[rnd])
```

可以在終端機移動到檔案資料夾執行程式檔案，例如：

```
python favorite_language.py
```

使用第三方套件擷取政府公開資料：

1. 引入 `requests` 套件
2. 爬取政府公開 Wifi 熱點資料，將取得資料回應給定給 `response` 變數
3. 將資料轉換成以 `{ "key": "value "}` 形式的 `json` 格式
4. 印出取得的資料

```python wifi_data.py
import requests

url = 'http://data.taipei/opendata/datalist/apiAccess?scope=resourceAquire&rid=b087af42-2c54-4dbf-86c8-427d788779e5'
response = requests.get(url)
data = response.json()
print(data)
```

# 總結
以上就是程式設計思維入門簡介，透過了解什麼是程式設計思維和不同語言的特性。當然，網路上也有[許多學習資源](http://happycoder.org/2017/01/27/learning-coding-programming-tutorial-and-resource/)可以當做參考。

（image via [mshcdn](https://i.amz.mshcdn.com/rRxXhoIhNucutinAio8YRF4TvzE=/1200x630/2017%2F06%2F15%2F71%2Fc1a206081efd44d1b61f5c0f86dcda6c.c222e.jpg)）

# 延伸閱讀
1. [Python 官網](https://www.python.org/)
2. [JavaScript 程式設計](https://pics.ee/1HC~)
3. [Python Web 程式設計](http://pics.ee/c34g)
4. [非本科生，我想半路出家學寫程式，該如何開始？](https://cofounderinc.com/2015/03/15/lerning-how-to-write-code/)
5. [自學程式設計學習資源懶人包](http://happycoder.org/2017/01/27/learning-coding-programming-tutorial-and-resource/)
6. [CS50](https://cs50.harvard.edu/)
7. [CS50 TV](http://cs50.tv/2016/fall/)

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 