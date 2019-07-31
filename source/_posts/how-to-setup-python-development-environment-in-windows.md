---
title: 如何在 Windows 打造 Python 開發環境設定基礎入門教學
date: 2018-04-14 20:23:23
author: kdchang
tags:
    - Python
    - Django
    - MVC
    - Web
    - MTV
    - Web Backend
    - Web Framework
    - 教學
    - Flask
    - 框架
    - coding
    - code
    - 程式設計
    - 自學程式設計
    - CS
    - Computer
    - Computer Science
---

# 前言
如何設定開發環境應該是每個初學程式設計者的痛（即便是老手有時也會覺得苦惱），尤其當你需要在終端機（terminal）輸入指令（command）來操控你的指令時（好吧，若你完全只走圖形化介面，習慣 GUI 操作就另當別論，但若你有志於往程式設計領域發展，建議還是熟悉一下指令碼）。

要在 [Linux](https:#zh.wikipedia.org/zh-tw/Linux)、[Mac OS](https:#zh.wikipedia.org/zh-tw/MacOS) 這種屬於[類 Unix 系統](https:#zh.wikipedia.org/wiki/%E7%B1%BBUnix%E7%B3%BB%E7%BB%9F)（指各種 Unix 的衍生系統，而 [Unix](https:#zh.wikipedia.org/wiki/UNIX) 指的是一種電腦作業系統，具有多工、多使用者的特色，是許多作業系統的父親）上打造 Python 開發環境相對容易，但當你使用 Windows 作業系統並希望在終端機下指令操作或開發應用程式時，往往受限於環境而產生許多困難和誤踩地雷。因此，接下來本文將教大家如何在 Windows 打造屬於自己的 Python 開發環境（包含一般 Winodows 安裝和使用虛擬機在 Windows 環境下建立 Linux/Ubuntu 作業系統，開發 Python 程式一般建議使用 Linux/Ubuntu 環境避免環境設定除錯困擾）！

# 開始建置 Python 開發環境
所謂工欲善其事，必先利其器，在開始之前我們必須先建置相關的開發環境，讓自己從麻瓜（不會寫程式的人）變成擁有程式魔力的魔法師。以下介紹我們在接下來開發 Python Web 應用程式所需要安裝的開發環境工具（強烈建議使用 Virtual Box 虛擬機搭配 Linux/Ubuntu 環境，若你真的很想使用 Windows 環境就繼續往下看吧！）。

1. Microsoft VSCode 編輯器
    [Microsoft VSCode](https:#code.visualstudio.com/) 是 Microsoft 推出的編輯器（非整合開發環境 IDE），夾帶著 Microsoft 過去打造 Visual studio 整合開發環境的豐富經驗，相比 Sublime Text 和其他編輯器，VSCode 的優勢在於開源且活躍的開發社群、內建 debugger 框架、原生 Git 整合、套件整合容易等特性。綜合以上幾點，對於初學者來說 VSCode 是一個蠻適合入門的開發環境。它的安裝方式也十分簡易，在官網下載後按照指示安裝完成即可，下載完成後可以打開看看。

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/vscode-index.png) 
 
    我們可以點選左邊 icon 欄的第五個（長得像正方形拼圖），安裝 Python 語法和格式檢查的相關套件幫助除錯（搜尋 Python），選擇 Python 並點選 install 安裝：

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/vscode-python.png) 

    你也可以於`檔案（file）-> 開啟（open）`打開你在電腦中已經準備好的專案資料夾，同時也可以在資料夾中新增檔案，我們之後的程式也會希望在建立好的專案資料夾中進行開發。這邊我們建立一個 hello.py 的檔案並印出 hello 訊息。

    ```py hello.py
    print('hello python')
    ```

2. 終端機環境（iTterm/內建 terminal）
    terminal（終端機）是我們下指令的地方，許多時候我們進行程式開發時不會使用 GUI 圖形化介面而是使用下指令方式請電腦做相對應的行為（記得寫程式就是下指令請電腦做事情！）。在 Linux 和 Mac 裡面都有內建的 terminal 的應用程式，以下為 MacOS 的 iTerm2 終端機下指令示意圖（iTerm2 中 $ 為提示字元，不用輸入）：

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/iterm2-example.png) 

    使用 Windows 讀者可以使用 [Cmder](http:#cmder.net/) 這個軟體當做終端機環境。

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/cmder1.png) 

    比起 Winodws 內建的命令列 CMD，cmder 更貼近 Unix 的命令列指令碼：

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/cmder2.jpg) 
    
    首先到 [Cmder 官網](http:#cmder.net/)先安裝 Cmder Full 版本（含 git），安裝完成後解壓縮資料夾到桌面，執行裡面的 `cmder.exe` 檔案即可。

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/cmder4.png) 

    Cmder 預設是 `λ`，如果不習慣可以改成 Mac / Linux 環境下的 `$`，具體流程請[參考這份文件](https:#jeffjade.com/2016/01/13/2016-01-13-windows-software-cmder/)。

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/cmder3.png) 

    對於有志於從事程式開發相關工作的讀者建議可以多熟悉指令碼的輸入，更多指令碼可以參考鳥哥撰寫的 [Linux 基本指令介紹](http:#linux.vbird.org/linux_basic/redhat6.1/linux_06command.php) 和 [Linux 學習資源](http:#www.linux.org.tw/resource.html)。

    ```shell
    以下是常用指令
    # 移動到桌面
    cd \Users\XXXX\Desktop
    # 列出資料夾下檔案
    ls
    # 刪除檔案
    rm 檔名
    # 複製檔案
    cp 檔名
    ```


3. Git 版本控制系統/註冊 GitHub 帳戶 
    Git 是一種分散式版本控制系統，可以讓我們可以更方便地管理我們的程式碼。在網路上有非常多優秀的 Git 教學文件（[連猴子都能懂的Git入門指南](https:#backlogtool.com/git-guide/tw/)、[寫給大家的 Git 教學](https:#www.slideshare.net/littlebtc/git-5528339)、[初心者 Git 上手攻略](https:#www.slideshare.net/lkiral/git-34157836)）。安裝 Git 方式是到官網下載軟體，依照指示安裝（若您使用 Cmder 的完整安裝 Download Full 的版本就不用安裝 git，因為已經幫你安裝好了）。

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/git-index.png) 

    互動式語法學習：

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/try-git.png) 

    在介紹完 git 之後我們來了解一下 GitHub。GitHub 是一個可以存放 git 程式碼專案的平台，透過 GitHub 我們可以接觸到最新的開放原始碼資訊，也可以將我們的程式碼開源出來。
    
    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/github-index.png) 

    從 GitHub 上複製程式碼
    
    <!--![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/github-clone.png) -->

    ```
    # 複製一份到本地端
    $ git clone https:#github.com/kdchang/python101.git
    # 移動到資料夾
    $ cd python101
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

        - 便於安裝許多流行的科學、數學、工程、資料分析的 Python 模組  
        - 免費並支援跨平台：Linux、Windows、Mac
        - 內建 Spyder 編輯器和 Jupyter Notebook 環境 
        - 方便建立不同的虛擬開發環境

    安裝流程也十分簡單，進入 Anaconda 首頁，選擇對應作業系統（這邊使用 Windows）和是屬於 64 還是 32 位元：
    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/anaconda-index.png)

    Windows10 可以在`系統`看到位元資訊；
    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/about-64bit-windows-10.jpg) 

    Windows7 可以在`控制台->系統與安全->系統`觀看作業系統位元資訊：
    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/windows-64-bit.png) 

    選擇對應 Python 版本下載（這裡選擇 Python3 以上版本），我們使用 Graphical Installer（圖像介面安裝方式），接著在下載完成時按照預設的安裝方式完成安裝；
    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/anaconda-install.png) 

    記得安裝時要注意建議在安裝 anaconda 時勾選把環境變數加入（path environment variable），這樣在使用 cmder 時使用 conda 相關指令才不會出現錯誤，若一開始沒有勾選的話建議解除安裝後再重新安裝 anaconda 勾選加入環境變數。

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/windows-anaconda-path.png) 

    若是完整安裝成功，可以打開 Cmder 終端機輸入，若是顯示 Python 3.6.0 :: Anaconda 4.3.0 (x86_64) 版本號即安裝成功（若沒成功可能要檢查一下是不是環境變數路徑的問題）：

    ```
    $ python -V
    Python 3.6.0 :: Anaconda 4.3.0 (x86_64)
    ```

    接著要建立我們專案虛擬環境，這樣在安裝操作套件時比較不容易被污染到 root 全域的環境（因為你可能會有很多專案，專案使用的套件不盡相同，正式上線時只要把相關套件資訊透過 `pip freeze > requirements.txt` 存起來，然後在正式上線的伺服器安裝 `pip install -r requirements.txt` 即可），啟動後會出現（套件名稱）的提示字元：

    ```
    顯示目前虛擬環境列表
    conda info -e 
    創建虛擬環境
    conda create -n 套件名稱 python=3.6
    進入虛擬環境（若是非 Windows cmder 環境加 source 於開頭） ，成功後提示字元變成：（套件名稱）$
    activate 虛擬環境名稱
    離開虛擬環境（若是非 Windows cmder 環境加 source 於開頭） 
    deactivate 
    ```

## 建立虛擬環境（virtual environment）
接著我們正式來建立一個 Python 的專案，打開終端機移動到桌面，建立專案資料夾（在輸入指令時可以使用 tab 自動補完指令）

```
# 移動到桌面
cd \Users\XXXX\Desktop
# 建立資料夾
mkdir python_examples
# 移動到資料夾
cd python_examples
```

建立獨立虛擬環境，並進入虛擬環境：

```
# 創立虛擬環境
conda create -n python_examples_venv python
# 啟動虛擬環境
activate python_examples_venv
```

成功進入虛擬環境後（會出現小括號 python_examples_venv）代表已經進入虛擬環境，即可以在裡面執行 Python 程式並安裝相關套件於該虛擬環境下：

```
# 安裝 django web 框架套件
pip install django
# 執行 python 檔案
python hello.py
```

事實上，在 Python3 中還有另外兩種建立虛擬開發環境的方式，包括使用 Python 內建工具：

```shell
# 使用 Python3 內建工具建立名為 example_venv 的虛擬開發環境
python -m venv example_venv
```

使用 `virtualenv`，和 anaconda 不同的是 virtualenv 會在建立虛擬環境的專案資料夾下建立一個資料夾裡面放虛擬環境的東西：

```shell
# 先安裝 virtualenv
pip install virtualenv
# 使用 virtualenv 產生一個名為 example_venv 的
virtualenv example_venv
# 移動到 example_venv 的外面資料夾，執行進入虛擬環境
example_venv\Scripts\activate
# 安裝 django web 框架套件到虛擬環境中（只會安裝在該虛擬環境中）
pip install django
```

# 整合在一起：在 Windows 撰寫你的第一個 Python 程式
確認安裝好以下工具後，我們就可以開始撰寫你的第一個 Python 程式
1. 安裝 Microsoft VSCode
2. 安裝 Cmder
3. 安裝 Anaconda（記得勾選加入環境變數）
4. 安裝 virtualenv (在終端機使用：`pip install virtualenv` 安裝)
5. 在桌面創建一個 python_example 資料夾，打開 Microsoft VSCode 後開啟該專案資料夾，創建一個 `hello.py` 的檔案並在裡面打上 `print('hello python!!')`
6. 打開 cmder 終端機 cd 移動到 `hello.py` 所在資料夾
7. 執行 `python hello.py`，恭喜你完成第一個 Python 程式！

# 在 Windows 上安裝 Linux/Ubuntu
1. [安裝 VirtualBox](https:#www.virtualbox.org/wiki/Downloads) 對應版本虛擬機（這邊安裝 Windows 版本，若你是 Mac 想嘗試 Linux 也可以安裝 Mac 版本），下載完按照步驟安裝完成

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/virtualbox1.png) 

2. 到[官網下載 Linux/Ubuntu 光碟映像檔案](https:#www.ubuntu-tw.org/modules/tinyd0/)，請根據電腦位元架構選擇最新桌面穩定版本 16.04 LTS（for windows）

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/virtualbox2.png) 

3. 建立 Linux Ubuntu 虛擬機，可以參考[這篇 VirtualBox 虛擬機器安裝 Ubuntu 設定教學](https:#www.pcsetting.com/linux/29)，設定一下名稱、作業系統類型和版本，欲分配給虛擬的記憶體大小（建議調整為大約實體記憶體的 1/3，舉例來說你有 30G 記憶體，可以分配 10G 給虛擬機）。接著選擇立即建立虛擬硬碟、VDI (VirtualBox 磁碟映像)、動態配置硬碟大小不會造成浪費（虛擬硬碟容量建議 30G 以上）。

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/virtualbox3.jpg) 

4. 選擇建立好的虛擬機並選設定值按鈕，選擇存放位置選項，控制器中選擇虛擬 CD/DVD 檔案選擇剛剛從官網下載下來的檔案，確定後接著選擇虛擬機並啟動，接下來選擇安裝 Linux/Ubuntu，選擇立即安裝和預設值，需要一段時間安裝和設定（中間會有語言相關的選擇和密碼設定）

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/virtualbox4.jpg) 

5. 啟動使用 Linux/Ubuntu，按照上面教學安裝 Microsoft VSCode 編輯器，安裝 Anaconda，建立第一個 Python 檔案、打開終端機（termical）安裝 ，執行 Python 程式

    ![title: 如何在 Windows 打造 Python 開發環境設定](/img/kdchang/python101/virtualbox5.png) 

網路上也有許多相關參考教學文章：[VirtualBox 5.2安裝Ubuntu 16.04 相關教學](http:#blog.xuite.net/yh96301/blog/list-view/432341564)、[VirtualBox 虛擬機器安裝 Ubuntu 設定教學](https:#www.pcsetting.com/linux/29)

若是螢幕太小可以在上排選單中選擇：`裝置->插入 Guest Additions CD 映像...->執行->重開虛擬機->可以調整大小`。

# 整合在一起：用 Linux 撰寫你的第一個 Python 程式
確認安裝好以下工具後，我們就可以開始撰寫你的第一個 Python 程式
1. 進入虛擬機的 Linux/Ubuntu 
2. 安裝 Microsoft VSCode
3. 安裝 Anaconda（記得勾選加入環境變數）
4. 安裝 virtualenv (在終端機使用：`pip install virtualenv` 安裝)
5. 在桌面創建一個 python_example 資料夾，打開 Microsoft VSCode 後開啟該專案資料夾，創建一個 `hello.py` 的檔案並在裡面打上 `print('hello python!!')`
6. 打開 terminal 終端機 cd 移動到 `hello.py` 所在資料夾
7. 執行 `python hello.py`，恭喜你完成第一個 Python 程式！

# 總結
如何設定開發環境應該是每個初學程式設計者的痛（強烈建議使用 Virtual Box 虛擬機搭配 Linux/Ubuntu 環境），以上介紹了如何在 Windows 打造 Python 開發環境設定，請讀者務必照著自己的電腦作業系統環境安裝一次，當然若你有志於往程式設計領域發展，也要熟悉一下指令碼操作。

# 參考文件
1. [VirtualBox 5.2安裝Ubuntu 16.04](http:#blog.xuite.net/yh96301/blog/432341564-VirtualBox+5.1%E5%AE%89%E8%A3%9DUbuntu+16.04)
2. [Create virtual environments for python with conda](https:#uoa-eresearch.github.io/eresearch-cookbook/recipe/2014/11/20/conda/)
3. [conda vs. pip vs. virtualenv](http:#stuarteberg.github.io/conda-docs/_downloads/conda-pip-virtualenv-translator.html)
4. [Anacodna之conda VS Virtualenv VS Python 3 venv 对比使用教程，创建虚拟环境](https:#segmentfault.com/a/1190000005828284)
5. [命令列指令碼查詢](https:#ss64.com/)

（image via [githubusercontent](https:#camo.githubusercontent.com/812b2647d6cd216ecddfb3f0ec71639473717955/687474703a2f2f692e696d6775722e636f6d2f67316e4e6630492e706e67)、[websiteoptimization](https:#i.ytimg.com/vi/Xm790AkFeK4/maxresdefault.jpg)、[ytimg](https://i.ytimg.com/vi/GGorVpzZQwA/maxresdefault.jpg)、[ytimg](https://i.ytimg.com/vi/DPIPC25xzUM/maxresdefault.jpg)、[ostechnix](https://www.ostechnix.com/wp-content/uploads/2017/03/VirtualBox_Ubuntu-16.04-LTS-Desktop_20_03_2017_19_44_58.png)）

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 