---
title: Git 與 Github 版本控制基本指令與操作入門教學
date: 2018-01-17 10:23:23
author: kdchang
tags: 
    - 軟體工程師
    - 軟體工程
    - software engineering
    - bash
    - shell
    - svn
    - version control
    - VCS
    - github
    - git
    - svn
    - cvs
    - 自學程式心得   
---

![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/git.png)

# 前言
要成為一個真正的軟體工程師（Software Engineer）除了掌握基礎開發能力外，更重要的是和其他工程師和開發者團隊合作和溝通的能力，所以若你能培養出掌握 Git 等版本控制操作和 Git server 架設的能力，你會更容易參與開放原始碼（open source）的社群和提昇自己在職場上的價值。好，那我們就準備開始吧！

# 什麼是版本控制系統（Version Control System）？

![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/files.png)

版本控制系統是一種軟體工程的開發技巧，可以透過這個系統讓每位成員的軟體版本可以方便同步和維護管理（不然要用 email 或是其他工具傳送和管理十分麻煩，尤其程式又常常會有不同版本修改的問題！）。在沒有版本控制系統時，我們常會在編輯檔案前複製一個備份，或是在更新檔案後產生許多重複檔案，非常不便且難以維護。因此，使用版本控制系統的需求就應運而生啦！

![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/hero-mac-screenshot.png)

一般在軟體開發中又分為中央式系統（例如：[Subversion](https://zh.wikipedia.org/wiki/Subversion)、[CVS](https://zh.wikipedia.org/wiki/%E5%8D%94%E4%BD%9C%E7%89%88%E6%9C%AC%E7%B3%BB%E7%B5%B1) 等）與分散式系統（例如：[Git](https://git-scm.com/)、[BitKeeper](BitKeeper)、[mercurial](https://zh.wikipedia.org/zh-tw/Mercurial) 等），中央式版本控制系統的工作主要在一個伺服器進行，由中央管理存取權限「鎖上」檔案庫中的檔案，一次只能讓一個開發者進行工作。而分散式系統讓不同開發者直接在各自的本地檔案庫工作，並容許多個開發者同時更動同一個檔案，而每個檔案庫有另外一個合併各個改變的功能。分散式系統讓開發者能在沒有網路的情況下也能繼續工作，也讓開發者有充分的版本控制能力，而不需經中央管理者的許可，但分散式系統仍然可以有檔案上鎖功能。

![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github.png)

# 什麼是 Git？什麼是 Github？
[Git](https://zh.wikipedia.org/wiki/Git) 是一個分散式版本控制軟體，最初由 Linus Torvalds 創作（也是作業系統 [Linux](https://zh.wikipedia.org/zh-tw/Linux) 系統的開發者），其最初目的是為更好地管理 Linux kernel 開發而設計，其具備優秀的 merge tracing 合併程式碼的能力（使用程式碼 snapshot 來比較歷史版本差異）。

[Github](https://github.com) 則是一個支援 git 程式碼存取和遠端托管的平台服務，有許多的開放原始碼的專案都是使用 Github 進行程式碼的管理。若是讀者未來有志於從事程式設計相關工作的話，建議可以熟悉掌握 Git 和 Github 的使用，並建立自己的 Github profile 作品集。

# Git 基本觀念

![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/git-workflow.png)

Git 可以分為 Local（本地）和 Remote（遠端）兩個環境，由於 Git 屬於分散式的版本控制系統，所以開發者可以在離線 local 環境下開發，等到有網路時再將自己的程式推到 Remote 環境或 pull 下其他開發者程式碼進行整合。在 Local 中我們又分為 working directory（工作資料夾）、staging area（暫存區）和 repositories（檔案庫）。

當自己開發時會在工作資料夾工作，當要進入檔案庫之前會先將檔案加入暫存區，確認沒問題則 commit 到檔案庫中，最後 push 上去 remote 環境。在 Git 中若是有和其他開發者一起合作，則會需要處理不同 branch 之間 conflict 和 merge 的問題。

![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/git-branch.png)

# Git 與 Github 實戰操作入門教學
接下來我們用一個實際的實戰範例讓讀者可以快速的掌握那些基礎必知必會的 Git/Github 操作技能：

1. 安裝並且設定 Git

    `任務 1：安裝 Git 到你的電腦上，並且設定好 Git 內部的使用者名稱和電子信箱`

    Linux 若是使用 `Debian` 基礎的作業系統可以在終端機輸入以下指令安裝 git：

    ```
    $ apt-get install git
    ```

    若是 `Fedora` 系列可以輸入：

    ```
    $ yum install git-core
    ```

    若是 windows 可以下載安裝 [cmder](http://cmder.net/) 這個模擬 Linux terminal 終端機時選擇完整版本就會順便安裝或是到 [Git 官網安裝](https://git-scm.com/)。當然在市面上有許多免費 [GUI 圖形化的 Git 操作軟體](https://git-scm.com/download/gui/linux)，若是初學者則建議先熟悉整個 git 工作模式和指令，再去使用圖形化工具會比較好，這樣你才比較知道圖形化程式背後做了什麼事情。Mac 則是可以到 [Git 官方網站](https://git-scm.com/downloads) 選擇對應作業系統，按照步驟完整下載安裝。另外也可以參考[中文安裝教學](https://git-scm.com/book/zh-tw/v1/%E9%96%8B%E5%A7%8B-%E5%AE%89%E8%A3%9D-Git)。

    由於我們會選擇 Github 當作遠端托管程式的環境，所以我們也可以根據作業系統安裝 [Github 桌面版](https://desktop.github.com/)當作操作工具（內建安裝 Git）。

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/git-version.png)

    若是安裝完成，打開終端機（terminal）或 cmder 輸入以下指令，若成功顯示版本（建議 Git 版本在 2 以上），代表安裝成功囉！terminal（終端機）是一個可以下指令讓電腦做事情的互動介面（例如：新增檔案、移動到資料夾等）

    ```
    // 其中 $ 為提示字元，在輸入指令時不用輸入該符號，否則會錯誤，若是使用 windows cmder 預設是 λ
    $ git --version
    ```

    接下來設定你的帳戶，讓 Git 知道這台電腦做的修改要連結到哪一個使用者（待會我們要在 Github 上註冊帳號，建議使用一致的帳號和電子信箱）：

    ```
    $ git config --global user.name "<Your Name>"
    ```

    設定電子郵件：

    ```
    $ git config --global user.email "<your@gmail.com>"
    ```

2. 建立一個本機的 repository

    `任務二：在自己的電腦上建立一個新的 local repositories（本地檔案庫）`

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/git-folder.png)

    repository（檔案庫）是一個什麼樣的概念呢？事實上 repository 就是一個專案，又簡稱 repo。以電腦的檔案資料管理來看，我們通常會把同一個專案的資料放到同一個資料夾下，所以我們也可以把 repository 看成一個資料夾。

    ```
    // 建立一個 hello-git 資料夾
    $ mkdir hello-git
    // 移動到 hello-git 資料夾
    $ cd hello-git
    // 將專案資料夾建立成 git repository
    $ git init
    // 列出專案資料夾下的檔案和資料夾（-l 參數為列出詳細資料，-a 為列出隱藏資料夾）
    $ ls -la 
    ```

    當你執行 `git init` 後，你可以發現多出了 `.git` 這個隱藏資料夾，可以看到裡面檔案和資料夾如下：

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/git-folder2.png)

    到這邊恭喜讀者，完成你第一個 git 專案資料夾改造計畫！

3. 檢視狀態、新增或修改 commits

    `任務三：在你的 repository 檔案庫中建立一個新檔案，新增一些內容到該檔案並且將那些檔案修改提交 commit 到 Git 中`

    接著我們使用文字編輯器，新增一個 hello.py 的檔案（裡面是一個會印出 `hello python & git` 字串的 python 程式）：

    ```python hello.py
    print('hello python & git')
    ```

    然後在終端機的專案資料夾下輸入 `git status` 顯示目前工作環境狀態：

    ```
    $ git status
    On branch master

    Initial commit

    Untracked files:
    (use "git add <file>..." to include in what will be committed)

        hello.py

    nothing added to commit but untracked files present (use "git add" to track)
    ```

    我們會發現因為我們有新增新的檔案，但是還沒進到 git 追蹤範圍中/暫存區，所以我們要使用 `git add hello.py` 加入追蹤，這樣之後檔案有修改就可以追蹤到。

    ```
    $ git add hello.py
    $ git status
    On branch master

    Initial commit

    Changes to be committed:
    (use "git rm --cached <file>..." to unstage)

        new file:   hello.py
    ```

    若是確認沒問題我們就準備 commit 進去 repository 囉！

    ```
    // 比較現在檔案和上次 commit 之間的差異，也就是說你做了哪些修改
    $ git diff
    // -m 為輸入 commit message，也就是說這個 commit 內做了哪些事情
    $ git commit -m "Init hello.py"
    [master (root-commit) ad6d328] Init hello.py
    1 file changed, 1 insertion(+)
    create mode 100644 hello.py

    // commmit 完成
    $ git status
    On branch master
    nothing to commit, working tree clean
    ```

    若是想反悔不想把檔案加入追蹤呢？

    ```
    // 檔案尚未加入過追蹤時使用，即可恢復到檔案尚未加入暫存區
    $ git rm --cached hello.py

    // 若檔案已經在 repository 內，則使用以下指令
    // repository 與 stage 的檔案都會被還原到 HEAD，但 working directory 內的檔案不變
    $ git reset HARD
    ```

    當你追蹤後修改了檔案，例如把 `hello.py` 的內容改成：

    ```python hello.py
    print('hello python & git rock')
    ```

    若有檔案修改，記得要再 `add` 修改的檔案（這是新手比較容易忘記的部分），若是要放棄修改可以使用 `git checkout -- 檔案名稱`

    ```
    // 比較現在檔案和上次 commit 之間的差異，也就是說你做了哪些修改
    $ git diff
    // 查看目前工作狀態
    $ git status
    Changes not staged for commit:
    (use "git add <file>..." to update what will be committed)
    (use "git checkout -- <file>..." to discard changes in working directory)

        modified:   hello.py
    ```

    commit 這個修改時簡寫會寫成這樣（-a 是 add，-m 為 message 簡寫，後面接訊息資訊）：

    ```
    $ git commit -a -m "修改了 hello.py"
    ```

4. 註冊 GitHub 帳號

    `任務四：建立一個 GitHub 帳號，並在 Git 設定中加入使用者帳號`

    到目前為主我們程式主要是在我們 local 工作環境中操作，但還記得我們在 Git 基本觀念中有提到，git 分為 local 和 remote，若是我們想要和全世界其他開發者合作或是貢獻開放原始碼的話，我們可以透過 github 來當作我們 remote 工作環境，去管理我們程式碼，同樣的也可以透過 github 平台和其他開發者一起合作，參與開放原始碼的開發。github 若是公開的 repo 是無限制數量免費的，但是若是想使用 private repo 可以[參考付費方案](https://github.com/pricing)。

    ```
    // 注意大小寫要一致
    $ git config --global user.username <你的 github 使用者名稱>
    ```

    先到 github.com 註冊帳號：

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example1.png)

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example4.png)

    點選右上角 `+` 來新增 new repository（檔案庫）：

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example3.png)

    輸入 repository name（你要取的專案名稱），最好跟電腦 local 專案一致，我們這邊輸入 `git-example`。然後輸入簡短專案描述並先不勾選初始化 README 和，也先不要選擇 .gitignore 和 License 授權（不然會造成本地端和遠端不一致會需要額外一些處理）：

    `.gitignore`：要忽略的檔案清單，這是用來告訴 Git，當在做版本控制記錄的時候，忽略這些檔案。通常一些機密資料，如資料庫帳號密碼或是 server IP 位置等，記得要加入。也可以參考 github 上面的[一些範本](https://github.com/github/gitignore) 在新增 repository 時選取對應的程式語言

    `README.md`：repository 介紹和使用方式說明（例如：使用方法、參與專案方式等），使用 `markdown` 語法撰寫。另外通常有 CONTRIBUTING.md 額外說明如何參與貢獻。

    `LICENSE`：專案使用何種授權方式，例如：MIT、BSD 等

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example5.png)

    接著按 create 按鈕！恭喜你新增成功，創建了自己第一個 github repository（遠端檔案庫）！

5. 將 repository 做本機和遠端的連結

    `任務五：把電腦裡 Local（本地端）的 repository（檔案庫）和 remote（遠端）的 repository（檔案庫）連結起來，並 push 電腦上的修改`

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example6.png)

    因我們已經在專案有把 `hello.py` 修改加入追蹤並 commit 到 local 檔案庫，所以我們可以參考下方的指令說明把 remote 網址加入：

    ```
    // 本地端專案知道 origin 對應到遠端網址
    $ git remote add origin <remote 網址>
    ```

    接者準備將本地端程式 push 到遠端檔案庫：

    ```
    // 觀看情況
    $ git status
    // 將本地端程式 push 到遠端檔案庫
    $ git push -u origin master
    Counting objects: 3, done.
    Writing objects: 100% (3/3), 239 bytes | 0 bytes/s, done.
    Total 3 (delta 0), reused 0 (delta 0)
    To https://github.com/happycodergit/git-example.git
    * [new branch]      master -> master
    Branch master set up to track remote branch master from origin.
    ```

    參數 `-u` 等同於 `--set-upstream`，設定 `upstream` 可以使分支開始追蹤指定的遠端分支。事實上，只要做過一次 `$ git push -u <remote name> <branch name>`，並且成功 push 出去；本機端的 master 就會被設定去追蹤遠端的 `<remote name>/<branch name>` 分支。只要成功設定好 upstream 後，第二次以後要上傳分支時，就只需要透過 git push 就可以了，不必再帶 `<remote name>` 跟 `<branch name>` 等參數。例如：`$ git push`。

    事實上，`$ git push -u origin master` 可以拆解成：

    ```
    $ git push origin master
    $ git checkout master
    $ git branch -u origin/master
    ```

    恭喜你成功把你的專案推送上去 github 啦！

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example7.png)

6. Fork 和 clone 一個 open source（開源）的計畫

    `任務六：從 GitHub.com 建立專案，複本 fork，並下載 clone 到電腦上`

    點選左上角 github icon 回到首頁，我們從上面搜尋欄搜尋 react：

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example8.png)

    點選右上角 fork 按鈕，複製一份專案到我們這：

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example9.png)

    等待 fork 中：

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example10.png)

    點選右邊綠色按鈕 clone download 複製 HTTP 網址：

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example11.png)

    ```
    // 複製到本地端
    $ git clone https://github.com/happycodergit/react.git
    // 移動到 react 資料夾
    $ cd react
    // 切出自己的新分支（使用 -b）
    $ git checkout -b happycoder@feature_branch
    // 做一些 README.md 檔案修改，然後 commit 到自己 fork 過來的專案
    $ git commit -a -m "Update README"
    $ git push origin happycoder@feature_branch
    ```

    事實上，每個開放原始碼都有他自己貢獻的方式，記得要先了解。例如可以[參考 react 貢獻說明](https://reactjs.org/docs/how-to-contribute.html)！

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example12.png)

    你可以透過將自己的修改 commit 到自己 fork 過來的專案，然後到原始專案頁面點選 new pull request 按鈕發 pull request（會比對程式碼的差異）。若對方 review 完後接受就可以將自己的程式碼合併到原始專案中，為開放原始碼做出第一步貢獻！

    ```
    // 若完成 pull request 記得讓 master（或是合併進去的 branch）保持同步
    $ git pull upstream master
    ```

7. 練習建立一個 feature branch

    `任務七：回來原來的 git-example 專案新增 feature branch（分支）`

    ```
    // 建立一個名為 dev 的 branch
    $ git checkout -b dev
    ```

    在 `hello.py` 最上面多加一行 `# hi, this is comment` 註解後存檔

    ```
    // commit 到本地端更新
    $ git commit -a -m "Init dev branch"
    $ gut push origin dev
    ```

8. 邀請別人和你合作

    `任務八：在專案新增夥伴 collaborator`

    在右上角進入 setting 可以選擇到 collaborator 新增合作者：

    ![自學程式設計與電腦科學入門實戰：Git 與 Github 版本控制基本指令與操作入門教學](/img/kdchang/cs101/github-example13.png)

9. 利用 push 和 pull 來和 GitHub.com 同步

    `任務九：用 pull 來和其他 collaborators（合作者）同步更新，確保程式是最新的版本`

    請其他開發者 git colone 下來你的 git-example 專案並 checkout 到 `dev` branch，並完成新增一個 `README.md` 檔案後發 pull request 過來，若沒問題就按同意並合併。我們則透過 git pull 來保持本地端和遠端程式碼同步：

    ```
    $ git pull origin dev
    ```

10. Merge（合併）和刪除 branches

    `任務十：在本機上 merge 合併你的 branch（分支），刪除舊的 branch（分支）`

    ```
    // 移動到 master branch
    $ git checout master
    // 合併 dev 到 master
    $ git merge dev
    // 刪除 dev branch
    $ git branch -d dev
    // 將合併後的 master 推送到遠端
    $ git push origin master
    ```

# 總結
以上就是 Git 與 Github 版本控制基本指令與操作入門教學！希望讀者可以動手一起操作，漸漸就能感受到 git 的威力和好處，同時也有能力參與開放原始碼（open source）的社群和提昇自己在職場上的價值，朝成為一個真正的軟體工程師（Software Engineer）邁進！

# 參考文件
1. [30 天精通 Git 版本控管](https://github.com/doggy8088/Learn-Git-in-30-days/blob/master/zh-tw/README.md)
2. [Git 官方網站](https://git-scm.com/)
3. [Try Git](https://www.codeschool.com/courses/try-git)
4. [Git-it](http://jlord.us/git-it/index-zhtw.html)
5. [GitHub Guides](https://guides.github.com)
6. [如何將檔案從stage移除?](http://oomusou.io/git/git-remove-stage/)
7. [Git 教學](https://www.gitbook.com/book/zlargon/git-tutorial/details)

（image via [git-scm](https://git-scm.com/images/logos/downloads/Git-Logo-1788C.png)、[qbox](https://dn-sdkcnssl.qbox.me/article/fyuBUISCkmddVNC0t2Iu.png)、[quoracdn](https://qph.ec.quoracdn.net/main-qimg-3aa29f29ede6a8245b6964f663c60339)、[ytimg](https://i.ytimg.com/vi/DPIPC25xzUM/maxresdefault.jpg)、[linux](https://3c1703fe8d.site.internapcdn.net/newman/gfx/news/hires/2014/linux.jpg)、[imgur](https://i.stack.imgur.com/RUIIq.png)、[martinfitzpatrick](https://martinfitzpatrick.name/images/method/1472/GitBranchName_1.png)）

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 