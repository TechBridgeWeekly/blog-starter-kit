---
title: Linux Command 命令列指令與基本操作入門教學
date: 2017-12-23 10:23:23
author: kdchang
tags: 
    - linux
    - 指令
    - 命令列
    - command line
    - 軟體工程師
    - 軟體工程
    - software engineering
    - bash
    - shell
    - script
    - unix
    - mac
    - os
    - windows
---

![Linux Command 命令列指令與基本操作入門教學](/img/kdchang/cs101/terminal.jpg)

# 前言
要成為一個真正的軟體工程師（Software Engineer）不一定一定要使用 vim 之類的編輯器進行開發，但熟悉使用終端機（terminal）操作和常用的 Linux 命令列指令碼操作則是必須的。因此接著我們將介紹軟體工程師在開發上常用的 Linux 命令列指令碼給讀者參考。若讀者使用的是 Windows 建議安裝 VirtualBox 並在上面跑 Linux/Ubuntu 來進行指令碼操作，或是使用像是 Cmder 的工具來進行操作。關於相關工具安裝可以參考 [如何在 Windows 打造 Python 開發環境設定基礎入門教學](http://happycoder.org/2017/11/17/how-to-setup-python-development-environment-in-windows/) 這篇教學文章。好，那我們就準備開始吧！

![Linux Command 命令列指令與基本操作入門教學](/img/kdchang/cs101/linux.jpg)

# 什麼是 Linux？
[Linux](https://zh.wikipedia.org/wiki/Linux) 是一種自由開放原始碼的[類 Unix](https://zh.wikipedia.org/wiki/%E7%B1%BBUnix%E7%B3%BB%E7%BB%9F) 的作業系統，其廣泛運用於伺服器和嵌入式系統中。目前主流的 Linux 發佈版本（Linux distributions，係指可完整安裝使用的套件版本）包括：Debian、Fedora、CentOS、Ubuntu 等。以下我們主要將會聚焦在 Linux/Ubuntu 常用指令和基礎操作的入門教學介紹上（每個指令前使用 `$` 當作提示字元，不用輸入）。

![Linux Command 命令列指令與基本操作入門教學](/img/kdchang/cs101/ubuntu.png)

# Linux 檔案系統架構
理論上所有的 Linux 發佈版本應該都要遵守檔案系統的標準（Filesystem Hierarchy Standard, FHS），但根據發佈版本不同或有差異，不過大致上檔案系統架構如下：

1. /bin, /sbin  
`/bin` 主要放置一般使用者可以操作的指令，`/sbin` 放置系統管理員可以操作的指令。連結到 `/usr/bin`，`/usr/sbin` 

2. /boot  
主要放置開機相關檔案  

3. /dev  
放置 device 裝置檔案，包話滑鼠鍵盤等

4. /etc  
主要放置系統檔案

5. /home, /root
`/home` 主要是一般帳戶的家目錄，`/root` 為系統管理者的家目錄

6. /lib, /lib64 
主要為系統函式庫和核心函式庫，若是 64 位元則放在 `/lib64`。連結到 `/usr/lib`, `/usr/lib64`

7. /proc
將記憶體內的資料做成檔案類型

8. /sys
與 `/proc` 類似，但主要針對硬體相關參數

9. /usr
`/usr` 全名為 `unix software resource` 縮寫，放置系統相關軟體、服務（注意不是 user 的縮寫喔！）

10. /var 
全名為 `variable`，放置一些變數或記錄檔

11. /tmp 
全名為 `temporary`，放置暫存檔案

12. /media, /mnt
放置隨插即用的裝置慣用目錄， `/mnt` 為管理員/使用者手動掛上（mount）的目錄

13. /opt
全名為 `optional`，通常為第三方廠商放置軟體處

14. /run
系統進行服務軟體運作管理處

15. /srv
通常是放置開發的服務（service），如：網站服務 `www` 等 

# 檔案與目錄管理指令
在 Ubuntu 中我們可以打開終端機進行指令操作，就可以透過指令來管理檔案。

一般指令格式如下：

```
$ 指令 [選項] [選項值]
```

![Linux Command 命令列指令與基本操作入門教學](/img/kdchang/cs101/linux-ls-g.png)

1. ls：list，查看檔案及子目錄

    列出基本資料夾資料：

    ```
    ls
    ```

    列出詳細資料和隱藏資料：

    ```
    // -l 列出詳細資料 -a 列出隱藏資料
    $ ls -la
    ```

    列出部分檔案：

    ```
    // 列出為 .js 的檔案
    $ ls *.js
    ```

2. pwd：print work directory，印出目前工作目錄

    ```
    $ pwd
    // /Users/happycoder/Desktop/projects/HappyCoder
    ```

3. cd：change directory，移動進入資料夾

    移動到目前資料夾下的 examples 資料夾：

    ```
    $ cd ./examples
    ```

    移動到家目錄：`~`：

    ```
    $ cd ~
    ```

    移動到上一層目錄 `..`：

    ```
    $ cd ..
    ```

    移動到根目錄 `/`：

    ```
    $ cd /
    ```

4. mkdir：make directory，創建新資料夾

    ```
    $ mkdir examples
    ```

5. cp：copy，複製檔案

    先將字串 TEST 存入 README.md 文件中

    ```
    $ echo "TEST" > README.md
    ```

    ```
    $ cp README.md
    ```

6. mv：move (rename) files，移動檔案或是重新命名檔案

    移動檔案：

    ```
    $ mv README.md /examples/README.md
    ```

    重新命名

    ```
    $ mv README.md README_MV.md
    ```

7. rm：remove file，刪除檔案

    ```
    $ rm README.md
    ```

    刪除目前資料夾下副檔名為 .js 檔案：

    ```
    $ rm *.js
    ```

    刪除資料夾和所有檔案：

    ```
    $ rm -f examples
    ```

8. touch：用來更新已存在文件的 timestamp 時間戳記或是新增空白檔案

    ```
    $ touch README.md
    ```

9. cat：將文件印出在終端機上

    ```
    $ cat README.md
    ```

10. tail：顯示檔案最後幾行內容

    ```
    $ tail README.md
    ```

    持續顯示更新內容，常用於 web server 看 log debug 使用：

    ```
    $ tail -f README.md
    ```

11. more：將檔案一頁頁印在終端機上

    可以使用上下移動換頁，按 q 離開：

    ```
    $ more README.md
    ```

12. file：檢查檔案類型

    ```
    $ file README.md
    // README.md: HTML document text, UTF-8 Unicode text
    ```

# 編輯文字檔案
1. nano：在終端機編輯文字檔案

    編輯或是新增文字檔案：

    ```
    $ nano README.md
    ```

    啟動編輯完後可以使用 Ctrl + X 離開，Ctrl + V 移動到上一頁，Ctrl + Y 移動到下一頁，Ctrl + W 搜尋文字內容 

2. vim：在終端機編輯文字檔案

    ```
    $ vim README.md
    ```

    啟動後，使用 i 進入編輯，esc 離開編輯模式，`:q` 不儲存離開，`:wq` 儲存離開，`:q!` 強制離開

# 檔案權限設定

在 Linux 系統中，每一個 Linux 檔案都具有四種存取權限：
1. 可讀取`（r，Readable）`，用數字 4 表示
2. 可寫入`（w，writable）`，用數字 2 表示
3. 可執行：`（x，eXecute）`，用數字 1 表示
4. 無權限（-），用數字 0 表示

系統管理者依據使用者需求來設定檔案權限，若我們想檢視檔案權限可以使用 `$ ls -l` 來查看

![Linux Command 命令列指令與基本操作入門教學](/img/kdchang/cs101/linux-ls-al.png)

1. 第一欄：使用者權限
    由 10 個字元組成，第一個字元表示檔案型態（- 為檔案，d 表示目錄，1 表示連結檔案）。字元 2、3、4 表示檔案擁有者的存取權限。字元 5、6、7 表示檔案擁有者所屬群組成員的存取權限。字元 8、9、10 表示其他使用者的存取權限

    舉例來說 -rwxrwxr--，代表這是一格檔案，擁有者和群組都具備讀取、寫入和執行權限，其他使用者只擁有讀取權限

2. 第二欄：檔案數量

3. 第三欄：擁有者

4. 第四欄：群組

5. 第五欄：檔案大小

6. 第六欄：檔案建立時間

7. 第七欄：檔案名稱

接下來介紹如何透過指令修改權限：

1. chmod：修改檔案權限

    將權限設為 `rw-rw-r--`：

    ```
    $ chmod 664 README.md
    ```

    將檔案的使用者和群組加入執行權限

    ```
    $ chmod ug+x README.md
    ```

2. chown：修改檔案擁有者與群組

    ```
    $ chown www-data:www-data README.md
    ```

# 系統管理
1. sudo：使用最高權限（superuser）執行指令，會要求輸入自己密碼，使用上必須非常小心

    ```
    $ sudo git clone xxx.py
    ```

2. su：su 指令可以讓一般的 Linux 使用者輸入 root 密碼取得 root 權限，暫時取得 root 權限的使用者就如同 root 一樣可以對系統進行各種變更動作

    ```
    $ su
    ```

3. kill：根據 Process ID 指定要終止程式

    ```
    $ kill PID
    ```

    立即強制執行：

    ```
    $ kill -9 PID
    ```

4. killall：直接使用程式的名稱來指定要終止的程式

    ```
    $ killall hello.py
    ```

# 套件管理
1. apt-get：套件管理工具

    更新套件資料庫列表：

    ```
    $ sudo apt-get update
    ```

    升級套件並下載安裝套件：

    ```
    $ sudo apt-get upgrade
    ```

    搜尋相關軟體套件（使用名稱）：

    ```
    $ apt-cache search --names-only fortune
    ```

    安裝套件：

    ```
    $ sudo apt-get install fortune
    ```

    移除套件：

    ```
    $ sudo apt-get remove fortune
    ```

# 網際網路相關操作
1. ping：網路檢測工具，透過發送 `ICMP ECHO_REQUEST` 的封包，檢查自己與特定設備之間的網路是否暢通，速度是否正常

    可輸入 hostname 或是 IP：

    ```
    $ ping google.com
    ```


    PING google.com (172.217.160.110): 56 data bytes
    64 bytes from 172.217.160.110: icmp_seq=0 ttl=57 time=7.037 ms
    64 bytes from 172.217.160.110: icmp_seq=1 ttl=57 time=9.411 ms
    64 bytes from 172.217.160.110: icmp_seq=2 ttl=57 time=22.690 ms
    64 bytes from 172.217.160.110: icmp_seq=3 ttl=57 time=6.561 ms
    64 bytes from 172.217.160.110: icmp_seq=4 ttl=57 time=6.909 ms
    64 bytes from 172.217.160.110: icmp_seq=5 ttl=57 time=6.311 ms
    64 bytes from 172.217.160.110: icmp_seq=6 ttl=57 time=6.860 ms
    64 bytes from 172.217.160.110: icmp_seq=7 ttl=57 time=6.583 ms


2. traceroutes：檢查從你的電腦到網路另一端的主機是走的什麼路徑

    ```
    $ traceroute google.com
    ```

    traceroute to google.com (172.217.27.142), 64 hops max, 52 byte packets
    1  zyxel.home (192.168.1.1)  2.047 ms  1.208 ms  1.888 ms
    2  h254.s98.ts.hinet.net (168.95.98.254)  6.189 ms  8.556 ms  5.875 ms
    3  168-95-85-2.hinet-ip.hinet.net (168.95.85.2)  7.057 ms  5.796 ms  5.998 ms
    4  211-22-226-1.hinet-ip.hinet.net (211.22.226.1)  9.766 ms  10.422 ms
    72.14.222.94 (72.14.222.94)  9.744 ms
    5  108.170.244.97 (108.170.244.97)  8.386 ms
    108.170.244.129 (108.170.244.129)  11.500 ms  12.247 ms
    6  209.85.142.13 (209.85.142.13)  7.015 ms  7.505 ms
    209.85.240.15 (209.85.240.15)  6.750 ms
    7  tsa03s02-in-f142.1e100.net (172.217.27.142)  11.478 ms  6.608 ms  6.893 ms

3. nslookup：查詢 DNS 回應是否正常

    ```
    $ nslookup google.com
    ```

    Server:		192.168.1.1
    Address:	192.168.1.1#53

    Non-authoritative answer:
    Name:	google.com
    Address: 216.58.200.238


# 其他好用指令
1. man：查詢 Linux 線上手冊（man page）

    ```
    $ man
    ```

    例如我們可以使用 `man` 來查詢 `ls` 的使用用法：

    ```
    $ man ls
    ```

2. find：查詢檔案

    在目前目錄下尋找檔名為 README.md 檔案

    ```
    $ find . -name README.md
    ```

3. grep：強大文件字串搜尋工具

    ```
    $ grep '找這個字串' file_name
    ```

    找所有目錄（含子目錄）下檔案

    ```
    $ grep -r '字串' *

    ```

4. crontab：例行性工作排程

    編輯 crontab

    ```
    $ crontab -e
    ```

    crontab 格式：

    ```
    分 時 日 月 星期 要執行的指令
    
    30 12   *   *   *  python /projects/hello.py &

    在 12:30 時執行 hello.py，& 表示背景執行
    * 號表示每日每月每星期都執行
    ```

# 撰寫第一個 shell script
Shell 是我們和 Linux 系統的介面，我們可以透過終端機在上面輸入指令和作業系統互動，讓他做我們想做的事情。在 Linux 中標準的 Shell 為（Bourne Again Shell），檔案路徑為 `/bin/sh`，我們可以透過 `$ echo $SHELL` 去印出目前使用的 shell

其中 Shell Script 為使用 shell 所提供的語法所撰寫的程式碼，其為直譯式不用編譯。可以讓你將複雜或是重複性的指令寫成程式碼檔案


```
$ vim example.sh
```

在編輯模式輸入以下程式碼，：

```
#!/bin/bash
# 這是註解，上面所使用的 shell

echo "日期"
date
echo "印出檔案列表"
ls -l
```

修改權限成可以執行：

```
$ chmod u+x example.sh
```

接著執行，若是一切順利就可以在終端機看到時間日期和檔案列表！恭喜你完成你的第一個 shell script！

```
$ ./example.sh
```

# 總結
以上介紹了 Linux/Ubuntu 常用指令和基礎操作的入門教學介紹上，實際上讀者不用刻意去背誦，而是在實際上操作中練習，多累積撰寫程式並使用指令碼去加快程式開發的速度自然而然就會把指令碼記憶起來了，若是真的忘記再去網路上查找就好，加油！

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 

# 參考文件
1. [初窥Linux 之 我最常用的20条命令](http://blog.csdn.net/ljianhui/article/details/11100625)
2. [【Ｌ】Linux 常用指令集](http://blog.xuite.net/chingwei/blog/16285667-%E3%80%90%EF%BC%AC%E3%80%91Linux+%E5%B8%B8%E7%94%A8%E6%8C%87%E4%BB%A4%E9%9B%86)
3. [Linux 的 su 與 sudo 指令教學與範例](https://blog.gtwang.org/linux/sudo-su-command-tutorial-examples/)
4. [dig、host 與 nslookup 指令的查詢語法](http://www.vixual.net/blog/archives/100)
5. [nslookup — 查詢 DNS 指令](https://www.phpini.com/linux/nslookup-query-dns-command)
6. [在 Linux 下使用 find 指令查詢目錄與檔案的速查筆記](https://blog.miniasp.com/post/2010/08/27/Linux-find-command-tips-and-notice.aspx)
7. [grep 搜尋目錄下所有檔案字串](https://www.phpini.com/linux/grep-directory-all-files)
8. [第十五章、例行性工作排程(crontab)](http://linux.vbird.org/linux_basic/0430cron.php)

（image via [unixmen](https://www.javatpoint.com/linux/img/kdchang/linux-ls-g.png)、[cloudxlab](https://s3.amazonaws.com/cloudxlab/static/img/kdchang/aha/ls-al.png)、[ytimg](https://i.ytimg.com/vi/GGorVpzZQwA/maxresdefault.jpg)、[ytimg](https://i.ytimg.com/vi/DPIPC25xzUM/maxresdefault.jpg)、[linux](https://3c1703fe8d.site.internapcdn.net/newman/gfx/news/hires/2014/linux.jpg)、[imgur](https://i.stack.imgur.com/RUIIq.png)）
