---
title: Vatic 安裝教學
date: 2019-03-16 15:00:50
tags: Deep Learning, Computer Vision, Video labeling
author: pojenlai
---

## 前言

Vatic 是用來標註影片中 bounding box 的好用工具，因為它提供了 bounding box tracking 的功能，不需要一個一個 frame 去重新標註，所以想用 Machine Learning 學習影片中 bounding box 的人來說是很好用的工具。

![img](https://i.imgur.com/3vTE1dO.jpg)

因為 vatic 的安裝步驟有一點複雜，所以寫這篇教學提供給需要的讀者朋友。筆者自己有在 Ubuntu 16.04 裝成功，這篇教學主要會 cover 到如何在自己電腦上 locally 跑起來 vatic。

## 開始安裝

首先，大家可以先看看官方 [tutorial](https://github.com/cvondrick/vatic/tree/contrib#installation)，裡面的步驟就是我們要做的，只是如果只按照裡面的步驟做，會遇到不少的 error，所以接下來我們會補充一些相關的經驗。

## Download

官方 tutorial 中會叫大家下載 install.sh 檔，但是他們提供的網址已經無法連上，這邊建議大家可以使用 [此連結](https://github.com/cvondrick/vatic/blob/contrib/vatic-install.sh) 裡面的檔案，是熱心的網友幫忙更新過的 installation script，裝起來比較沒有問題。

## Apache server setup

這邊提供大家一個修改完的 /etc/apache2/sites-enabled/000-default.conf 範例：

```
WSGIDaemonProcess www-data
WSGIProcessGroup www-data

<VirtualHost *:80>
ServerName localhost
DocumentRoot /home/icaros/tool/vatic_home/vatic/public

WSGIScriptAlias /server /home/icaros/tool/vatic_home/vatic/server.py
CustomLog /var/log/apache2/access.log combined
</VirtualHost>
```

當你改完上面的檔案，想要重新啟動 apache 時，可能會遇到下面的 error:

```
icaros@icaros-MS-7B61:/etc/apache2/sites-enabled$ sudo apache2ctl graceful
[sudo] password for icaros:
AH00558: apache2: Could not reliably determine the server's fully qualified domain name, using 127.0.1.1. Set the 'ServerName' directive globally to suppress this message
```

這時候可以參考這篇回答 - [Apache error “Could not reliably determine the server's fully qualified domain name”](https://askubuntu.com/a/396048)，就可以繼續進行。

## Database setup

當設定完 apache server 後，下一步我們要來設定儲存 annotation 需要用到的 database server。只是，當你按照官方教學的第一步做下去，你就會發現這個 error：

```
icaros@icaros-MS-7B61:~/tool/vatic_home/vatic/public$ mysql -u root
ERROR 1045 (28000): Access denied for user 'root'@'localhost' (using password: NO)
```

當你遇到這個情況，別擔心，請下 `mysql -u root -p`，然後密碼請輸入 `hail_ukraine`，就可以進去 mysql 的命令視窗了：

```
icaros@icaros-MS-7B61:~/tool/vatic_home/vatic/public$ mysql -u root -p
Enter password:
Welcome to the MySQL monitor. Commands end with ; or \g.
Your MySQL connection id is 20
Server version: 5.7.25-0ubuntu0.16.04.2 (Ubuntu)

Copyright (c) 2000, 2019, Oracle and/or its affiliates. All rights reserved.

Oracle is a registered trademark of Oracle Corporation and/or its
affiliates. Other names may be trademarks of their respective
owners.

Type 'help;' or '\h' for help. Type '\c' to clear the current input statement.

mysql>
```

然後就可以建立 vatic 這個 database 並離開 mysql：

```
mysql> create database vatic;
mysql> quit
```

## Setup

接下來要處理 config.py，這邊提供大家一個範例：

```
signature = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" # AWS secret access key
accesskey = "xxxxxxxxxxxxxxxxxxxx" # AWS access key ID
sandbox = True # if true, put on workersandbox.mturk.com
localhost = "http://localhost/" # your local host
database = "mysql://root@localhost/vatic" # server://user:pass@localhost/dbname
geolocation = "d4c08cbcd873067102638cd6f1bf3c5d63ff053a87a4148273159c3ed1493b04" # api key for ipinfodb.com
maxobjects = 25;

# probably no need to mess below this line

import multiprocessing
processes = multiprocessing.cpu_count()

import os.path
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
```

改完上面的範例，執行 `turkic setup --database`，很可能會遇到下一個 error：

```
icaros@icaros-MS-7B61:~/tool/vatic_home/vatic$ turkic setup --database
Traceback (most recent call last):
File "/usr/local/bin/turkic", line 4, in <module>
__import__('pkg_resources').run_script('turkic==0.2.5', 'turkic')
File "/usr/local/lib/python2.7/dist-packages/pkg_resources/__init__.py", line 666, in run_script
self.require(requires)[0].run_script(script_name, ns)
File "/usr/local/lib/python2.7/dist-packages/pkg_resources/__init__.py", line 1446, in run_script
exec(code, namespace, namespace)
File "/usr/local/lib/python2.7/dist-packages/turkic-0.2.5-py2.7.egg/EGG-INFO/scripts/turkic", line 12, in <module>
import cli
File "/home/icaros/tool/vatic_home/vatic/cli.py", line 19, in <module>
import qa
File "/home/icaros/tool/vatic_home/vatic/qa.py", line 1, in <module>
from match import match
File "/home/icaros/tool/vatic_home/vatic/match.py", line 1, in <module>
import munkres
File "/usr/local/lib/python2.7/dist-packages/munkres-1.1.2-py2.7.egg/munkres.py", line 79
def pad_matrix(self, matrix: Matrix, pad_value: int=0) -> Matrix:
^
SyntaxError: invalid syntax
```

會遇到這個 error 是因為 munkres 這個 library 在 1.1.0 版之後就不支援 Python 2 了，但 vatic 似乎都是用 Python 2.7 寫的，所以請用 `pip install munkres==1.0.12` 安裝較舊的版本。

好了，處理完 munkres 之後，你又會遇到下一個 error：

```
icaros@icaros-MS-7B61:~/tool/vatic_home/vatic$ turkic setup --database
Traceback (most recent call last):
File "/usr/local/bin/turkic", line 4, in <module>
__import__('pkg_resources').run_script('turkic==0.2.5', 'turkic')
File "/usr/local/lib/python2.7/dist-packages/pkg_resources/__init__.py", line 666, in run_script
self.require(requires)[0].run_script(script_name, ns)
File "/usr/local/lib/python2.7/dist-packages/pkg_resources/__init__.py", line 1446, in run_script
exec(code, namespace, namespace)
File "/usr/local/lib/python2.7/dist-packages/turkic-0.2.5-py2.7.egg/EGG-INFO/scripts/turkic", line 16, in <module>
turkic.cli.main()
File "/usr/local/lib/python2.7/dist-packages/turkic-0.2.5-py2.7.egg/turkic/cli.py", line 147, in main
handler(args[1:])
File "/usr/local/lib/python2.7/dist-packages/turkic-0.2.5-py2.7.egg/turkic/cli.py", line 47, in __init__
self(parser.parse_args(args))
File "/usr/local/lib/python2.7/dist-packages/turkic-0.2.5-py2.7.egg/turkic/cli.py", line 470, in __call__
self.database(args)
File "/usr/local/lib/python2.7/dist-packages/turkic-0.2.5-py2.7.egg/turkic/cli.py", line 455, in database
database.install()
File "/usr/local/lib/python2.7/dist-packages/turkic-0.2.5-py2.7.egg/turkic/database.py", line 45, in install
Base.metadata.create_all(engine)
File "build/bdist.linux-x86_64/egg/sqlalchemy/sql/schema.py", line 4287, in create_all
File "build/bdist.linux-x86_64/egg/sqlalchemy/engine/base.py", line 2032, in _run_visitor
File "/usr/lib/python2.7/contextlib.py", line 17, in __enter__
return self.gen.next()
File "build/bdist.linux-x86_64/egg/sqlalchemy/engine/base.py", line 2024, in _optional_conn_ctx_manager
File "build/bdist.linux-x86_64/egg/sqlalchemy/engine/base.py", line 2226, in _contextual_connect
File "build/bdist.linux-x86_64/egg/sqlalchemy/engine/base.py", line 2266, in _wrap_pool_connect
File "build/bdist.linux-x86_64/egg/sqlalchemy/engine/base.py", line 1536, in _handle_dbapi_exception_noconnection
File "build/bdist.linux-x86_64/egg/sqlalchemy/util/compat.py", line 383, in raise_from_cause
File "build/bdist.linux-x86_64/egg/sqlalchemy/engine/base.py", line 2262, in _wrap_pool_connect
File "build/bdist.linux-x86_64/egg/sqlalchemy/pool/base.py", line 354, in connect
File "build/bdist.linux-x86_64/egg/sqlalchemy/pool/base.py", line 751, in _checkout
File "build/bdist.linux-x86_64/egg/sqlalchemy/pool/base.py", line 483, in checkout
File "build/bdist.linux-x86_64/egg/sqlalchemy/pool/impl.py", line 138, in _do_get
File "build/bdist.linux-x86_64/egg/sqlalchemy/util/langhelpers.py", line 68, in __exit__
File "build/bdist.linux-x86_64/egg/sqlalchemy/pool/impl.py", line 135, in _do_get
File "build/bdist.linux-x86_64/egg/sqlalchemy/pool/base.py", line 299, in _create_connection
File "build/bdist.linux-x86_64/egg/sqlalchemy/pool/base.py", line 428, in __init__
File "build/bdist.linux-x86_64/egg/sqlalchemy/pool/base.py", line 630, in __connect
File "build/bdist.linux-x86_64/egg/sqlalchemy/engine/strategies.py", line 114, in connect
File "build/bdist.linux-x86_64/egg/sqlalchemy/engine/default.py", line 453, in connect
File "build/bdist.linux-x86_64/egg/MySQLdb/__init__.py", line 81, in Connect
File "build/bdist.linux-x86_64/egg/MySQLdb/connections.py", line 193, in __init__
sqlalchemy.exc.OperationalError: (_mysql_exceptions.OperationalError) (1045, "Access denied for user 'root'@'localhost' (using password: NO)")
(Background on this error at: http://sqlalche.me/e/e3q8)
```

這個 error 是源自於 mysql 需要 password 才能 access，但你可以用下面的指令將 password 設成空的：

```
use mysql
update user set authentication_string=password('') where user='root';
flush privileges;
```

以下是執行範例：

```
mysql> use mysql
Reading table information for completion of table and column names
You can turn off this feature to get a quicker startup with -A

Database changed
mysql> update user set authentication_string=password('') where user='root';
Query OK, 1 row affected (0.00 sec)
Rows matched: 1 Changed: 1 Warnings: 0

mysql> flush privileges;
Query OK, 0 rows affected (0.00 sec)

mysql> quit
Bye
```

當你再執行一次 `turkic setup --database`，你就可以看到成功設定 database 了：

```
icaros@icaros-MS-7B61:~/tool/vatic_home/vatic$ turkic setup --database
Installed new tables, if any.
```

## Verification

接下來要執行 `turkic status --verify`（如果你還沒有要跟 MTurk 綁定，你可以忽略 Amazon Mechanical Turk 的 error）：

```
icaros@icaros-MS-7B61:~/tool/vatic_home/vatic$ turkic status --verify
Configuration:
Sandbox: True
Database: mysql://root@localhost/vatic
Localhost: http://localhost/

Testing access to Amazon Mechanical Turk... OK
Testing access to database server... OK
Testing access to web server... ERROR! HTTP Error 403: Forbidden

One or more tests FAILED!
```

Amazon 的 MTurk 是一個 crowd labeling 的平台，你可以將 labeling 的工作放到網站上，讓世界上各地的人來為你標註 data，省下許多時間。

可是即使忽略 MTurk，還是有 web server error，這時候你會需要改一下 /etc/apache2/apache2.conf，加入下面幾行：

```
<Directory /your_path_to_vatic/public>
Options Indexes FollowSymLinks
AllowOverride All
Require all granted
</Directory
```

然後重新啟動 apache（`sudo apache2ctl graceful`），應該就可以成功啦：

```
icaros@icaros-MS-7B61:~/tool/vatic_home/vatic$ turkic status --verify
Configuration:
Sandbox: True
Database: mysql://root@localhost/vatic
Localhost: http://localhost/

Testing access to Amazon Mechanical Turk... OK
Testing access to database server... OK
Testing access to web server... OK

All tests passed!
```

這時候在瀏覽器輸入 localhost 就可以看到 vatic 的畫面。

## Frame extraction

在把自己的 video frame extract 出來時，記得要在 vatic 的目錄下執行 turkic，不然會出現 error。

## Publish in offline mode

最後一步就是在自己電腦上 offline publish 剛剛弄好的 video：

```
turkic load identifier_off /home/icaros/Downloads/turkic_frames/my_video/ apple --offline
turkic publish --offline
```

你應該可以看到下列的 output：

```
icaros@icaros-MS-7B61:~/tool/vatic_home/vatic$ turkic load identifier_off /home/icaros/Downloads/turkic_frames/my_video/ apple --offline
Checking integrity...
Searching for last frame...
Found 9320 frames.
Binding labels and attributes...
Creating symbolic link...
Creating segments...
Video loaded and ready for publication.
icaros@icaros-MS-7B61:~/tool/vatic_home/vatic$ turkic publish --offline
http://localhost?id=65&hitId=offline
http://localhost?id=66&hitId=offline
http://localhost?id=67&hitId=offline
http://localhost?id=68&hitId=offline
http://localhost?id=69&hitId=offline
http://localhost?id=70&hitId=offline
http://localhost?id=71&hitId=offline
http://localhost?id=72&hitId=offline
http://localhost?id=73&hitId=offline
http://localhost?id=74&hitId=offline
http://localhost?id=75&hitId=offline
http://localhost?id=76&hitId=offline
http://localhost?id=77&hitId=offline
http://localhost?id=78&hitId=offline
http://localhost?id=79&hitId=offline
http://localhost?id=80&hitId=offline
http://localhost?id=81&hitId=offline
http://localhost?id=82&hitId=offline
http://localhost?id=83&hitId=offline
http://localhost?id=84&hitId=offline
http://localhost?id=85&hitId=offline
http://localhost?id=86&hitId=offline
http://localhost?id=87&hitId=offline
http://localhost?id=88&hitId=offline
http://localhost?id=89&hitId=offline
http://localhost?id=90&hitId=offline
http://localhost?id=91&hitId=offline
http://localhost?id=92&hitId=offline
http://localhost?id=93&hitId=offline
http://localhost?id=94&hitId=offline
http://localhost?id=95&hitId=offline
http://localhost?id=96&hitId=offline
```

要 access 你的 task，去瀏覽器輸入上面任意一個 url，例如 `http://localhost/?id=86&hitId=offline` 即可。 但如果你看到 server error, 請在 000-default.conf 檔案中加上 `Directory tag` 並重啟 apache：

```
WSGIDaemonProcess www-data
WSGIProcessGroup www-data

<VirtualHost *:80>
ServerName localhost
DocumentRoot /home/icaros/tool/vatic_home/vatic/public

WSGIScriptAlias /server /home/icaros/tool/vatic_home/vatic/server.py
CustomLog /var/log/apache2/access.log combined
</VirtualHost>

<Directory /home/icaros/tool/vatic_home/vatic>
<Files server.py>
Require all granted
</Files>
</Directory>
```

這時候再去瀏覽器輸入 `http://localhost?id=86&hitId=offline`，你就可以看到 annotation 的視窗了：

![img](https://i.imgur.com/3vTE1dO.jpg)

## 總結

今天教大家要怎麼從無到有 setup vatic 這個可以更方便標註 video 中 bounding box 的工具，希望對有相關需求的讀者有幫助！

## 延伸閱讀

1. [BeaverDam: Video annotation tool for deep learning training labels](https://github.com/antingshen/BeaverDam)
2. [Tool for annotating and evaluating video object detection or tracking ?](https://www.researchgate.net/post/Tool_for_annotating_and_evaluating_video_object_detection_or_tracking)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
