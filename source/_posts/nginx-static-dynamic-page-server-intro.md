---
title: Nginx + Flask 動態與靜態頁面分離入門教學
date: 2018-08-11 09:54:49
author: kdchang
tags:
   - Python
   - Flask
   - Nginx
---

![ Nginx + Flask 動態與靜態頁面分離入門教學](/img/kdchang/nginx/nginx-logo.png)


# 前言
Nginx（發音：engine x）是一個非同步的 Web 伺服器，也可以用作反向代理，負載平衡器和 HTTP 快取等功能。Flask 則是一個使用 Python 編寫的輕量級 Web 應用框架。 

有些讀者可能是學生或是事業剛起步的 startup，往往為了省錢或是開發方便會需要將 web 前後端都放在同一台機器上，本文希望透過簡單範例來建立一個前後端放在同一台 server 的入門教學，其中靜態檔案由 nginx 來負責 serve，而動態檔案則由 flask app + 開發用伺服器來提供 api endpoint（這邊因為教學方便，一般 production 上建議使用 uwsgi 或是 gunicorn，不要使用開發用伺服器）

# 環境建置
工欲善其事，必先利其器。在開發應用程式之前往往需要設定好環境，這邊我們使用一台 Ubuntu 18.04 LTS server，安裝 python 和虛擬開發環境以及 nginx 等相關套件等。

```
$ sudo apt install python3-pip python3-dev build-essential libssl-dev libffi-dev python3-setuptools
$ sudo apt install nginx
$ sudo apt install python3-venv
```

# 設定 nginx config
這邊為了方便我們直接更改 site-available/default config 如一下檔案設定（site-available 下可存多個設定，真正使用是在 site-enabled 資料夾下，通常會用 link 連結過去）：
```/etc/nginx/site-available/default
server {
        listen 80;
        root /var/www; # serve 靜態檔案位置
        index index.html;
        location ^~ /api/v1 { # 當 api prefix request 過來會 proxy 給 5000 port 的 local api service
                proxy_pass http://127.0.0.1:5000;
                proxy_set_header Host $host; # 在 proxy request 時保留 client 的 header
        }
}
```

重新讀取 config 檔案
```
$ sudo nginx -s reload
```
若不是使用 default config 檔案，也可以自己開一個新的檔案設定 config 檔案，最後要把設定好的 demo.conf link 到真正在使用的 sites-enabled config 資料夾
```
$ sudo ln -s /etc/nginx/sites-available/demo.conf /etc/nginx/sites-enabled/
```
# 撰寫測試用靜態網頁
撰寫一個測試靜態頁面，正式情況可能會是一個 build 好的 html 檔案，裡面有含 HTML/CSS/JS 等前端靜態檔案（可能有使用前端框架）

```html /var/www/index.html
<h1>Hello Nginx based static Page!!!</h1>
```

這時候在瀏覽器觀看你的 server ip 應該會看到下列靜態檔案畫面：
![ Nginx + Flask 動態與靜態頁面分離入門教學](/img/kdchang/nginx/frontend.png)

# 撰寫 Flask 動態網站
接著我們來撰寫後端 api service，首先先建立虛擬開發環境和安裝套件：
```
$ sudo python3.6 -m venv venv
$ source venv/bin/activate
$ sudo pip3 install flask
```

index.py 內容，設計一個測試用 api endpoint，提供 GET movies list
```py /var/www/index.py
from flask import Flask, jsonify
app = Flask(__name__)

@app.route("/api/v1/movies")
def get_api():
    movies = [{'name': '刺激1995'}, {'name': '教父'}]
    return jsonify(movies)
```

# 成果展現
這時候在虛擬環境下執行資料夾的 index.py 檔案：
```
$ FLASK_APP=index.py flask run
```

這時候在瀏覽器觀看你的 SERVER_ID/api/v1/movies，應該會看到 fake api 回傳的 json 電影資料：
![ Nginx + Flask 動態與靜態頁面分離入門教學](/img/kdchang/nginx/backend.png)


# 總結
以上我們討論了簡單的 nginx + flask 動/靜態檔案分離的 server 建置入門教學，若是更複雜的架構也可以把系統進行真正的前後端分離甚至是資料庫的讀寫分離等。但礙於篇幅我們今天就介紹到這邊，關於更多系統設計和架構設計的討論我們下次見囉～

# 延伸閱讀
1. [Flask 官網](http://flask.pocoo.org/)
2. [Nginx 官網](https://www.nginx.com/)

（image via [matplotlib](https://matplotlib.org/_static/logo2.svg)

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 