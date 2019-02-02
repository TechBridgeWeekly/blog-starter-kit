---
title: Linux Curl Command 指令與基本操作入門教學
date: 2019-02-01 10:23:23
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
    - curl
---

![Linux Curl Command 指令與基本操作入門教學](/img/kdchang/cs101/terminal.jpg)

# 前言
身為一個 Web 開發者，往往需要開發不同的 Restful API （Application Programming Interface）來存取資源。在開發完 API 後測試則會使用類似 [Postman](https://www.getpostman.com/) 的測試工具來進行測試。除了使用 Postman 等 GUI Tool 外，身為一個軟體工程師，當然要學會使用 Linux 指令中的 Curl！這邊我們整理了 Curl 常用的指令，讓大家可以重新溫習，讀者也可以分享自己的常用的指令來提升工作效率。好，那我們就開始吧！

# Curl 指令基本介紹與常見用法
Curl 是一個在 Linux 上用來透過 HTTP Protocol（HTTP HyperText Transfer Protocol 定義存取網路資源的協定，讓我們可以使用 client / server 模式來取得網路資源）下載和上傳檔案的指令（比起 wget 只能下載強大許多）。它基本的指令格式如下：

```
curl [options] [URL...]
```

新手上路，我們來看看最基本的用法：

打開終端機（terminal）然後，curl 後面加網址，就會在終端機內顯示回傳的 response，可能是 HTML、JSON 或是 XML 等格式，根據輸入的 URL 內容而定。 

```
curl https://www.google.com
```

例如我想要下載：黃色小鴨的圖片，可以使用 `-o` 搭配欲下載的檔名和網址

```
curl -o duck.jpg https://im2.book.com.tw/image/getImage?i=https://www.books.com.tw/img/N00/040/56/N000405619.jpg&v=522ff1cf&w=348&h=348
```

若是使用 `-O` 則可以直接使用下載網址的檔案檔名來命名下載的檔案（N000405619.jpg）：

```
curl -O https://im2.book.com.tw/image/getImage?i=https://www.books.com.tw/img/N00/040/56/N000405619.jpg&v=522ff1cf&w=348&h=348
```

有可能在下載過程中被中斷，若是想要從中斷的地方繼續的話，可以使用 `-C` 選項：

```
 curl -C - -O http://releases.ubuntu.com/18.04/ubuntu-18.04-desktop-amd64.iso
```

若希望可以跟隨著網址 301/302 redirect 的話，可以使用 `-L` 選項：

```
curl -L http://google.com
```

可以比較沒有使用 -L 的回應：

```
$ curl http://google.com
<HTML><HEAD><meta http-equiv="content-type" content="text/html;charset=utf-8">
<TITLE>301 Moved</TITLE></HEAD><BODY>
<H1>301 Moved</H1>
The document has moved
<A HREF="http://www.google.com/">here</A>.
</BODY></HTML>
```

若我們要追蹤整個 curl 過程並將結果存入 debugdump.txt 檔案可以使用 `--trace-ascii` 指令：

```
curl --trace-ascii debugdump.txt http://www.example.com/
```
```
$ cat debugdump.txt
== Info:   Trying 93.184.216.34...
== Info: TCP_NODELAY set
== Info: Connected to www.example.com (93.184.216.34) port 80 (#0)
=> Send header, 142 bytes (0x8e)
0000: GET / HTTP/1.1
0010: Host: www.example.com
0027: User-Agent: Mozilla/5.0 (compatible; MSIE 9.0; Windows NT 6.1; T
0067: rident/5.0)
0074: Accept: */*
0081: Referer:
008c:
<= Recv header, 17 bytes (0x11)
0000: HTTP/1.1 200 OK
<= Recv header, 31 bytes (0x1f)
0000: Cache-Control: max-age=604800
<= Recv header, 40 bytes (0x28)
0000: Content-Type: text/html; charset=UTF-8
<= Recv header, 37 bytes (0x25)
0000: Date: Sat, 02 Feb 2019 13:54:23 GMT
<= Recv header, 31 bytes (0x1f)
0000: Etag: "1541025663+gzip+ident"
<= Recv header, 40 bytes (0x28)
0000: Expires: Sat, 09 Feb 2019 13:54:23 GMT
<= Recv header, 46 bytes (0x2e)
0000: Last-Modified: Fri, 09 Aug 2013 23:54:35 GMT
<= Recv header, 24 bytes (0x18)
0000: Server: ECS (sjc/4E44)
<= Recv header, 23 bytes (0x17)
0000: Vary: Accept-Encoding
<= Recv header, 14 bytes (0xe)
0000: X-Cache: HIT
<= Recv header, 22 bytes (0x16)
0000: Content-Length: 1270
<= Recv header, 2 bytes (0x2)
0000:
<= Recv data, 1270 bytes (0x4f6)
0000: <!doctype html>.<html>.<head>.    <title>Example Domain</title>.
0040: .    <meta charset="utf-8" />.    <meta http-equiv="Content-type
0080: " content="text/html; charset=utf-8" />.    <meta name="viewport
00c0: " content="width=device-width, initial-scale=1" />.    <style ty
0100: pe="text/css">.    body {.        background-color: #f0f0f2;.
0140:      margin: 0;.        padding: 0;.        font-family: "Open S
0180: ans", "Helvetica Neue", Helvetica, Arial, sans-serif;.        .
01c0:    }.    div {.        width: 600px;.        margin: 5em auto;.
0200:        padding: 50px;.        background-color: #fff;.        bo
0240: rder-radius: 1em;.    }.    a:link, a:visited {.        color: #
0280: 38488f;.        text-decoration: none;.    }.    @media (max-wid
02c0: th: 700px) {.        body {.            background-color: #fff;.
0300:         }.        div {.            width: auto;.            mar
0340: gin: 0 auto;.            border-radius: 0;.            padding:
0380: 1em;.        }.    }.    </style>    .</head>..<body>.<div>.
03c0: <h1>Example Domain</h1>.    <p>This domain is established to be
0400: used for illustrative examples in documents. You may use this.
0440:   domain in examples without prior coordination or asking for pe
0480: rmission.</p>.    <p><a href="http://www.iana.org/domains/exampl
04c0: e">More information...</a></p>.</div>.</body>.</html>.
== Info: Curl_http_done: called premature == 0
== Info: Connection #0 to host www.example.com left intact
```

# 使用 Curl 來進行 HTTP Request
除了簡易的下載檔案或是取得網頁內容外，Curl 還支援各種不同 HTTP 請求方法（HTTP method），以下列出常用指令和選項參數：

```
-X/--request [GET|POST|PUT|DELETE|PATCH]  使用指定的 http method 來發出 http request
-H/--header                           設定 request 裡所攜帶的 header
-i/--include                          在 output 顯示 response 的 header
-d/--data                             攜帶 HTTP POST Data 
-v/--verbose                          輸出更多的訊息方便 debug
-u/--user                             攜帶使用者帳號、密碼
-b/--cookie                           攜帶 cookie（可以是參數或是檔案位置）
```

1. GET

簡易一個 URL 版本，可以攜帶 query string 參數取得網路資源：

```
$ curl https://example.com?q1=123&q2=abc
```

在同一個指令使用多個 URL：

```
$ curl http://example1.com http://example2.com
```

2. Form POST

一般而言我們 Form 表單的 HTML 會長這樣：

```
 <form method="POST" action="form.php">
    <input type=text name="email">
    <input type=submit name=press value=" OK ">
 </form>
```

由於 Form post 是使用 `application/x-www-form-urlencoded` Content-Type，所以傳遞的值需要編碼：

```
＄ curl -X POST --data "email=test@example.com&press=%20OK%20"  http://www.example.com/form.php
```

3. File Upload POST
另一種常見 Form 表單是有涉及檔案上傳（使用 multipart/form-data Content-Type）：

```
<form method="POST" enctype='multipart/form-data' action="upload.php">
 <input type=file name=upload>
 <input type=submit name=press value="OK">
</form>
```

可以看到指令會需要攜帶 upload 檔案：

```
$ curl -X POST -F 'file=@./upload.txt' http://www.example.com/upload.php
```

4. 常見 Restful CRUD 指令：

    1. GET 單一和全部資源
    ```
    $ curl -X GET "http://www.example.com/api/resources"
    $ curl -X GET "http://www.example.com/api/resources/1"
    ```

    2. POST JSON 資料： 
    ```
    $ curl -X POST -H "Content-Type: application/json" -d '{"status" : false, "name" : "Jack"}' "http://www.example.com/api/resources"
    ```

    3. PUT JSON 資料： 
    ```
    $ curl -X PUT -H "Content-Type: application/json" -d '{"status" : false }' "http://www.example.com/api/resources"
    ```

    4. DELETE 資源：
    ```
    $ curl -X DELETE "http://www.example.com/api/resources/1"
    ```

5. 攜帶 cookie

在指令中輸入 cookie：
```
$ curl --cookie "name=Jack" http://www.example.com
```

從檔案讀取 cookie：

```
$ curl --cookie stored_cookies_file_path http://www.example.com
```

6. 攜帶 User Agent

```
$ curl --user-agent "Mozilla/5.0 (compatible; MSIE 5.01; Windows NT 5.0)" http://www.example.com

```

7. Basic Authentication

若所存取網頁有使用 Basic Authentication，可以攜帶 `--user username:password` 來通過驗證：

```
$ curl -i --user secret:vary_secret http://www.example.com/api/resources
```


# 總結
以上介紹了 Linux Curl 常用指令和基礎操作的入門教學介紹：

1. GET/POST/PUT/DELETE 操作
2. 下載檔案
3. Form 表單操作
4. 檔案上傳
5. Restful CRUD 指令
6. User Agent
7. Basic Authentication

之後讀者除了使用 Postman 等 GUI Tool 外，也可以使用 Curl 來進行 HTTP endpoint 的測試和開發上！讀者也可以分享自己的常用的指令來提升工作效率！

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 

# 參考文件
1. [Upload files with CURL](https://medium.com/@petehouston/upload-files-with-curl-93064dcccc76)

（image via [linuxinsider](https://www.linuxinsider.com/story/84356.html)）
