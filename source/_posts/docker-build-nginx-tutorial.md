---
title: 使用 Docker 建立 nginx 伺服器入門教學
date: 2018-03-17 11:23:23
author: kdchang
tags:
    - docker
    - nginx
    - server
    - container
    - apache
    - iamge
---

![使用 Docker 建立 nginx 伺服器入門教學](/img/kdchang/docker101/docker-logo.png)

# 前言
相信一般開發者每次開發應用程式時最討厭的就是設定環境。往往裝好環境時開發應用的熱情往往被消磨一半，不過如今有了 [Docker](https://www.docker.com/) 不但可以提高開發效率也可以更容易管理整個服務架構，發展 micro service！

# Docker 基本概念
Docker 和傳統在硬體做虛擬化的方式不同，其主要在作業系統層做虛擬化且在主流的作業系統和雲端平台上都可以使用（例如：Linux、MacOS 和 Windows），不同作業系統 Docker 相關安裝方式可以參考[官方網站](https://docs.docker.com/install/)。

![使用 Docker 建立 nginx 伺服器入門教學](/img/kdchang/docker101/docker-vm-container.png)

關於 Docker 基本有三大主軸概念：
1. Docker 映像檔（Image）：類似於虛擬機的映像檔，是一個可以創建容器的模版（template）
2. Docker 容器（Container）：類似於一個輕量級 sandbox。Docker 是透過容器來運行並隔離應用
3. Docker 倉庫（Repository）：類似於程式碼儲存地，可以想成是 Docker 版的 git repo。而 [Docker Hub](https://hub.docker.com/) 類似於 Docker 版的 Gihub，有許多 repo 在上面分享

# 使用 Docker 建立 Nginx 伺服器

![使用 Docker 建立 nginx 伺服器入門教學](/img/kdchang/docker101/linux-vs-docker-comparison-architecture-docker-components.png)

1. 使用官方 nginx image 運行 docker container

    將 nginx image 跑起來成為一個 webserver container，並把 docker container 80 port 對應到本機端的 0.0.0.0:7777

    ```
    $ docker run -d -p 7777:80 --name webserver nginx
    ```

    若本地端沒有 image 則會從遠端下載：
    Unable to find image 'nginx:latest' locally
    latest: Pulling from library/nginx

2. 檢查 Docker container 是否運行（有可能需要 sudo 權限）

    ```
    $ docker ps
    ```

    此時在瀏覽器 http://localhost:7777 應該可以看到 nginx 伺服器的首頁


3. 替換 nginx 首頁

    在目前資料夾建立一個 index.html 檔案：

    ```
    <!DOCTYPE html>
    <html lang="en">
        <head>
            <title></title>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1">
        </head>
        <body>
            <h1>Hi Nginx Docker</h1>
        </body>
    </html>
    ```

    記得把之前的 docker container 先 docker stop webserver 暫停然後輸入：

    （若遇到已經創建同樣名稱 container 時可以先輸入 docker stop CONTAINER_NAMES） 後 docker rm CONTAINER_NAMES）

    ```
    $ docker run --name nginx-container -p 7777:80 -v index.html:/usr/share/nginx/html:ro -d nginx
    ```

    此時在瀏覽器 http://localhost:7777 即可以看到 nginx 伺服器的首頁被換成我們 index.html 內容

4. 使用 Dockerfile 建立新的映像檔：

    ```
    FROM nginx
    COPY ./index.html /usr/share/nginx/html
    ```

    開始根據目錄下 Dockerfile 建立映像檔

    ```
    $ docker build . -t my-nginx-image
    ```

    成功後可以執行 run 動作，但記得把之前的 docker container 先 stop 暫停

    ```
    $ docker stop webserver
    ```

    執行 docker run 透過我們剛建立的 my-nginx-imag 來建立 container

    ```
    $ docker run --name nginx-container -d my-nginx-image
    ```

    此時在瀏覽器 http://localhost:7777 應該又可以看到 nginx 伺服器的首頁被換成我們 index.html 內容

# 總結
以上就是簡單的使用 Docker 建立 nginx 伺服器入門教學，事實上 docker 功能還有多有趣的應用，網路上也有許多已經打包好的 image 檔案等著發掘。對了，自從有了 docker 之後不但可以早點下班，搞壞環境也不用怕！

# 參考文件
1. [Docker 基本觀念與使用教學：自行建立 Docker 影像檔](https://blog.gtwang.org/virtualization/docker-basic-tutorial/)
2. [twtrubiks/docker-tutorial](https://github.com/twtrubiks/docker-tutorial)
3. [教你一次學會安裝 Docker 開始玩轉 Container 容器世界](https://blog.hellosanta.com.tw/%E7%B6%B2%E7%AB%99%E8%A8%AD%E8%A8%88/%E4%BC%BA%E6%9C%8D%E5%99%A8/%E6%95%99%E4%BD%A0%E4%B8%80%E6%AC%A1%E5%AD%B8%E6%9C%83%E5%AE%89%E8%A3%9D-docker-%E9%96%8B%E5%A7%8B%E7%8E%A9%E8%BD%89-container%C2%A0%E5%AE%B9%E5%99%A8%E4%B8%96%E7%95%8C)
4. [Docker —— 從入門到實踐](https://www.gitbook.com/book/philipzheng/docker_practice/details)


（image via [logz](https://logz.io/blog/what-is-docker/)）

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
