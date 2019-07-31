---
title: Docker Compose 建置 Web service 起步走入門教學
date: 2018-09-07 12:23:23
author: kdchang
tags: 
    - docker
    - 容器
    - docker machine
    - docker compose
    - docker swarm
    - vm
---

![Docker101](/img/kdchang/docker101/docker-compose-logo.png)

# 前言
身為一個開發者最惱人的莫過於環境建置和部屬應用程式。隨著 Docker 容器和虛擬化技術進步以及 DevOps、Infrastructure as Code 文化的推廣，讓我們可以更容易在不同環境開發部屬並調度（Orchestration）我們的專案應用程式。在 Docker 中，除了 Docker 指令和 Docker Engine 背後的技術外，最重要的莫過於 Docker Machine、Docker Compose 和 Docker Swarm 三劍客了。接下來我們將透過 Docker Compose 來啟動並執行 Python Flask + Redis 網頁人數統計的專案，讓讀者能夠理解 Docker Compose 的優勢和使用方式。那就讓我們開始吧！

![Docker101](/img/kdchang/docker101/docker-compose-services.png)

# Docker Compose 簡介
一開始我們先了解 Docker Compose 是一個工具可以讓你可以透過一個指令就可以控制所有專案（project）中所需要的 services。Docker Compose 是用 `YAML` 檔案格式來描述和定義 project 中 services 運作關係，白話來說就是用來管理 Container 的文件檔。

什麼意思呢？

試想一下，我們在開發一個典型的 Web project 時通常不是只有一個 service，有可能需要 app server、database、cache，甚至是 reverse proxy 等 service 才能構成一個可以上線運行的專案，這些 service 往往會需要多個 container 來運行。此時若是使用 Docker CLI 需要手動輸入多少行才能正式啟動一個 project？這時候就是 Docker Compose 發揮功能的時候啦！

我們先來看看，一個基本的 docker-compose.yml 檔案長這樣（YAML 檔案格式，使用空格來縮排，附檔名為 .yml）：

```yaml
version: '3' # 目前使用的版本，可以參考官網：
services: # services 關鍵字後面列出 web, redis 兩項專案中的服務
  web:
    build: . # Build 在同一資料夾的 Dockerfile（描述 Image 要組成的 yaml 檔案）成 container
    ports:
      - "5000:5000" # 外部露出開放的 port 對應到 docker container 的 port
    volumes:
      - .:/code # 要從本地資料夾 mount 掛載進去的資料
    links:
      - redis # 連結到 redis，讓兩個 container 可以互通網路
  redis:
    image: redis # 從 redis image build 出 container
```

# Dockerfile 和 Docker Compose 的差異是？
在了解到 Docker Compose 主要是用來描述 Service 之間的相依性和調度方式後，我們來看看同樣初學者會比較容易搞混的觀念：Dockerfile。事實上 Dockerfile 是用來描述映像檔（image）的文件。

所謂的 `Image`，就是生產 `Container` 的模版，你可以從 Docker Hub 官方下載或是根據官方的 Image 自己加工後打包成 Image 或是完全自己使用 Dockerfile 描述 Image 內容來製作 Image。而 Container 則是透過 Image 產生隔離的執行環境，稱之為 Container，也就是我們一般用來提供 microservice 的最小單位。

```dockerfile
# 這是一個創建 ubuntu 並安裝 nginx 的 image
FROM ubuntu:16.04 # 從 Docker hub 下載基礎的 image，可能是作業系統環境或是程式語言環境，這邊是 ubuntu 16.04
MAINTAINER demo@gmail.com # 維護者

RUN apt-get update # 執行 CMD 指令跑的指令，更新 apt 套件包資訊
RUN apt-get install –y nginx # 執行 CMD 指令跑的指令，安裝 nginx
CMD ["echo", "Nginx Image created"] 
```

以上為簡單的 Dockerfile。我們可以看到，只需一個文字檔，就清楚描述一個 Docker image。利於使用版本控制，也可以減少 shell script 的工作量。

# 透過 Docker 建立 Python Pageview App
在建立了 Dockerfile 和 Docker Compose 的基礎觀念後，我們來透過一個簡單 Python Flask + Redis 網頁人數統計的專案讓讀者可以更深刻理解 Docker Compose 的威力。

環境準備：
Lniux/MacOS 為主
可以輸入指令的終端機（若為 windows 可以使用 [cmder](http://cmder.net/)）
到官網安裝 [Docker](https://docs.docker.com/install/)
基本 Docker 和 Web 知識

1. 創建專案資料夾
```shell
$ mkdir docker-compose-python-flask-redis-counter
$ cd docker-compose-python-flask-redis-counter
```

2. 在資料夾下建立 app.py 當做 web app 進入點，裡面有 flask 和 redis 操作，當使用者瀏覽首頁時，redis 會記錄次數，若有 exception 則有 retry 機制
    ```py
    import time
    import redis
    from flask import Flask

    app = Flask(__name__)
    cache = redis.Redis(host='redis', port=6379)

    def get_hit_count():
        retries = 5
        while True:
            try:
                return cache.incr('hits')
            except redis.exceptions.ConnectionError as exc:
                if retries == 0:
                    raise exc
                retries -= 1
                time.sleep(0.5)


    @app.route('/')
    def get_index():
        count = get_hit_count()
        return 'Yo! 你是第 {} 次瀏覽\n'.format(count)

    if __name__ == "__main__":
        app.run(host="0.0.0.0", debug=True)
    ```

3. 建立套件 requirements.txt 安裝資訊讓 Dockerfile 可以下指令安裝套件

    ```
    flask
    redis
    ```

4. 建立 Web App 的 Dockerfile

    ```dockerfile
    FROM python:3.4-alpine # 從 python3.4 基礎上加工
    ADD . /code # 將本地端程式碼複製到 container 裡面 ./code 資料夾
    WORKDIR /code # container 裡面的工作目錄
    RUN pip install -r requirements.txt
    CMD ["python", "app.py"]
    ```

5. 用 Docker Compose file 描述 services 運作狀況，我們的專案共有 web 和 redis 兩個 service

    ```yaml
    version: '3'
    services:
    web:
        build: .
        ports:
        - "5000:5000"
        volumes:
        - .:/code # 把當前資料夾 mount 掛載進去 container，這樣你可以直接在本地端專案資料夾改動檔案，container 裡面的檔案也會更動也不用重新 build image！
    redis:
        image: "redis:alpine" # 從 Docker Hub registry 來的 image
    ```

6. 用 Docker Compose 執行你的 Web app（-d detached 是在背景執行，可以使用 $ docker ps -a 觀看目前所有 docker container 狀況，使用 `$ docker-compose ps` 觀看 docker-compose process 狀況）

    ```shell
    $ docker-compose up -d
    ```

    若要終止並移除 container 則可以使用 `$ docker-compose down`

7. 到 http://127.0.0.1:5000/ 觀看成果

![Docker101](/img/kdchang/docker101/docker-compose-flask-redis-demo.png)

# 總結
以上透過 Docker Compose 來啟動並執行 Python Flask + Redis 網頁人數統計的專案，讓讀者能夠理解 Docker Compose 的優勢和使用方式（Docker Compose 是一個工具可以讓你可以透過一個指令就可以控制所有專案（project）中所需要的 services）。同時也複習了 Dockerfile、Docker Image、Container 相關知識。Ya，自從有了 Docker Compose 在本地開發測試專案更加方便，考試都考一百分了！

# 參考文件
1. [Get started with Docker Compose](https://docs.docker.com/compose/gettingstarted/#step-8-experiment-with-some-other-commands)
2. [深入淺出 Dockerfile 與 Docker Compose](https://oomusou.io/docker/dockerfile-dockercompose/)
3. [部署Docker Compose　實例示範定義檔撰寫](http://www.netadmin.com.tw/article_content.aspx?sn=1712060002)
4. [Docker Compose 初步閱讀與學習記錄](http://blog.maxkit.com.tw/2017/03/docker-compose.html)

（image via [cuelogic](https://medium.com/skillshare-team/from-docker-compose-to-minikube-d94cbe97acda)、[Zabbix](https://share.zabbix.com/virtualization/docker/docker-compose-yml-v2-format-for-zabbix-3-0)）

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:)
