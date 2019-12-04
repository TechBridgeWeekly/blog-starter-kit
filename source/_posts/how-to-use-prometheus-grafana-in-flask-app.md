---
title: 使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統
date: 2019-08-26 20:23:23
author: kdchang
tags: 
    - Python
    - Prometheus
    - Grafana
    - Flask
    - Alertmanager
    - docker-compose
    - slack
    - monitor
    - monitoring system
    - pagerduty
    - alert system
    - Cloud Native
    - tutorial
    - Grafana 教學
    - Prometheus 教學
---

![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/cover.png)

# 前言
身為一個開發者，當我們部屬一個 Web service 時，不是射後不理，而是需要透過監控預警系統去 monitor server 的運行狀況，並在緊急狀況即時通知相關人員作對應處理。所以透過好的 monitoring/alert system 了解目前 server 硬體系統使用狀況（CPU/Memory usage）和整個 service 的網路 networking 狀況是非常重要的一件事情。若是有經驗的讀者，可能過去曾經使用過 [Zabbix](https://www.zabbix.com/)、[Nagios](https://www.nagios.org/) 等工具來監控 service 的運行狀況，以便除錯和維持 service 的可用性（Availability）。

在眾多的 monitor 工具中，[Prometheus](https://prometheus.io/) 是一個很方便且整合完善的監控預警框架 TSDB（Time Series Database）時間序列資料庫，可以很容易建立不同維度的 metrics 和整合不同的 alert tool 以及資訊視覺化圖表的監控工具並提供自帶的 [PromQL (Prometheus Query Language)](https://prometheus.io/docs/prometheus/latest/querying/basics/) 進行 query 查詢。此外，源自於 [SoundCloud](http://soundcloud.com/) 的 Prometheus 目前是獨立於任何公司外的 open source project 並和 [Kubernetes](https://kubernetes.io/) 一樣是 [Cloud Native Computing Foundation（CNCF）](https://www.cncf.io/) 下的一員（目前已經孵化成熟畢業了），也有許多知名公司如：Uber、DigitalOcean 等導入企業專案，所以在使用上相對有保障。

今天我們就要透過 docker compose 搭配 flask 實作一個簡單 web service 範例，來整合 [Prometheus](https://prometheus.io/) 和 [Grafana](https://grafana.com/) 來建立一個 web service 監控預警系統。

# Prometheus 介紹

![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/prometheus-cover.png)

簡單來說 Prometheus 是一個監控預警框架和 TSDB（Time Series Database）時間序列資料庫，可以很容易建立不同維度（dimension）的 metrics 和整合不同的 alert tool 以及資訊視覺化的監控工具。透過 Prometheus 我們可以建立一站式的監控預警系統。Prometheus 可能在儲存擴展上比不上其他 Time Series Database，但在整合各種第三方的 data source 上十分方便（算是一個懶人包），且在支援雲端服務和 container 容器相關工具都十分友好。然而在圖表顯現上就稍嫌單薄，所以通常會搭配精美的儀表板工具 Grafana 等來進行資訊視覺化和圖表呈現。

![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/prometheus-architecture.png)

俗話說，一張圖勝過千言萬語：接下來我們用架構圖就可以更清楚了解 Prometheus 整體的架構和定位：

1. 有個 Prometheus server 本體，會去 Prometheus client pull 相關 metrics，若是短期的 job 例如 cronjob 在還來不及 pull 資料回來可能就已經完成任務，清洗掉資料。所以會有一個 pushgateway 接收 job push 過來的相關資訊，Prometheus server 再從其中拉取資料（就是 gateway 的感覺）
2. 上面部分則透過 Service discovery 的方式可以很好的蒐集 kubernetes 相關的資訊
3. Prometheus 本體會將資料儲存在 local on-disk time series database 或是可以串接 remote storage systems
4. Prometheus server 資料拉回來後可以提供本身自帶的 Web UI 或 Grafana 和其他 client 來呈現（透過使用 PromQL 進行查詢）
5. 當抓取資料的值超過 alert rule 所設定的閥值（threshold） 時， alert manager 就會將訊息送出（可以透過 Email、Slack、[pagerduty](https://www.pagerduty.com/) 等訊息通知），提醒相關人員注意

另外 Prometheus 更多強化模組礙於篇幅下次再討論：

1. Node exporter：蒐集作業系統（OS）和硬體（hardware）相關資料
2. cAdvisor：蒐集容器（container）相關資料

最後，我們要了解的是 Prometheus Client 函式庫支援了四種主要 Metric 的類型：

1. Counter: 累加的資料，重設值為 0。常用於 HTTP request 錯誤的出現次數或是 error exception 出現次數
2. Gauge: 屬於與時間無關的當下資料（可以增減），例如：CPU/Memory 使用量
3. Histogram: 主要使用在表示一段時間範圍內的資料蒐集，以長條圖呈現
4. Summary： 表示一段時間內的資料蒐集的總結

以上就是 Promethus 架構的一個概覽，相信讀者們對於 Promethus 已經有個初步認識，知道 Promethus 是一個監控預警框架和 TSDB（Time Series Database）時間序列資料庫，接下來我們介紹 Grafana 的部分。

# Grafana 介紹
![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/grafana-cover.png)

Grafana 是由 Grafana Lab 經營的一個非常精美的儀表板 dashboard 系統，可以整合各種不同的 datasource，例如：Promethus、[Elasticsearch](https://www.elastic.co/)、[MySQL](https://www.mysql.com/)、[PostgreSQL](https://www.postgresql.org/docs/10/plpgsql.html) 等。透過不同種 metric 呈現在 dashboard 上。在這裡我們主要聚焦在和 Promethus 和 Grafana 的整合上。


# 專題時間：透過 docker-compose 運行 Prometheus/Grafana 監控 Flask Web App
接著我們要透過 docker 和 docker-compose 啟動一個簡單 Python Flask Web Server 並使用 Prometheus 和 Grafana 當作其監控預警系統，最後我們會呈現的是一個 Prometheus 和 Grafana dashbaord 以及我們會使用 locust 直接模擬大量 request 去觸發預警系統送通知到 slack！（若是對於 docker/docker compose 或 locust 比較不熟悉的讀者可以參考筆者之前撰寫的相關文章：[Docker Compose 建置 Web service 起步走入門教學](https://blog.techbridge.cc/2018/09/07/docker-compose-tutorial-intro/) 和 [如何使用 Python 和 Locust 進行 Load testing 入門教學
](https://blog.techbridge.cc/2019/05/29/how-to-use-python-locust-to-do-load-testing/)）。

## 建立 Flask Web App
1. 透過 docker/docker-compose 建立 Web App

    我們這邊使用一個 flask web app 當作測試，也就是一個讓 prometheus_server pull 資料回去的 server。
    
    首先先安裝相關套件，建立 requirements.txt 檔案：
    
    ```
    flask
    prometheus_client
    locustio
    ```

    以下是 Dockerfile：
 
    ```
    FROM python:3.6-alpine

    ADD . /code
    WORKDIR /code
    RUN pip install -r requirements.txt

    CMD ["python", "app.py"]
    ```

    這邊使用一個簡單的 flask app 來當作測試（`app.py`），主要是提供一個 endpoint `/`，並使用 prometheus_client 中 counter metric，當有使用者打 endpoint，則累加紀錄一次。而 `/metrics` endpoint 則 export 了 flask app server 的資訊，提供 prometheus_server 來 pull 相關資料回去。（這裡為求簡化流程所以只使用 Counter，讀者可以自己嘗試新增 Gauge、Histogram、Summary 等 metric 類型）

    ```    
    import prometheus_client
    from prometheus_client import Counter
    from flask import Response, Flask, jsonify

    app = Flask(__name__)

    total_requests = Counter('request_count', 'Total webapp request count')

    @app.route('/metrics')
    def requests_count():
        total_requests.inc()
        return Response(prometheus_client.generate_latest(total_requests), mimetype='text/plain')


    @app.route('/')
    def index():
        total_requests.inc()
        return jsonify({
            'status': 'ok'
        })


    if __name__ == '__main__':
        app.run(host='0.0.0.0', port=5000)

    ```

    在 docker-compose.yaml 追加 flask app（透過 Dockerfile 啟動，port 在 5000）

    ```
    version: '3.7'
    services:
        web:
            build: .
            ports:
            - "5000:5000"
            volumes:
            - .:/code # 把當前資料夾 mount 掛載進去 container，這樣你可以直接在本地端專案資料夾改動檔案，container 裡面的檔案也會更動也不用重新 build image！
    ```

    接著在終端機透過 docker-compose up 就可以啟動 flask web app 在 localhost:5000 囉！

## 設定 Promethus

1. 在資料夾下建立設定檔案：prometheus.yaml

    ```
    # 設定 global 全域設定
    # scrape_interval 是多久抓取一次資料
    global:
        scrape_interval: 5s
        external_labels:
            monitor: 'demo-monitor'

    # scrape_configs 則是抓取來源，這邊先設定我們 prometheus 本體 server 和 flask api_monitor，docker-compose 會把 service 加入 default network 所以可以用 web:5000 找到 flask app web service
    scrape_configs:
        - job_name: 'prometheus'
        static_configs:
            - targets: ['localhost:9090']
        - job_name: 'api_monitor'
        scrape_interval: 5s
        static_configs:
            - targets: ['web:5000']
    ```

2. 透過 docker/docker-compose 安裝 Prometheus

    ```
    version: '3.7'

    volumes:
        prometheus_data: {}
        grafana_data: {}

    services:
      prometheus:
        image: prom/prometheus:v2.1.0
        volumes:
          - ./prometheus.yaml:/etc/prometheus/prometheus.yaml
        command:
          - '--config.file=/etc/prometheus/prometheus.yaml'
        ports:
          - '9090:9090'
    ```

在終端機使用 `$ docker-compose up` 啟動 Prometheus


3. 觀看 promethus web UI dashboard

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/prometheus-1.png)


4. 觀看 promethus metrics

    Prometheus metric 呈現格式：

    ```
    <metric name>{<label name>=<label value>, ...}
    ```

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/prometheus-2.png)


5. 觀看 flask app metrics

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/prometheus-3.png)

到這邊恭喜讀者已經完成 Promethus 的初始設定並擁有一個 Promethus Service 了，接下來我們要把資料串接到 Grafana 讓資料可以透過 dashboard 呈現。

## 設定 Grafana

1. 透過 docker-compose 安裝 Grafana
    ```
    grafana:
        image: grafana/grafana
        volumes:
            - grafana_data:/var/lib/grafana
        environment:
          - GF_SECURITY_ADMIN_PASSWORD=pass
        depends_on:
          - prometheus
        ports:
          - '3000:3000'
    ```

2. 串接 promethus datasource 到 grafana 上（登入帳號為 admin/pass）

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/grafana-1.png)

    首先我們先選擇 Add data source，可以看到有很多不同的資料來源可以串接：

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/grafana-2.png)

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/grafana-3.png)

    我們選擇串接 Promethus 為 datasource，並在串接 url 輸入 http://promethus:9090（透過 promethus docker compose 會幫我們找到對應 ip），就可以找到 Promethus 的 metrics 資料。

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/grafana-4.png)

    由於 Promethus 是使用 PromQL 讀取資料，所以一開始不熟悉的話可以先把預設的 dashboard 給 import 進來，在 dashboard 就可以看到預設 dashboard，可以選擇 dashboard 上的 edit 參考相關語法

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/grafana-5.png)

    回到就可以看到 Promethus 2.0 Stats 等圖表可以觀看：

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/grafana-6.png)

    但我們想看的是我們 Flask Web App 的 Counter 次數，所以我們可以點選左方選單的 + 號，手動新增圖表 dashbaord（選擇 Add Query），透過 Query 下拉式選單選擇 Promethus 然後 Metrics 下拉選 request -> request_count_toal 就可以看到 flask web app 被 request 次數。此時可以手動重新整理多次 flask web app 頁面就可以看到統計資料持續往上。接下來我們會使用 locust 直接模擬大量 request 去觸發預警系統送通知到 slack！

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/grafana-7.png)

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/grafana-8.png)

    更多圖表，讀者若有興趣可以繼續探索：

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/grafana-9.png)

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/grafana-10.png)


## 設定 Alert manager
1. 透過 docker-compose 安裝 Alert manager
    在 docker-compose 追加 alertmanager 相關設定，也記得也要把這段 `- ./alert_rules.yaml:/etc/prometheus/alert_rules.yaml` 放到 prometheus service 的 volumes 中
    ```
      alertmanager:
        image: prom/alertmanager
        ports:
          - 9093:9093
        volumes:
          - ./alertmanager.yaml/:/etc/alertmanager/alertmanager.yaml
        restart: always
        command:
          - '--config.file=/etc/alertmanager/alertmanager.yaml'
          - '--storage.path=/alertmanager'
    ```

    設定 slack 取得 YOUR_SLACK_WEBHOOK_URL

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/alert-1.png)

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/alert-2.png)

2.  新增 alert_rules.yaml 檔案於資料夾下，定義規則（request_count_total > 100 持續超過 10s 就準備發出 alert 送到 slack）：

    ```
    groups:
        - name: too_many_request_count_total
          rules:
          - alert: TooManyReq
            expr: request_count_total > 100
            for: 10s
            labels:
              user: test
            annotations:
              summary: "request_count_total is too over!"
              description: "{{ $labels.instance }} of job {{ $labels.job }} has over 100 for more than 1 sec."
              username: "@channel"
    ```

3. 新增設定 alertmanager.yaml 於資料夾下，指定訊息傳送方式（可以是送 email、slack 等方式，這邊使用 slack，記得先去 slack 開申請 install app 到對應 channel，然後取得 YOUR_SLACK_WEBHOOK_URL）

    ```
    global:
      resolve_timeout: 2h

    route:
      group_by: ['alertname']
      group_wait: 5s
      group_interval: 10s
      repeat_interval: 1h
      receiver: 'slack'

    receivers:
      - name: 'slack'
        slack_configs:
          - api_url: "YOUR_SLACK_WEBHOOK_URL"
            channel: "#alert-test"
            text: "Alert!"
            title: "{{.CommonAnnotations.summary}}"
    ```

    重啟 docker-compose 然後回到 Prometheus dashboard 可以看到 alert 規則已經設定完成：

    ![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/alert-3.png)


## 完整程式碼 docker-compose.yaml

```
version: '3.7'

volumes:
    prometheus_data: {}
    grafana_data: {}

services:
  prometheus:
    image: prom/prometheus:v2.1.0
    volumes:
      - ./prometheus.yaml:/etc/prometheus/prometheus.yaml
      - ./alert_rules.yaml:/etc/prometheus/alert_rules.yaml
    command:
      - '--config.file=/etc/prometheus/prometheus.yaml'
    ports:
      - '9090:9090'
  grafana:
    image: grafana/grafana
    volumes:
        - grafana_data:/var/lib/grafana
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=pass
    depends_on:
      - prometheus
    ports:
      - '3000:3000'
  web:
    build: .
    ports:
      - "5000:5000"
    volumes:
      - .:/code
    depends_on:
      - prometheus
  alertmanager:
    image: prom/alertmanager
    ports:
      - 9093:9093
    volumes:
      - ./alertmanager.yaml/:/etc/alertmanager/alertmanager.yaml
    restart: always
    command:
      - '--config.file=/etc/alertmanager/alertmanager.yaml'
      - '--storage.path=/alertmanager'
```

## Alert Manager 測試
這邊我們參考 [如何使用 Python 和 Locust 進行 Load testing 入門教學](https://blog.techbridge.cc/2019/05/29/how-to-use-python-locust-to-do-load-testing/) 來送出測試 request，讓 request count 快速增加達到 alert 的門檻。

撰寫 locustfile.py

```
from locust import HttpLocust, TaskSet, task


class WebsiteTasks(TaskSet):
    @task(1)
    def index(self):
        self.client.get('/')


class WebsiteUser(HttpLocust):
    task_set = WebsiteTasks
    min_wait = 5000
    max_wait = 15000

```

在終端機 terminal 值行以下指令，模擬不斷發出大量 request：

```
$ locust -f locustfile.py -H http://localhost:5000 --no-web -c 100 -r 10 -t 600s
```

看到數值已超過閥值：

![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/alert-4.png)

等待中：

![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/alert-5.png)

發送通知：

![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/alert-6.png)

可以看到 grafana 上 request_count metric 圖表不斷增加，promethus 的 alert 最後也響起，發送訊息到 slack 上！

![使用 Prometheus 和 Grafana 打造 Flask Web App 監控預警系統](/img/kdchang/cloud-native/alert-7.png)

# 總結
以上我們整合了 Promethus（負責蒐集資料和預警）和 Grafana（負責視覺化資料）成功為我們的 Python Flask App 打造了監控預警系統並發送訊息到 slack 上。未來當 service 發生問題時，就可以在第一時間提醒值班的工程師，準備上班囉！事實上，Promethus 和 Grafana 有蠻多更進階的主題值得持續探索，例如如何和 Kubernetes（k8s）的整合、硬體效能監控和評估、AIOps（人工智慧運維）等，未來有機會再和大家繼續分享！

# 參考文件
1. [prometheus installation](https://prometheus.io/docs/prometheus/2.3/installation/)
2. [Grafana Dashboard for Prometheus official Python client with Flask App metrics](https://blog.pilosus.org/posts/2019/06/01/grafana-dashboard-flask-app/)
3. [Using Prometheus in Grafana](https://grafana.com/docs/features/datasources/prometheus/)
4. [Flask application monitoring with Prometheus](https://burhan.io/flask-application-monitoring-with-prometheus/)

（image via [aptira](https://aptira.com/training/introduction-to-monitoring-with-prometheus-grafana/)）

# 關於作者
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
