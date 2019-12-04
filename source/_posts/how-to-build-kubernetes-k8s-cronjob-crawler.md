---
title: 如何使用 K8S 自動化定期 CronJob 抓網路公開資料
date: 2019-06-29 10:23:23
author: kdchang
tags: 
    - Python
    - k8s
    - cronjob
    - kubernetes
    - crontab
---
![如何使用 K8S 自動化定期 CronJob 抓網路公開資料](/img/kdchang/kubernetes101/cronjob.png)

# 前言
有使用 Linux 的讀者就知道，若是有定期需要執行的程式就可以 Crontab 把寫好的 script 透過定期的 scheduler 定期執行節省人力。一般常見的使用範疇就是定期更新檔案資料或是網路爬蟲等。今天我們則是要介紹，如何使用 Kubernetes（k8s） 的 CronJob 來自動化抓取網路公開資料（這邊我們使用[政府公開資料的雨量資料 JSON 檔案](https://data.gov.tw/dataset/9177)），我們想要的定期執行程式的效果。好的，那就讓我們開始吧！

# 環境設定
若是對於 Kubernetes（k8s）比較不熟悉的讀者可以想成是 Kubernetes（k8s）是一個大型的 container 調度和管理工具，透過 config 設定可以管理你的 dockerize 後的 application。

在這篇文章中我們會使用 minikube 這個 local 開發測試用的 Kubenetes（k8s）cluster 當作測試 demo 使用。若你的電腦還沒有安裝 Kubernetes（k8s）的相關環境的話，可以先[參考官方網站的教學](https://kubernetes.io/docs/tasks/tools/install-minikube/)和[我們之前的教學文章](https://blog.techbridge.cc/2018/12/01/kubernetes101-introduction-tutorial/)。

這邊我們使用 macOS 當作範例，需要安裝的有 virtual box、kubernetes-cli 和 minikube 並登入好你的 [docker hub 帳戶](https://hub.docker.com/)：

確認一下若是你的 minikube 已經 start 成功，可以使用下列指令確認是否正常啟動：

```
$ kubectl cluster-info
```

另外也可以安裝 [kubectx 這個好用小工具](https://github.com/ahmetb/kubectx)，方便你切換到不同 cluster，這邊我們要切換到 minikube。

# 撰寫 CronJob 程式和 Dockerfile
因為範例為求簡單，這邊我們使用 Python 撰寫一個簡單每分鐘定期抓取政府公開資料的 python 程式，主要功能為：

1. 抓取網路公開資料
2. 根據時間儲存成 {datatime}.json 檔案到 /data 資料夾下

範例程式 `app.py`： 
```
import json
from datetime import datetime
import glob

import requests


def main():
    url = 'https://opendata.cwb.gov.tw/fileapi/v1/opendataapi/O-A0002-001?Authorization=rdec-key-123-45678-011121314&format=JSON'
    resp = requests.get(url)
    data = resp.json()

    with open('/data/{}.json'.format(datetime.utcnow()), 'w') as f:
        json.dump(data, f)

if __name__ == '__main__':
    main()
```

參考 `Dockerfile`：

```
FROM python:3.7-alpine
ADD . /code
WORKDIR /code
RUN pip install -r requirements.txt
CMD ["python", "app.py"]
```

若是完成後可以透過將我們的程式打包成 docker image 然後上傳到 docker hub 上，讓之後的 k8s cronjob 可以抓下來（當然若是私人專案或是公司專案可以考慮使用 [google cloud registry](https://cloud.google.com/container-registry/) 來 host 你的 image）。

打包 image 檔案（xxxx 為你的 docker hub id，k8s-cronjob-pvc-example 為 image 名稱，v1 為 tag 名稱）：

```
docker build -t xxxx/k8s-cronjob-pvc-example:v1
```

若是完成後可以使用 `$ docker image list` 觀看是否有正常顯示

接著就要推送到 docker hub 上面：

```
$ docker push xxxx/k8s-cronjob-pvc-example:v1
```

成功後應該就可以在你自己的 docker hub 上面看到上傳的 image 檔案！

# 撰寫 k8s CronJob config 檔案
上傳 image 到 docker hub 後，我們要開始將我們的程式 deploy 到 minikube 這個 local k8s cluster 上面！

首先我們先定義 schedule 格式為：`*/1 * * * *` （每分鐘執行一次）

cron 主要格式 `* * * * *` 就是由左到右分別為：

* 分鐘
* 小時
* 每月中第幾天
* 月
* 星期幾

若是你對於 cronjob 格式比較不熟悉，可以[參考這個網站](https://crontab.guru/)，他可以透過輸入你的設定值告訴目前格式的效果，十分方便！

以下是參考的 `cronjob.yaml` 檔案：

1. 我們設定 apiVersion 版本和 k8s config 類型 kind 為 CronJob
2. 從 kdchang/k8s-cronjob-pvc-example:v1 抓下來我們的程式
3. 透過 /bin/sh 指令列印出 ls /data 寫入檔案列表
4. 值得注意的是由於 k8s 資源利用的設計，每次 pod 重啟不一定會是在同一個 node 上部屬，另外隨著 pod 的重啟在 local 的檔案生命週期也會隨之消失。這對於定期產生的  pod 完成後就回收的 CronJob 來說會是一個問題：因為我們想要我們定期抓下來的網路資料可以持續存在。關於這個問題我們可以使用寫入資料庫或是宣告 k8s persistent volume 並掛載檔案路徑到 CronJob 中來解決。

以下我們宣告 hostPath volumes 並把 volume mount 到 /data 下面（hostPath 你可以想成若是 pod 重啟，會去找到上次的 pod 的檔案路徑和保留的檔案，當實務上會造成 pod 沒辦法有效 deploy 到適合的 node 上，此處因為使用 minikube 測試所以沒使用 GCP、AWS等 persistent volume 和外掛檔案系統）。

```
apiVersion: batch/v1beta1
kind: CronJob
metadata:
  name: k8s-cronjob-pvc-example
spec:
  schedule: "*/1 * * * *"
  jobTemplate:
    spec:
      template:
        spec:
          containers:
          - name: k8s-cronjob-pvc-example
            image: kdchang/k8s-cronjob-pvc-example:v1
            args:
            - /bin/sh
            - -c
            - ls /data
            - date; echo Hello from the Kubernetes cluster
            volumeMounts:
            - mountPath: /data
              name: crawl-data
          restartPolicy: OnFailure
          volumes:
          - name: crawl-data
            hostPath:
              # directory location on host
              path: /data
              # this field is optional
              type: Directory
```

接著執行我們的 `cronjob.yaml`

```
$ kubectl create -f cronjob.yaml
cronjob.batch/k8s-cronjob-pvc-example created
```

此時使用以下指令應該就會看到 cronjob 開始執行

```
$ kubectl get cronjob
NAME                      SCHEDULE      SUSPEND   ACTIVE   LAST SCHEDULE   AGE
k8s-cronjob-pvc-example   */1 * * * *   False     0        <none>          11s
```

一分鐘後看到 cronjob pod 成功開始執行！

```
$ kubectl get pod
NAME                                       READY   STATUS      RESTARTS   AGE
k8s-cronjob-pvc-example-1561892400-cbpc4   0/1     Completed   0          75s
k8s-cronjob-pvc-example-1561892460-t9ckq   0/1     Completed   0          15s
```

觀看 log
```
$ kubectl logs -f k8s-cronjob-pvc-example-1561892400-cbpc4
2019-06-29 10:27:57.140583.json
2019-06-29 10:27:57.492762.json
2019-06-29 10:27:57.737316.json
2019-06-29 10:27:58.996981.json
2019-06-29 10:27:59.125029.json
2019-06-29 10:27:59.140703.json
2019-06-29 10:28:12.885361.json
2019-06-29 10:28:12.942726.json
2019-06-29 10:28:15.883667.json
2019-06-29 10:28:40.844267.json
2019-06-29 10:28:43.823896.json
2019-06-29 10:28:44.787840.json
2019-06-29 10:28:47.097306.json
2019-06-29 10:28:48.122653.json
2019-06-29 10:29:02.764501.json
2019-06-29 10:29:21.785981.json
2019-06-29 10:29:29.779006.json
2019-06-29 10:29:35.822945.json
2019-06-29 10:30:10.770817.json
2019-06-29 10:30:53.822591.json
2019-06-29 10:31:09.812561.json
```

若你要移除的話可以使用以下指令：

```
$ kubectl delete cronjob k8s-cronjob-pvc-example
cronjob.batch "k8s-cronjob-pvc-example" deleted
```

# 總結
以上簡單介紹了如何使用 K8S 自動化定期 CronJob 抓網路公開資料。有許多開發者在從裸機 bare metal server 轉換到 Kubenetes（k8s) 的過程中常常會覺得 deubg 不太習慣，主要原因就是原本可以隨便 ssh 進去主機和抓取最新的資料並重啟機器的簡單粗暴方式變得麻煩，但若是能克服這一點的話，就能享受 dockerize 的可攜性和 k8s 的簡單擴展和部屬特性，更加專注在業務邏輯上。我們下回見囉，掰撲！

# 參考文件
1. [Running Automated Tasks with a CronJob](https://kubernetes.io/docs/tasks/job/automated-tasks-with-cron-jobs/)
2. [Kubernetes, Docker, and Cron](https://medium.com/jane-ai-engineering-blog/kubernetes-docker-and-cron-8e92e3b5640f)

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 