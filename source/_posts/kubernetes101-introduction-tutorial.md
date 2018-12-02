---
title: Kubernetes 與 minikube 入門教學
date: 2018-12-01 10:23:23
author: kdchang
tags: 
    - Kubernetes
    - container
    - 容器
    - minikube
    - 雲端
    - cloud native
---

![Kubernetes 與 minikube 入門教學](/img/kdchang/kubernetes101/kubernetes-logo.png) 

## 前言
[Kubernetes](https://kubernetes.io/)（又稱 K8s，類似於 i18n/l10n，取中間字母的長度的簡寫命名）是一個協助我們自動化部署（automating deployment）、自動擴展（scaling）和管理容器應用程式（containerized applications）的指揮調度（Orchestration）工具。相比於傳統的手動部屬容器應用程式的方式，Kubernetes 主要有幾個好處：

1. Automated rollouts and rollbacks
Kubernetes 可以很容易更新容器版本並 rollback 回之前版本

2. Service Scalability
當定義好部屬服務需求，可以很容易因應流量進行 Auto Scaling

3. Service discovery and load balancing
自動分配容器相對應的 IP 位址，透過 Service cluster 達到 load balancing 效果

4. Self-healing
當 Container Application 出現 crash 時，可以根據部屬定義的需求，自動偵測重啟服務

![Kubernetes 與 minikube 入門教學](/img/kdchang/kubernetes101/kubernetes-architecture.jpg) 

## Kubernetes 架構簡介
Kubernetes cluster 主要可以分為 Master 和 Node 兩部份，Master 負責指揮調度 Node。在 Kubernetes 世界裡，Node 上的 Pod 是運行調度的最小單位，裡面可以放多個 container（一般以有緊密相關的服務為主，同一個 Pod 共享 IP），也可以只有單個 Container，同一個 Pod 的 container 是一起被調度。而 Deployment 為管理 Pod 的 Controller，我們可以視一組 Deployment 為一組應用服務。而 Service 可以固定住我們對外服務的 IP，不會因為 Pod 關閉重啟而喪失原來的 IP 位置。

## 環境設定

>Minikube is a tool that makes it easy to run Kubernetes locally. Minikube runs a single-node Kubernetes cluster inside a VM on your laptop for users looking to try out Kubernetes or develop with it day-to-day

在開始學習跑步之前，我們先學會走路。首先，我們透過 [minikube](https://github.com/kubernetes/minikube) 這個可以在本地端跑 Kubernetes 工作，來在本地端部屬我們的 Kubernetes cluster，感受一下 k8s 的應用。由於 minikube 只提供 signle-node Kubernetes Cluster，本身並不支援 HA (High availability)，所以不推薦在實際應用上運行呦。

在開始操作之前我們先準備好：
1. [virtualbox](https://www.virtualbox.org/)（可以根據對應作業系統下載對應版本），
2. 接著透過 homebrew 下指令來安裝 minikube
（我們這邊會以 macOS 為主，若是其他作業系統可以參考 [minikube](https://github.com/kubernetes/minikube) 官方 repo 教學）

```
$ brew cask install minikube
```

安裝好後可以透過以下指令確認版本，同時它也會一起安裝 kubectl 這個 Kubernetes 指令操作工具 kubectl

```
$ minikube version
minikube version: v0.30.0
```

## 創建 cluster
接著啟動我們 Kubernetes cluster

```
$ minikube start

Starting local Kubernetes v1.10.0 cluster...
Starting VM...
Downloading Minikube ISO
 170.78 MB / 170.78 MB [============================================] 100.00% 0s
Getting VM IP address...
Moving files into cluster...
Downloading kubelet v1.10.0
Downloading kubeadm v1.10.0
Finished Downloading kubelet v1.10.0
Finished Downloading kubeadm v1.10.0
Setting up certs...
Connecting to cluster...
Setting up kubeconfig...
Starting cluster components...
Kubectl is now configured to use the cluster.
Loading cached images from config file.
```

若是啟動完成可以透過以下指令觀看 cluster 情況：

```
$ kubectl cluster-info
```

查看 minikube 的狀態：

```
$ minikube status
minikube: Running
cluster: Running
kubectl: Correctly Configured: pointing to minikube-vm at 192.168.99.100
```

還可以透過圖形化介面來觀看和操作 Kubernetes

```
$ minikube dashboard
```

![Kubernetes 與 minikube 入門教學](/img/kdchang/kubernetes101/kubernetes-dashboard.png) 

## 部屬應用
接著我們來部屬一個簡單範例應用到 Kubernetes。這個應用名稱叫 [docker-python-flask-demo](https://hub.docker.com/r/kdchang/docker-python-flask-demo/tags/)，是一個 Dockerize Simple Flask App 。kubectl run 可以讓我們啟動我們的 Pod，--image 後面接的是 docker image 位置和版本，--port 則是 container 對外的 port

```
$ kubectl run docker-python-flask-demo --image=docker.io/kdchang/docker-python-flask-demo:v1 --port 3000
```

應該可以看到 docker-python-flask-demo-xxxxxxxx 的 Pod 已經啟動：

```
$ kubectl get pods
```

## 對外公開應用
由於 default 情況下 Pod 只允許 cluster 內部訪問，若是要讓外部可以訪問的話可以將容器的 port 對應到 Node 的 Port（Cluster IP 是只能內部訪問，kubectl expose 則是把 deployment expose 成為一個對外 service，type NodePort 可以把 Deployment 透過 Kubernetes Cluster 的 port 讓 Cluster 外部可以訪問）
```
$ kubectl expose deployment/docker-python-flask-demo --type="NodePort" --port 3000
service/docker-python-flask-demo exposed
```

看一下 service 列表，發現已經隨機分配一個 port 31862 （注意讀者產生的 port 有可能不一樣）

```
$ kubectl get services
NAME                       TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)          AGE
docker-python-flask-demo   NodePort    10.100.20.174    <none>        3000:32743/TCP   2s
```

```
$ kubectl service docker-python-flask-demo --url
http://192.168.99.100:32743
```

可以看到 service 將 deployment/docker-python-flask-demo 的 port number 3000 與 minikube-vm 上的 port number 32743 做 mapping
接著可以使用 minukube service 的指令快速找到 docker-python-flask-demo 的 url，這樣我們就可以在瀏覽器輸入網址看到成果囉：

![Kubernetes 與 minikube 入門教學](/img/kdchang/kubernetes101/docker-flask-demo-v1.png) 

## 擴充應用
由於 default 情況下 deployment 只會有一個副本 replication，我們可以透過以下指令查看副本數：

```
$ kubectl get deployments
NAME                       DESIRED   CURRENT   UP-TO-DATE   AVAILABLE   AGE
docker-python-flask-demo   1         1         1            1           23m
```

設定副本數量為 3：

```
$ kubectl scale deployments/docker-python-flask-demo --replicas=3
deployment.extensions/docker-python-flask-demo scaled
```

我們可以看到原本的副本數量從 1 變成了 3：

```
$ kubectl get deployments
NAME                       DESIRED   CURRENT   UP-TO-DATE   AVAILABLE   AGE
docker-python-flask-demo   3         3         3            3           25m
```

設定副本數量為 2：

```
$ kubectl get deployments
NAME                       DESIRED   CURRENT   UP-TO-DATE   AVAILABLE   AGE
docker-python-flask-demo   2         2         2            2           26m
```

## 更新應用
若是我們想要更新 container 的版本的話可以下以下指令（也就是說原本 docker image tag v1 版本改進到 v2 版本，我們可以透過更新 docker image 來進行進版）：

```
$ kubectl set image deployments/docker-python-flask-demo docker-python-flask-demo=docker.io/kdchang/docker-python-flask-demo:v2
deployment.extensions/docker-python-flask-demo image updated
```

此時我們在到瀏覽器重新整理 http://192.168.99.100:32743/，就會發現畫面中的 Flask Dockerized v1 變成了 Flask Dockerized v2！

![Kubernetes 與 minikube 入門教學](/img/kdchang/kubernetes101/docker-flask-demo-v2.png) 

若要回到 v1 版本可以透過 rollout undo 指令來進行：

```
$ kubectl rollout undo deployments/docker-python-flask-demo
deployment.extensions/docker-python-flask-demo
```

又回到 v1 惹！

![Kubernetes 與 minikube 入門教學](/img/kdchang/kubernetes101/docker-flask-demo-v1.png) 


## 總結
以上簡單透過 minikube 介紹 Kubernetes 的架構和部屬 cluster 和應用在本地端，實際上我們可以透過雲端服務來部屬我們的 Kubernetes 應用，minikube 主要是用在練習和教學使用，不建議使用在生產環境上。目前 Kubernetes 生態系發展非常快速，版本也持續更新。關於 Kubernetes 相關的實際應用，我們下回再繼續討論囉！


關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
