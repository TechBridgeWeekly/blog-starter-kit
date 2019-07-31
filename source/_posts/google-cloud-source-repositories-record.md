---
title: Google Cloud Source Repositories 使用紀錄
date: 2018-10-12 14:23:21
tags:
    - google cloud
---

# 前言
GCSR(Google Cloud Source Repositories) 是 Google 推出以 git 為基礎的程式碼代管服務。

原先我的 Side Project 也是自己建 git server 來放，後來因為硬體故障等等，自己維護需要花費不少時間和成本(懶!)，加上本身也是 Google 的愛好者，最後選擇 GCSR 來試試。

首先談談 GCSR 的缺點，基本上第 1 個缺點為 GCSR 必須依附於 Google Cloud Platform(GCP)。

> 也就是說你必須先了解基本的 GCP 使用方式

第 2 個缺點在於建立和拷貝 Repository 的動作必須透過 Google Cloud SDK。

> 也就是說你必須先安裝 Google Cloud SDK

第 3 個缺點在於 GCSR 免費版的限制

> 1. Repository 最多 5 位使用者，每增加 1 人，每月 1 美金
> 2. Repository 儲存空間最大 50 GB，每增加 1 GB，每月 0.10 美金
> 3. Repository 最大 50 GB 輸出，每增加 1 GB 輸出，每月 0.10 美金

缺點看完了，我們來看看優點吧。

個人認為 GCSR 最大的優點在於可以連結 Google Cloud Platform 搭配不同工具以進行擴充。其次在於速度及安全性有一定的保障，加上內建 CI 功能和 User Interface，優於現今一般的程式碼託管服務。

以上的優缺點若您都能接受，那就繼續往下看吧。

首先提供官網 [GCSR](https://cloud.google.com/source-repositories/) 的說明。若想直接參考使用方式請看 [Quick Start](https://cloud.google.com/source-repositories/docs/quickstart)。

# 1. 如何在 GCSR 上建立專案以及複製專案到本地端

1. **在 Google Cloud Platform 建立專案**

    建議新增獨立的 GCP 專案來放置 Repository 避免和其他的專案混淆。

2. **安裝 Google Cloud SDK**

    安裝 SDK 的目的就是用來新增，複製專案。注意安裝完成之後還需要進行初始化的動作，參考[這裡](https://cloud.google.com/sdk/docs/#windows)。

3. **安裝 git**

    Windows 系統官方推薦使用 [git-scm](https://git-scm.com/download/win)，Linux 系統就用 [command line](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) 安裝 git 吧

4. **建立 hello-world Repository(Google Cloud SDK)**

    `gcloud source repos create hello-world`

5. **拷貝 hello-world Repository 到 local**

    `gcloud source repos clone hello-world`

# 2. 加入 HelloWorld script 和 Deploy 檔案

1.**移動到本地端 hello-world 目錄並新增 main.py 檔案，將下方內容複製到 main.py**
 
``` py
!/usr/bin/env python
 
import webapp2
 
class MainHandler(webapp2.RequestHandler):
   def get(self):
      self.response.write('Hello World!')
 
app = webapp2.WSGIApplication([
   ('/', MainHandler)
], debug=True)
```
 
2.**新增 app.yaml 檔案，將下方內容複製到 app.yaml**
 
``` py
runtime: python27
api_version: 1
threadsafe: yes
 
handlers:
- url: .*
script: main.app
 
libraries:
- name: webapp2
version: "2.5.2"
```

3.**push 到遠端 hello-world Repository**
 
```
git add .
 
git commit -m "Add Hello World app to Cloud Source Repositories"
 
git push origin master
```

# 3. 部署到 App Engine

1.**輸入以下指令以部署到 App Engine**

`gcloud app deploy app.yaml`

2.**確認部署內容是否成功**

`gcloud app browse`

完成後應該會開啟瀏覽器並顯示 Hello World!

 
# 總結

經由上述的分享你就能進行基本的 GCSR 操作，但使用 GCSR 的優勢可不僅於此，你可以透過 App Engine 雲端部屬，整合 Cloud Build 自動編譯，使用 Cloud Debugger 進行除錯。