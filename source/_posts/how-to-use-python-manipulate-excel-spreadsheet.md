---
title: 如何使用 Python 程式操作 Excel 試算表
date: 2018-10-05 20:23:23
author: kdchang
tags: 
    - Python
    - excel
    - 試算表
    - how-to
    - tutorial
    - spreadsheet
---

![如何使用 Python 程式操作 Excel 試算表](/img/kdchang/python-automation-programming/excel-intro/excel-cover.png)

# 前言
Excel 幾乎是所有職場工作者最常使用的 Office 軟體工具，小至同事間訂便當、飲料，大到進出貨訂單管理，應收應付賬款的財務報表等都有它的身影。在一般工作上，你可能常常需要在不同表單中複製貼上許多的欄位，或是從幾百個列表中挑選幾列依照某些條件來更新試算表內容等。事實上，這些工作很花時間，但實際上卻沒什麼技術含量。你是否曾想過但使用程式語言來加快你的工作效率，減輕瑣碎的重複性無聊工作但又不知道如何開始？

別擔心，這邊我們就要使用 Python 和 `Openyxl` 這個模組，讓讀者可以輕鬆使用 Python 來處理 Excel 試算表，解決工作上的繁瑣單調工作！

![如何使用 Python 程式操作 Excel 試算表](/img/kdchang/python-automation-programming/excel-intro/excel-intro.png)

# Excel 試算表名詞介紹
在正式開始使用 Python 程式來操作 Excel 試算表前我們先來了解 Excel 常見名詞。首先來談一下基本定義，一般而言 Excel 試算表文件稱作活頁簿（workbook），而活頁簿我們會存在 `.xlsx` 的副檔名檔案中（若是比較舊版的 Excel 有可能會有其他 `.xls` 等檔名）。在每個活頁簿可以有多個工作表（worksheet），一般就是我們工作填寫資料的區域，多個資料表使用 tab 來進行區隔，正在使用的資料表（active worksheet）稱為使用中工作表。每個工作表中直的是欄（column）從和橫的是列（row）。在指定的欄和列的區域是儲存格（cell），也就是我們輸入資料的地方。一格格儲存格的網格和內含的資料就組成一份工作表。

# 環境設定
在開始撰寫程式之前，我們先準備好開發環境（根據你的作業系統安裝 [Anaconda Python3](https://www.anaconda.com/download/)、[virtualenv 模組](https://virtualenv.pypa.io/en/stable/installation/)、[openyxl 模組](https://openpyxl.readthedocs.io/en/stable/)）。

![如何使用 Python 程式操作 Excel 試算表](/img/kdchang/python-automation-programming/excel-intro/demo1.png)

這邊我們使用 MacOS 環境搭配 `jupyter notebook` 做範例教學：

```
# 創建並移動到資料夾
$ mkdir pyexcel-example
$ cd pyexcel-example
$ jupyter notebook
```

開啟 jupyter notebook 後新增一個 Python3 Notebook

![如何使用 Python 程式操作 Excel 試算表](/img/kdchang/python-automation-programming/excel-intro/demo2.png)

首先先安裝 `openyxl` 套件（在 jupyter 使用 `$ !pip install <your-package>` 安裝套件）：

使用 `shift + enter` 可以執行指令

```
!pip install openpyxl
```

記得要先安裝 `openpyxl` 模組，若是沒安裝模組則會出現 `ModuleNotFoundError: No module named 'openpyxl'` 錯誤訊息。

![如何使用 Python 程式操作 Excel 試算表](/img/kdchang/python-automation-programming/excel-intro/demo3.png)

# 讀取 Excel 檔案
1. 使用 Openpyxl 開啟 Excel 檔案（可以從這邊[下載範例 Excel 資料檔案](http://go.microsoft.com/fwlink/?LinkID=521962)），下載後檔名改為 `sample.xlsx`，並放到和 jupyter Notebook 同樣位置的資料夾下：

    ```py
    from openpyxl import load_workbook

    wb = load_workbook('sample.xlsx')
    print(wb.sheetnames)
    ```

    執行後可以讀取活頁簿物件（類似讀取檔案）並印出這個範例檔案的工作表名稱：

    ```
    ['Sheet1']
    ```

2. 從工作表中取得儲存格（取得 A1 儲存格資料）

    ```
    ws['A1'].value
    ```

3. 從工作表中取得欄和列

    列出每一欄的值

    ```
    for row in ws.rows:
        for cell in row:
            print(cell.value)
    ```

    列出每一列的值

    ```
    for column in ws.columns:
        for cell in column:
            print(cell.value)
    ```

# 寫入 Excel 檔案
![如何使用 Python 程式操作 Excel 試算表](/img/kdchang/python-automation-programming/excel-intro/demo4.png)

1. 創建並儲存 Excel 檔案

    ```
    from openpyxl import Workbook

    # 創建一個空白活頁簿物件
    wb = Workbook()
    ```

2. 建立工作表

    ```
    # 選取正在工作中的表單
    ws = wb.active
    ```

3. 將值寫入儲存格內

    ```
    # 指定值給 A1 儲存格
    ws['A1'] = '我是儲存格'

    # 向下新增一列並連續插入值
    ws.append([1, 2, 3])
    ws.append([3, 2, 1])
    ```

4. 儲存檔案

    ```
    # 儲存成 create_sample.xlsx 檔案
    wb.save('create_sample.xlsx')
    ```

![如何使用 Python 程式操作 Excel 試算表](/img/kdchang/python-automation-programming/excel-intro/demo5.png)

# 總結
以上簡單介紹如何使用 Python 程式操作 Excel 試算表，透過 Python 可以讀取和寫入 Excel 檔案，相信只要能活用就能夠減少一般例行性的繁瑣工作。若需要更多 `openpyxl` 操作方式可以參考[官方文件教學](https://openpyxl.readthedocs.io/en/stable/index.html#)，我們下回見囉！

# 參考文件
1. [openpyxl - A Python library to read/write Excel 2010 xlsx/xlsm files](https://openpyxl.readthedocs.io/en/stable/index.html#module-openpyxl)

（image via [matplotlib](https://www.google.com.tw/url?sa=i&rct=j&q=&esrc=s&source=images&cd=&cad=rja&uact=8&ved=2ahUKEwjNwbGM4OHdAhXGvrwKHQ7DBKsQjxx6BAgBEAI&url=https%3A%2F%2Fproducts.office.com%2Fzh-hk%2Fexcel&psig=AOvVaw3QGOaPdJF8pWlME7Vtbe1r&ust=1538363320894206)


關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
