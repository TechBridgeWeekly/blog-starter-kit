---
title: Python Flask + Keras 建置圖片分類 API
date: 2018-11-01 12:00:00
author: kdchang
tags: Python, Keras
---

![Python Flask + Keras 建置圖片分類 API](/img/kdchang/python-deep-learning101/td-deep-learning.jpg)

## 前言
深度學習是近年來電腦科學界火熱的話題，如何將深度學習模型整合到生活應用更是許多開發者想要嘗試的事情，本文透過 Python Flask 搭配 Keras 這個深度學習函式庫（backend 搭配 tensorflow），建立一個簡易版的預測圖片分類的 web api。那就讓我們開始吧。

## 專案初始化
首先我們先打開終端機，初始化我們的專案：

```
$ mkdir python-keras-image-predict-api
$ cd mkdir python-keras-image-predict-api
$ virtualenv venv
$ . venv/bin/activate
$ pip install keras tensorflow flask pillow
```

然後新增一個 main.py 檔案，建立一個簡單 flask api 雛形，一個 POST predict 圖片 api：

```
from flask import Flask, request, jsonify

@app.route('/predict', methods=['POST'])
def predict():
    # initialize the data dictionary that will be returned from the
    # view
    data = {'success': False}
    return jsonify(data)

# 當啟動 server 時先去預先 load model 每次 request 都要重新 load 造成效率低下且資源浪費
if __name__ == '__main__':
     app.run()
```

## 資料前處理
一般而言建立一個機器學習模型會有基本的幾個步驟：

1. 資料前處理
2. 訓練模型
3. 預測模型

首先我們先來建立資料前處理函式，在預測圖片前我們需要進行前處理，將輸入圖片轉為模型可以接受的格式：

```
def preprocess_image(image, target):
    # 將圖片轉為 RGB 模式方便 predict
    if image.mode != 'RGB':
        image = image.convert('RGB')

    # 將資料進行前處理轉成 model 可以使用的 input
    image = image.resize(target)
    image = img_to_array(image)
    image = np.expand_dims(image, axis=0)
    image = imagenet_utils.preprocess_input(image)

    return image
```

## 建立模型
緊接著我們來建立模型，這邊為了簡化過程，我們使用 keras 內建的 ResNet50 pre-trained 模型

```
def load_model():
    # load pre-trained 好的 Keras model，這邊使用 ResNet50 和 ImageNet 資料集（你也可以使用自己的 model）
    global model
    global graph
    model = ResNet50(weights='imagenet')
    # 初始化 tensorflow graph
    graph = tf.get_default_graph()
```

## 整合成 API
最後我們把我們的資料前處理和模型來整合進入我們的 API：

完整程式碼（main.py）：

```python
import io

# import the necessary packages
from keras.applications import ResNet50
from keras.preprocessing.image import img_to_array
from keras.applications import imagenet_utils
from PIL import Image
import numpy as np
from flask import Flask, request, jsonify
import tensorflow as tf

# initialize our Flask application and the Keras model
app = Flask(__name__)
model = None


def load_model():
    # load pre-trained 好的 Keras model，這邊使用 ResNet50 和 ImageNet 資料集（你也可以使用自己的 model）
    global model
    global graph
    model = ResNet50(weights='imagenet')
    # 初始化 tensorflow graph
    graph = tf.get_default_graph()


def preprocess_image(image, target):
    # 將圖片轉為 RGB 模式方便 predict
    if image.mode != 'RGB':
        image = image.convert('RGB')

    # 將資料進行前處理轉成 model 可以使用的 input
    image = image.resize(target)
    image = img_to_array(image)
    image = np.expand_dims(image, axis=0)
    image = imagenet_utils.preprocess_input(image)

    return image


@app.route('/predict', methods=['POST'])
def predict():
    # initialize the data dictionary that will be returned from the
    # view
    data = {'success': False}
    print('request')
    # ensure an image was properly uploaded to our endpoint
    if request.method == 'POST':
        if request.files.get('image'):
            # 從 flask request 中讀取圖片（byte str）
            image = request.files['image'].read()
            # 將圖片轉成 PIL 可以使用的格式
            image = Image.open(io.BytesIO(image))

            # 進行圖片前處理方便預測模型使用
            image = preprocess_image(image, target=(224, 224))

            # 原本初始化的 tensorflow graph 搭配 sesstion context，預測結果
            with graph.as_default():
                preds = model.predict(image)
                results = imagenet_utils.decode_predictions(preds)
            
            data['predictions'] = []

            # 將預測結果整理後回傳 json 檔案（分類和可能機率）
            for (_, label, prob) in results[0]:
                r = {'label': label, 'probability': float(prob)}
                data['predictions'].append(r)

            data['success'] = True

    return jsonify(data)

# 當啟動 server 時先去預先 load model 每次 request 都要重新 load 造成效率低下且資源浪費。記得等到 model 和 server 完整執行後再發 request
if __name__ == '__main__':
    print(('* Loading Keras model and Flask starting server...'
        'please wait until server has fully started'))
    load_model()
    app.run()
```

## 預測
終於來到了預測的階段，我們這邊使用一下可愛的狗狗圖片：

![Python Flask + Keras 建置圖片分類 API](/img/kdchang/python-deep-learning101/dog_1.jpg)

啟動 server 執行 Flask 測試
```
$ python main.py
```

打開另外一個終端機視窗：

```sh
curl -X POST -F image=@dog_1.jpg http://127.0.0.1:5000/predict
```

回傳結果，是 English_foxhound 英國獵狐犬！

```json
{
    "predictions": [
        {"label":"English_foxhound","probability":0.8246265649795532},
        {"label":"Saint_Bernard","probability":0.05787363648414612},
        {"label":"Walker_hound","probability":0.05374307930469513},
        {"label":"beagle","probability":0.029194286093115807},
        {"label":"Greater_Swiss_Mountain_dog","probability":0.009891272522509098}
    ],
    "success":true
}
```

## 總結
以上透過 Python Flask 搭配 Keras 這個深度學習函式庫（backend 搭配 tensorflow），我們建立一個簡易版的預測圖片分類的 web api，成功預測了狗狗圖片，是一隻英國獵狐犬！深度學習是目前進展非常快速的領域，歡迎讀者一起討論交流，我們下次見囉！


關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter, Software Engineer & Maker. JavaScript, Python & Arduino/Android lover.:)

（image via [semaphoreci](https://semaphoreci.com/community/tutorials/mocks-and-monkeypatching-in-python)）





