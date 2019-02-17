---
title: 如何用 TensorFlow object detection API 的 Single Shot MultiBox Detector 來做 hand detection
date: 2019-02-16 18:40:50
tags: TensorFlow, Deep Learning, Computer Vision, Hand detection
author: pojenlai
---

## 前言

今天要來教大家怎麼用 TensorFlow 的 Object Detection API 來偵測人的手，因為筆者最近在使用 [這個 repo](https://github.com/victordibia/handtracking) 的 code
時，還是遇到了一些問題，需要自己再去找資料解決，所以決定基於前人的基礎之上再補充說明一些

相比起去年寫過的 [一起來玩 OSRF 的 TensorFlow Object Detector](https://blog.techbridge.cc/2018/02/24/osrf-tensorflow-object-detector/)，這一篇會有更多對於演算法和細節的敘述，讓大家之後也可以自己學習深入研究各個 model。

## TensorFlow Object Detection API 的使用流程

在一開始，先知道所有要做的步驟，會對於整個流程比較有概念：

![img](https://i.imgur.com/u8tOwWq.jpg)

假設，你今天是只想拿別人現成的結果來做 hand detection，那你可以不需要會上面這一串步驟，你只需要會最後一步 - Inference，拿到現成的 `frozen_inference_graph.pb`，再寫個程式直接使用這個`frozen_inference_graph.pb`來 inference 就好 （請參考 [這個範例](https://github.com/victordibia/handtracking/blob/master/detect_single_threaded.py)）。

但如果你對結果不滿意，想要再 finetune 現有的 model；或是你不想要只偵測手，你可能還想分辨是左手還是右手（也就是 model 的輸出類別要變），那你就會需要學會整個流程了，而這篇教學就希望可以達到這個目的。

* 我假設大家都已經有 Python、TensorFlow 這些東西了，沒有的話可以先去安裝一下。

## Dataset download & Image annotation

Dataset 的準備常常是在做 Deep Learning 研究最麻煩的一塊，如果要自己去準備，是可以拿一台相機就出去開始拍，或上網爬下一堆圖片，然後再一張張慢慢標註。（可以用 [labelImg](https://github.com/tzutalin/labelImg) 或 [labelme](https://github.com/wkentaro/labelme)）

Hand dataset 是有一些選擇，比較有名的包含 [University of Oxford 的 Hand Dataset](http://www.robots.ox.ac.uk/~vgg/data/hands/) 和 [Indiana Univeristy 的 EgoHands](http://vision.soic.indiana.edu/projects/egohands/)。

因為這些 dataset 都算足夠量的圖片加上 ground truth data，所以我們可以直接拿這些 dataset 來用，我們先假設我們是用 EgoHands。

但因為 TemsorFlow Object Detection API 需要吃的是 TF record 格式的檔案，所以我們還需要做的準備是：

1. 把 dataset 裡面的的格式轉成 csv
2. 把 csv 的內容轉成 TF record

之所以要先轉成 csv 是因為 csv 是相對方便我們看 ground truth data 有沒有問題的。

關於把 EgoHands 的 labels 轉成自己的 csv 檔，你可以參考 [egohands_dataset_clean.py](https://github.com/victordibia/handtracking/blob/master/egohands_dataset_clean.py)，其中最關鍵的地方就是這個 function，把 EgoHands 裡面原本存的 label 讀出來並寫到 csv 當中。

```python
def get_bbox_visualize(base_path, dir):
image_path_array = []
for root, dirs, filenames in os.walk(base_path + dir):
for f in filenames:
if(f.split(".")[1] == "jpg"):
img_path = base_path + dir + "/" + f
image_path_array.append(img_path)

#sort image_path_array to ensure its in the low to high order expected in polygon.mat
image_path_array.sort()
boxes = sio.loadmat(
base_path + dir + "/polygons.mat")
# there are 100 of these per folder in the egohands dataset
polygons = boxes["polygons"][0]
# first = polygons[0]
# print(len(first))
pointindex = 0

for first in polygons:
index = 0

font = cv2.FONT_HERSHEY_SIMPLEX

img_id = image_path_array[pointindex]
img = cv2.imread(img_id)

img_params = {}
img_params["width"] = np.size(img, 1)
img_params["height"] = np.size(img, 0)
head, tail = os.path.split(img_id)
img_params["filename"] = tail
img_params["path"] = os.path.abspath(img_id)
img_params["type"] = "train"
pointindex += 1

boxarray = []
csvholder = []
for pointlist in first:
pst = np.empty((0, 2), int)
max_x = max_y = min_x = min_y = height = width = 0

findex = 0
for point in pointlist:
if(len(point) == 2):
x = int(point[0])
y = int(point[1])

if(findex == 0):
min_x = x
min_y = y
findex += 1
max_x = x if (x > max_x) else max_x
min_x = x if (x < min_x) else min_x
max_y = y if (y > max_y) else max_y
min_y = y if (y < min_y) else min_y
# print(index, "====", len(point))
appeno = np.array([[x, y]])
pst = np.append(pst, appeno, axis=0)
cv2.putText(img, ".", (x, y), font, 0.7,
(255, 255, 255), 2, cv2.LINE_AA)

hold = {}
hold['minx'] = min_x
hold['miny'] = min_y
hold['maxx'] = max_x
hold['maxy'] = max_y
if (min_x > 0 and min_y > 0 and max_x > 0 and max_y > 0):
boxarray.append(hold)
labelrow = [tail,
np.size(img, 1), np.size(img, 0), "hand", min_x, min_y, max_x, max_y]
csvholder.append(labelrow)

cv2.polylines(img, [pst], True, (0, 255, 255), 1)
cv2.rectangle(img, (min_x, max_y),
(max_x, min_y), (0, 255, 0), 1)

csv_path = img_id.split(".")[0]
if not os.path.exists(csv_path + ".csv"):
cv2.putText(img, "DIR : " + dir + " - " + tail, (20, 50),
cv2.FONT_HERSHEY_SIMPLEX, 0.75, (77, 255, 9), 2)
cv2.imshow('Verifying annotation ', img)
save_csv(csv_path + ".csv", csvholder)
print("===== saving csv file for ", tail)
cv2.waitKey(2) # close window when a key press is detected
```

成功執行完，你就可以得到如下的 csv 檔（class 我有自己改過，如果是跑範例的程式碼，都會是 hand）：

![img](https://i.imgur.com/78BcNQE.jpg)

若你想要修改自己的 class，那就需要去改上面那段程式碼的這一行：

```python
labelrow = [tail,np.size(img, 1), np.size(img, 0), "hand", min_x, min_y, max_x, max_y]
```

把 "hand" 用其他方式取代。


## Label map preparation & TF Record generation

當你有了 csv 檔之後，接下來你還會需要轉成 TF record 檔，所以你會需要用 [generate_tfrecord.py](https://github.com/EdjeElectronics/TensorFlow-Object-Detection-API-Tutorial-Train-Multiple-Objects-Windows-10/blob/master/generate_tfrecord.py) 來產生你的 TF record 檔。

其中有一個關鍵的地方是，TF Record 吃的類別是數字，所以你需要自己去注意數字跟類別間的對應關係，而這個對應關係是由 hand_label_map.pbtxt 來描述：

```
item {
id: 1
name: 'hand'
}
```

所以你在用 產生 TF record 檔時，要很注意這一段 code：

```python
# TO-DO replace this with label map
def class_text_to_int(row_label):
if row_label == 'nine':
return 1
elif row_label == 'ten':
return 2
elif row_label == 'jack':
return 3
elif row_label == 'queen':
return 4
elif row_label == 'king':
return 5
elif row_label == 'ace':
return 6
else:
return None
```

在我們的應用中，你會改成：

```python
# TO-DO replace this with label map
def class_text_to_int(row_label):
if row_label == 'hand':
return 1
else:
return None
```

## Pipeline configuration

前面的步驟是為了產生 TF record 和 label map，最後只要把你的 config 檔設定好就好，因為我們沒有要重新訓練一個模型，而是拿人家已經用 Coco dataset pretrain 好的 SSD 來 finetune，所以可以參考[這個資料夾](https://github.com/victordibia/handtracking/tree/master/model-checkpoint/ssdmobilenetv1) 裡面的 [ssd_mobilenet_v1_coco.config](https://github.com/victordibia/handtracking/blob/master/model-checkpoint/ssdmobilenetv1/ssd_mobilenet_v1_coco.config)。

裡面比較常改的地方是

- Training step 數（一般我們都是訓練到再增加結果也不會變好）
```
num_steps: 200000
```

- 指定 training data 的 TF record 檔跟 label map：

```
train_input_reader: {
tf_record_input_reader {
input_path: "PATH_TO_DATA/train.record"
}
label_map_path: "PATH_TO_DATA/hand_label_map.pbtxt"
}
```

- 指定 training data 的 TF record 檔跟 label map：

```
eval_input_reader: {
tf_record_input_reader {
input_path: "PATH_TO_DATA/test.record"
}
label_map_path: "hand_inference_graph/hand_label_map.pbtxt"
shuffle: false
num_readers: 1
num_epochs: 1
}
```

以上就是要自己做 training 的所有事前準備工作。因為有很多東西是別人寫過的，我並沒有重複寫得很詳細，如果：

- 你用 Windows，請看 [How To Train an Object Detection Classifier for Multiple Objects Using TensorFlow (GPU) on Windows 10](https://github.com/EdjeElectronics/TensorFlow-Object-Detection-API-Tutorial-Train-Multiple-Objects-Windows-10#2-set-up-tensorflow-directory-and-anaconda-virtual-environment)
- 你用 Linux，請看 [Hand Detection Tutorial](https://github.com/jkjung-avt/hand-detection-tutorial)

## Training

如果上面的步驟都已經做完， training 就只是跑幾行指令而已：

```
cd C:\Users\rosindigo\Documents\GitHub\object_detection_training_env\models\research\object_detection

set PYTHONPATH=C:\Users\rosindigo\Documents\GitHub\object_detection_training_env\models;C:\Users\rosindigo\Documents\GitHub\object_detection_training_env\models\research;C:\Users\rosindigo\Documents\GitHub\object_detection_training_env\models\research\slim

python train.py --logtostderr --train_dir=training/ --pipeline_config_path=training/ssd_mobilenet_v1_coco.config
```

如果 train 完想要看一下結果，可以用 tensorboard 看一下：

```
tensorboard --logdir=training
```

如果結果滿意，就可以輸出 frozen graph：

python export_inference_graph.py --input_type image_tensor --pipeline_config_path training/ssd_mobilenet_v1_coco.config --trained_checkpoint_prefix training/model.ckpt --output_directory inference_graph

## 怎麼評估自己 finetune 完的 model 是否夠好？

在上面訓練的過程中，你可以看到每個 step 的 loss 有多少，一般來說我們會希望訓練到 2 以下，算是可以有還 OK 的辨識效果。

如果你想做更嚴謹的 evaluation，我跟大家推薦 [Object-Detection-Metrics](https://github.com/rafaelpadilla/Object-Detection-Metrics)，只要將 test data 的 ground truth 跟你的辨識結果都輸出到 txt 檔，就可以跑裡面提供的程式來畫出 Precision-Recall Curve，得到 AP 和 mAP，頗為方便。

## 總結

今天跟大家分享了要怎麼用 TensorFLow object detection API 來訓練和辨識手，希望透過學習流程，大家也可以將這個技術應用到自己有興趣的領域！

## 延伸閱讀

1. [How to Build a Real-time Hand-Detector using Neural Networks (SSD) on Tensorflow](https://medium.com/@victor.dibia/how-to-build-a-real-time-hand-detector-using-neural-networks-ssd-on-tensorflow-d6bac0e4b2ce)
2. [Hand Detection Tutorial](https://github.com/jkjung-avt/hand-detection-tutorial)
3. [SSD: Single Shot MultiBox Detector](https://arxiv.org/pdf/1512.02325.pdf)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
