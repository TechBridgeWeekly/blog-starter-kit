---
title: 如何在 Windows 安裝 OpenPose 跟使用 Python API 來偵測人體姿態
date: 2019-01-18 21:01:50
tags: 
   - OpenPose
   - Robotics
   - Deep Learning
   - Computer Vision
author: pojenlai
---

## 前言

![img](https://github.com/CMU-Perceptual-Computing-Lab/openpose/raw/master/.github/Logo_main_black.png)

[OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) 是 Carnegie Mellon University（CMU）論文實作的開源函式庫，提供的功能主要就是可以偵測人體的各部位在什麼地方，例如可以在影像中找到臉、身體、手等等地方的特徵點。但他厲害的地方在於可以一次偵測很多人，看看下面這張圖就知道：

![img](https://github.com/CMU-Perceptual-Computing-Lab/openpose/raw/master/doc/media/dance_foot.gif)

這個 repository 仍然相當地 active，有兩個 CMU 的學生常常在 Github 上面解 issue。這次因為筆者需要在 Windows 上面安裝 OpenPose 並呼叫 Python API，所以跟開發者有不少互動，才想到乾脆來寫一篇教學，讓之後想要使用 OpenPose 的讀者可以省去好幾天的環境安裝 debug。

## OpenPose 演算法簡介

![img](https://www.learnopencv.com/wp-content/uploads/2018/05/openpose-body-architecture-1024x291.png)

上面是 OpenPose 的架構圖，輸入會是一張大小為 w x h 的彩色影像；接著會先經過 Stage 0 的 VGG-19 Network，得到輸入影像的 feature map；然後會在 Stage 1 經過一些 CNN 來得到身體各部位的 confidence map。Stage 1 分成兩個 branch，branch 1 可以產生某特定部位的 confidence map，例如左肩的 confidence map：

![img](https://www.learnopencv.com/wp-content/uploads/2018/05/confidence-left-shoulder.jpg)

branch 2 則是產生不同身體部位之間的關聯性，例如脖子跟左肩的關係：

![img](https://www.learnopencv.com/wp-content/uploads/2018/05/heatmap-left-shoulder.jpg)

最後結合這些結果就可以得到全身各部位大致在什麼地方，還有彼此相對位置的關係。以上的講法有點過於簡化，有興趣的讀者可以看看延伸閱讀 1 的原始論文。

## Windows 安裝步驟

0. 確認一下你的硬體跟作業系統符合 [OpenPose 要求](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md#requirements-and-dependencies)

1. Clone OpenPose 的原始碼

   因為我們要使用 Python API，所以不能只是下載已經編譯好的 library 文件，必須要從原始碼開始編譯。我推薦大家可以用 Github Desktop 來下載原始碼 & 管理。

2. 下載並安裝 CMake GUI

   可以上 [CMake 的網站](https://cmake.org/download/)下載，Windows 的話請下載 `cmake-X.X.X-win64-x64.msi`。

3. 安裝 Visual Studio 2015

   OpenPose 的官方要求是 Visual Studio 2015 Enterprise Update 3，但似乎也有人用 Visual Studio 2015 Community 安裝成功。筆者是用 Enterprise 版本安裝成功的。

4. 安裝 CUDA

   官方推薦的版本是 CUDA 8。這一步要在安裝完 Visual Studio 之後做，因為安裝過程會產生一些 Visual Studio 需要的檔案。

5. 安裝 cuDNN

   官方建議安裝 5.1 版本。安裝方法很簡單，只要把下載的壓縮檔內容解壓縮到 `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v8.0` 路徑就好。（cuDNN 壓縮檔裡面會有三個資料夾，可以對應到 `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v8.0` 裡面也有的資料夾，分別把檔案放到對應的資料夾就好）。

6. 開始設定 CMake 來準備編譯需要的檔案

   - 設定檔案路徑（在 clone 下來的路徑中自己創一個 build 資料夾，並設定成 `Where to build the binaries`）
   ![img](https://github.com/CMU-Perceptual-Computing-Lab/openpose/raw/master/doc/media/cmake_installation/im_1_windows.png)

   - 按下 Configure 按鈕（generator 記得選 Visual Studio 14 2015 Win64）
　 ![img](https://github.com/CMU-Perceptual-Computing-Lab/openpose/raw/master/doc/media/cmake_installation/im_2.png)
   ![img](https://github.com/CMU-Perceptual-Computing-Lab/openpose/raw/master/doc/media/cmake_installation/im_2_windows.png)
 
   - 等待 Configure Done
   ![img](https://github.com/CMU-Perceptual-Computing-Lab/openpose/raw/master/doc/media/cmake_installation/im_3.png)
 
   - 勾選 BUILD_PYTHON
   ![img](https://i.imgur.com/CfLZCd5.jpg)

   - 按下 Generate

8. 用 Visual Studio 2015 打開 `build/OpenPose.sln` 檔案
9. 切換到 Release Mode 並 Build Project

這一步很重要，因為只用 Debug Mode build 會讓後面的 Python API 啟動失敗。詳見 [issue 1026](https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/1026)。

## Python API 呼叫步驟

如果上面的步驟都做完，應該可以在 `openpose\build\python\openpose\Release` 看到 `openpose_python.cp36-win_amd64.pyd` library 文件。（OpenPose 原生是用 C++ 寫的，是用 pybind11 包成 Python 可以呼叫的 library）。

接著，我們準備要來跑 `openpose\build\examples\tutorial_api_python\1_body_from_image.py`，要記得把裡面的 library 搜尋路徑改成自己的：

我可以跑起來的範例如下：

```python
# Import Openpose (Windows/Ubuntu/OSX)
dir_path = os.path.dirname(os.path.realpath(__file__))
try:
# Windows Import
if platform == "win32":
# Change these variables to point to the correct folder (Release/x64 etc.)
sys.path.append(dir_path + '/../../python/openpose/Release');
os.environ['PATH'] = os.environ['PATH'] + ';' + dir_path + '/../../x64/Release;' + dir_path + '/../../bin;'
import openpose_python as op
```

這時，你就可以去 cmd.exe 執行，執行下列步驟

```
::切換到你自己的 openpose 路徑
cd openpose\build\examples\tutorial_api_python
python 1_body_from_image.py
```

然後跑出下面的結果：
![img](https://i.imgur.com/Jas1B8z.jpg)

## 總結

今天跟大家介紹了 CMU 的 OpenPose 要怎麼安裝，也稍微介紹了 Python API 要怎麼使用，理論上學完這篇教學的內容後，你就可以用 Python 呼叫 OpenPose 的 API 來實作自己想要的更高階功能了。希望對你的研究或 project 有幫助！

## 延伸閱讀

1. [Convolutional Pose Machines](https://arxiv.org/pdf/1602.00134.pdf)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
