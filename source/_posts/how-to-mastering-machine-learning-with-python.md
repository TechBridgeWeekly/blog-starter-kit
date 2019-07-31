---
title: 如何使用 Python 學習機器學習（Machine Learning）
date: 2017-04-08 09:54:49
tags:
	- Python
	- Machine Learning
	- 機器學習
	- AI
	- Artificial Intelligence
	- NLP
	- Data Mining
	- 人工智慧
	- 監督式學習
	- Supervised learning
author: kdchang
---

![如何使用 Python 學習機器學習（Machine Learning）](/img/kdchang/machine_learning.jpg)

隨著資料科學（Data Science）技術的興起，[人工智慧（Artificial Intelligence）](https://en.wikipedia.org/wiki/Artificial_intelligence)、[機器學習（Machine Learning）](https://en.wikipedia.org/wiki/Machine_learning) 成為近幾年來電腦科學界十分熱門的研究領域，如今在實體和線上的學習機器學習的資源有很多，本文整理了一些好用學習資源希望幫助初學者能更容易使用 Python 入門機器學習的領域中，從零開始學習機器學習。若是對於資料科學不熟悉的讀者可以先參考[適用於初學者的資料科學影片](https://azure.microsoft.com/zh-tw/documentation/articles/machine-learning-data-science-for-beginners-the-5-questions-data-science-answers/) ，讓自己對於資料科學有初步的認識。

# 什麼是機器學習（Machine Learning）？
機器學習是一種資料科學的技術，協助電腦從現有的資料學習，以便預測未來的行為、結果和趨勢。根據學習的方式又可以分為需要解答的[監督式學習（Supervised learning）](https://en.wikipedia.org/wiki/Supervised_learning)、[非監督式學習（Unsupervised learning）](https://en.wikipedia.org/wiki/Unsupervised_learning)和[增強學習（Reinforcement learning）](https://en.wikipedia.org/wiki/Reinforcement_learning)（還有一種混合式的半監督式學習）等子類別。機器學習技術可以應用的範圍十分廣泛，總的來說機器學習可以解決以下幾種問題：

1. 分類問題：這是 A 類 或 B 類嗎？

2. 異常值判斷：這很奇怪嗎？

3. 預測性分析：有多少？

4. 分群問題：這是如何組織的？

5. 增強學習協助決策：我接下來該怎麼辦？

當我們蒐集到相關、精確、連貫、足夠資料就可以挑選合適的演算法進行模型的的建置。

# 為什麼選擇 Python？
在資料科學和機器學習領域最重要的兩大程式語言就是 Python 和 R，Python 簡潔易學、應用範圍廣（不限於數據分析）且學習曲線平緩，適合作為第一個入門的程式語言，透過 pandas、SciPy/NumPy、sckikit-learn、matplotlib 和 statsmodels 可以進行數據分析的工作，適合工程任務和需要和網路應用程式整合的專案。至於 R 由於是統計學家開發的程式語言，則是擅長於統計分析、圖表繪製，常用於學術研究領域，建議也要有一定程度的掌握。一般情況下 Python 和 R 並非互斥，而是互補，許多資料工程師、科學家往往是在 Python 和 R 兩個語言中轉換，小量模型驗證、統計分析和圖表繪製使用 R，當要撰寫演算法和資料庫、網路服務互動等情況時在移轉到 Python。為了降低學習成本，我們先使用 Python 進行介紹。

若對於 Python 和 R 比較，這邊有兩篇文章可以參考 [数据科学界华山论剑：R与Python巅峰对决](http://bi.dataguru.cn/article-7257-1.html)、[Which is better for data analysis: R or Python?](https://www.quora.com/Which-is-better-for-data-analysis-R-or-Python)。

# 如何開始入門機器學習？
事實上，資料科學是個跨領域學門，在學習如何使用 Python 進行機器學習過程中通常必須掌握以下知識：

- 機器學習演算法
- Python 程式語言和資料分析函式庫
- 線性代數/統計學等相關學門
- 專業領域的領域知識（Domain Knowledge）

為了掌握以上三大領域知識（我們先把焦點放在機器學習核心技法，暫時忽略資料科學中對於領域知識的掌握），具體來說我們可以有以下步驟可以參考：

1. 掌握基礎 Python 程式語言知識

	線上學習資源：

	- [Codecademy](https://www.codecademy.com/learn/python)
	- [DataCamp](https://www.datacamp.com/) （也可以學 R）
	- [Learn X in Y Minutes (X = Python)](https://learnxinyminutes.com/docs/python/)
	- [Learn Python the Hard Way](https://learnpythonthehardway.org/book/)

2. 了解基礎數學/統計學和機器學習基礎知識

	- [可汗學院線性代數](https://www.khanacademy.org/math/algebra)

	- [Intro to Descriptive Statistics](https://www.udacity.com/course/intro-to-descriptive-statistics--ud827)
	- [Intro to Inferential Statistics](https://www.udacity.com/course/intro-to-inferential-statistics--ud201)	

	- [Andrew Ng 機器學習課程](https://www.coursera.org/learn/machine-learning)
	- [Andrew Ng 機器學習筆記](http://www.holehouse.org/mlclass/)
	- [Carnegie Mellon University Machine Learning](http://www.cs.cmu.edu/~ninamf/courses/601sp15/lectures.shtml)

3. 知道如何使用 Python 科學計算函式庫和套件
	
	推薦安裝 [Anaconda](https://docs.continuum.io/anaconda/install)，支援跨平台多種版本 Python，預設將數據分析、科學計算的套件裝好，自帶 spyder 編輯器、Jupyter Notebook（IPython Notebook），可以提供一個網頁版介面，讓使用者可以透過瀏覽器進行 Julia、Python 或 R 程式的開發與維護。

	- numpy：科學分析，[Scipy Lecture Notes 教學文件](http://www.scipy-lectures.org/)
	- pandas：資料分析
	- matplotlib：會製圖瞟
	- scikit-learn：機器學習工具

4. 使用 scikit-learn 學習 Python 機器學習應用

	- [Machine Learning: Python 機器學習：使­用Pytho­n](https://www.gitbook.com/book/htygithub/machine-learning-python)

5. 運用 Python 實作機器學習演算法

	- 感知器
	- 決策樹
	- 線性迴歸
	- k-means 分群

6. 實作進階機器學習演算法

	- SVM
	- KNN
	- Random Forests
	- 降低維度
	- 驗證模型

7. 了解深度學習（Deep Learning）在 Python 的實作和應用
	
	- [NTU Applied Deep Learning](https://www.csie.ntu.edu.tw/~yvchen/f105-adl/index.html)
	- [Stanford Deep Learning](http://deeplearning.stanford.edu/tutorial/)
	- [深度學習(Deep Learning)自學素材推薦](https://dt42.github.io/2016/04/27/deep-learning-material-recommendations/)
	- [深度學習 Deep Learning：中文學習資源整理](http://www.jerrynest.com/deep-learning-resource/)

# 總結
以上整理了一些機器學習網路學習資源，若你累積一些小小經驗後，不妨挑戰一下 [Kaggle](https://www.kaggle.com/) 測試一下自己的實力並累積更多數據分析的經驗。

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 

# 延伸閱讀
1. [7 Steps to Mastering Machine Learning With Python](http://www.kdnuggets.com/2015/11/seven-steps-machine-learning-python.html)
2. [人人都可成為資料科學大師！一整年的網路自學清單就在這了](https://buzzorange.com/techorange/2016/02/02/plan-to-be-a-data-scientist-in-new-year/)
3. [Analytics Vidhya](https://www.analyticsvidhya.com/)
4. [台灣資料科學年會](http://datasci.tw/)
5. [「2016 台灣資料科學愛好者年會」精彩資料總整理(持續更新中)](http://dataology.blogspot.tw/)
6. [大數據會消失，資料科學不會！你該知道的資料科學第一堂課](http://www.bnext.com.tw/article/view/id/40220)
7. [如何選擇 Microsoft Azure Machine Learning 的演算法](https://azure.microsoft.com/zh-tw/documentation/articles/machine-learning-algorithm-choice/)
8. [Microsoft Azure Machine Learning 機器學習服務文件](https://azure.microsoft.com/zh-tw/documentation/services/machine-learning/)
9. [Kdnuggets](http://www.kdnuggets.com/)
10. [Bigdatafinance](http://www.bigdatafinance.tw/)
11. [Using Python and R together: 3 main approaches](http://www.kdnuggets.com/2015/12/using-python-r-together.html)
12. [机器学习最佳入门学习资源](http://blog.csdn.net/shadow_mi/article/details/51829389)
13. [机器学习(Machine Learning)&深度学习(Deep Learning)资料(Chapter 1)](https://github.com/ty4z2008/Qix/blob/master/dl.md)

（image via [respondr](http://respondr.io/wp-content/uploads/2016/03/machine_learning-1024x724.jpg)）
