---
title: 使用 Python 資料分析和視覺化上市櫃公司薪資公開資料
date: 2019-07-26 20:23:23
author: kdchang
tags: 
    - Python
    - Data Analytics
    - Data Visualization
    - make a fortune
    - pandas
    - 上市櫃薪資資料
    - 公開資訊觀測站
    - 台灣薪資
    - 台灣薪水
    - salary
    - python 資料分析
    - python 資料視覺化
---

![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/salary-wordcloud.png)

# 前言
> 證交所今（28）日在公開資訊觀測站公布上市公司申報之107年度「非擔任主管職務之全時員工薪資」資訊。藉由提升員工薪酬資訊透明度，讓摳門老闆無所遁形。

根據新聞媒體的報導 [證交所公布上市公司非主管全時員工薪資](https://tw.news.yahoo.com/%E8%AD%89%E4%BA%A4%E6%89%80%E5%85%AC%E5%B8%83%E4%B8%8A%E5%B8%82%E5%85%AC%E5%8F%B8%E9%9D%9E%E4%B8%BB%E7%AE%A1%E5%85%A8%E6%99%82%E5%93%A1%E5%B7%A5%E8%96%AA%E8%B3%87-145450200.html)，讓我們可以一窺近兩千家（853 + 733）上市櫃公司的薪資水準，也可以看看哪些公司薪資水準相對較低（順便看看各種逗趣的理由？本文 cover 圖即是將低薪公司的理由放入 [wordcloud 產生器](https://wordcloud.timdream.org/)產生的圖片！），當作我們尋找合適工作的參考依據之一（但薪資高低不是決定合適工作的唯一標準）。

當然身為一個略懂略懂資料分析的軟體工程師，我們當然不能只看新聞媒體所提供的加工過的二手資料，更應該親自動手 hands on 來進行資料探索看看有哪些有趣的小 insight。因此接下來我們將使用 Python 網路爬蟲爬取證交所公開資訊觀測站上的上市櫃公司「非擔任主管職務之全時員工薪資」資訊並使用 python pandas、matplotlib 和 jupyter notebook 進行資料分析和資訊視覺化，看到更多有趣的薪水數據分析。

# 資料蒐集 Data Collect / Data Scraping
![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo1.png)

`正所謂巧婦難為無米之炊，資料分析難為沒 data！` 所以在決定要進行上市櫃公司的薪資資料後，我們必須蒐集相關的資料以利後續分析。

一開始我們先來到 [證交所公開資訊觀測站](https://mops.twse.com.tw/mops/web/t100sb15) 來觀察如何爬取非擔任主管職務之全時員工薪資資訊。

![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo2.png)

經過一番觀察後發現當下拉選單選擇時會使用 ajax 去後端 api 擷取資料。所以我們可以透過 POST `https://mops.twse.com.tw/mops/web/ajax_t100sb15` 這個 endpoint 去取得薪資資料。

![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo3.png)

![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo4.png)

由於資料集的欄位有合併的部分，所以我們拿掉 `非擔任主管職務之全時員工資訊`、`同業公司資訊` 和 `薪資統計情形` 讓 columns 可以更單純，以利於接下來使用 pandas 進行資料分析。

```
from collections import OrderedDict

import requests
import pandas as pd
from bs4 import BeautifulSoup


class HTMLTableParser:

    def get_html_tables_from_resp(self, html_text):
        soup = BeautifulSoup(html_text, 'html.parser')
        tables = soup.find_all('table')
        return tables

    def parse_html_table(self, table):
        """
        <tr>
            <th align="center" class="tblHead" nowrap="" rowspan="2">產業類別</th>
            <th align="center" class="tblHead" nowrap="" rowspan="2">公司代號</th>
            <th align="center" class="tblHead" nowrap="" rowspan="2">公司名稱</th>
            <th align="center" class="tblHead" colspan="4" nowrap="">非擔任主管職務之<br/>全時員工資訊</th>
            <th align="center" class="tblHead" colspan="2" nowrap="">同業公司資訊</th>
            <th align="center" class="tblHead" colspan="4" nowrap="">薪資統計情形</th>
        </tr>
        <tr>
            <th align="center" class="tblHead" nowrap="">員工薪資總額(仟元)</th>
            <th align="center" class="tblHead" nowrap="">員工人數-加權平均(人)</th>
            <th align="center" class="tblHead" nowrap="">員工薪資-平均數(仟元/人)</th>
            <th align="center" class="tblHead" nowrap="">每股盈餘(元/股)</th>
            <th align="center" class="tblHead" nowrap="">員工薪資-平均數(仟元/人)</th>
            <th align="center" class="tblHead" nowrap="">平均每股盈餘(元/股)</th>
            <th align="center" class="tblHead" nowrap="">非經理人之<br/>全時員工薪資<br/>平均數未達50萬元</th>
            <th align="center" class="tblHead" nowrap="">公司EPS獲利表現較同業為佳<br/>，惟非經理人之全時員工<br/>薪資平均數低於同業水準</th>
            <th align="center" class="tblHead" nowrap="">公司EPS較前一年度成長<br/>，惟非經理人之全時員工<br/>薪資平均數較前一年度減少</th>
            <th align="center" class="tblHead" nowrap="">公司經營績效與員工薪酬<br/>之關聯性及合理性說明</th>
        </tr>
        <tr>
            <td nowrap="" style="text-align:left !important;">資訊服務業</td>
            <td nowrap="" style="text-align:left !important;">8416</td>
            <td nowrap="" style="text-align:left !important;">實威</td>
            <td nowrap="" style="text-align:right !important;"> 158,636 </td>
            <td nowrap="" style="text-align:right !important;"> 186 </td>
            <td nowrap="" style="text-align:right !important;"> 853 </td>
            <td nowrap="" style="text-align:right !important;"> 9.69 </td>
            <td nowrap="" style="text-align:right !important;"> 807 </td>
            <td nowrap="" style="text-align:right !important;"> 1.20 </td>
            <td nowrap="" style="text-align:right !important;"></td>
            <td nowrap="" style="text-align:right !important;"></td>
            <td nowrap="" style="text-align:right !important;"></td>
            <td nowrap="" style="text-align:left !important;"><br/></td>
        </tr>
        """
        parsed_data = []

        # Find number of rows and columns
        # we also find the column titles if we can
        table_row_tags = table.find_all('tr')
        table_header_tags = table.find_all('th')
        column_names = [table_header_tag.get_text() for key, table_header_tag in enumerate(table_header_tags) if key not in (3, 4, 5)]
        column_names[7] = '同業公司{}'.format(column_names[7])
        column_names[8] = '同業公司{}'.format(column_names[8])

        tr_td_tags = [
            [td_tag.get_text().strip() for td_tag in table_row.find_all('td')]
            for table_row in table_row_tags if table_row.find_all('td')
        ]

        parsed_data = [
            OrderedDict({
                column_names[index]: td_tag
                for index, td_tag in enumerate(tr_td_tag)
            })
            for tr_td_tag in tr_td_tags
        ]

        df = pd.DataFrame.from_dict(parsed_data)

        return df


htlm_parser = HTMLTableParser()

payload = {
    # 'encodeURIComponent': 1,
    'step': 1,
    'firstin': 1,
    'TYPEK': 'sii', # sii 上市 / otc 上櫃
    'RYEAR': 107,
    'code': '',
}

headers = {'User-Agent': 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_11_6) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/61.0.3163.100 Safari/537.36'}

resp = requests.post('https://mops.twse.com.tw/mops/web/ajax_t100sb15', data=payload, headers=headers, timeout=2)

html_tables = htlm_parser.get_html_tables_from_resp(resp.text)

df_table = htlm_parser.parse_html_table(html_tables[0])

df_table.to_csv('107_{}.csv'.format(payload['TYPEK']), index=False, encoding='utf-8')

print(df_table)
```

# 資料前處理 Data Preprocessing
資料前處理是資料探勘和資料分析流程步驟之一，主要目的是將真實世界的資料進行整理轉化，變得可以閱讀和分析。

在蒐集完資料後，我們將使用 pandas 載入我們抓取的[上市櫃公司薪資的 .csv 檔案來進行分析](https://pse.is/GKXXU) 並進行資料前處理。我們蠻幸運的是這個資料集算是完整且是結構化資料，不太需要做太多的資料前處理（例如：補值、刪除遺漏值等），主要要做的是 columns 名稱調整和員工薪資值的取代和轉換成整數。

上市公司的 107_sii.csv 檔案：

![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/spreadsheet-1.png)

![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/spreadsheet-3.png)

上櫃公司的 107_otc.csv 檔案：

![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/spreadsheet-4.png)

![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/spreadsheet-6.png)

接著我們來進行資料的前處理，方便接下來的探索性資料分析，首先載入 pandas、matplotlib 函式庫並使用 pandas 載入資料集

![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo5.png)

```
import pandas as pd
import matplotlib.pyplot as plt

df_sii = pd.read_csv('./107_sii.csv')
df_otc = pd.read_csv('./107_otc.csv')

df_sii.shape
# (853, 13) 共有 853 家上市公司資料，13 個欄位
df_otc.shape
# (733, 13) 共有 733 家上市公司資料，13 個欄位
df_sii.index
# RangeIndex(start=0, stop=853, step=1)
df_sii.values
"""
array([['水泥工業', 1101, '台泥', ..., nan, nan, nan],
       ['水泥工業', 1102, '亞泥', ..., nan, nan, nan],
       ['水泥工業', 1103, '嘉泥', ..., nan, nan, nan],
       ...,
       ['建材營造', 9946, '三發地產', ..., nan, nan, nan],
       ['其他', 9955, '佳龍', ..., nan, nan, nan],
       ['鋼鐵工業', 9958, '世紀鋼構', ..., nan, nan, nan]], dtype=object)
"""
df_sii.columns
"""
Index(['產業類別', '公司代號', '公司名稱', '員工薪資總額(仟元)', '員工人數-加權平均(人)', '員工薪資-平均數(仟元/人)',
       '每股盈餘(元/股)', '同業公司員工薪資-平均數(仟元/人)', '同業公司平均每股盈餘(元/股)',
       '非經理人之全時員工薪資平均數未達50萬元', '公司EPS獲利表現較同業為佳，惟非經理人之全時員工薪資平均數低於同業水準',
       '公司EPS較前一年度成長，惟非經理人之全時員工薪資平均數較前一年度減少', '公司經營績效與員工薪酬之關聯性及合理性說明'],
      dtype='object')
""
```

整體資料資訊：

```
df_sii.info()

<class 'pandas.core.frame.DataFrame'>
RangeIndex: 853 entries, 0 to 852
Data columns (total 13 columns):
產業類別                                    853 non-null object
公司代號                                    853 non-null int64
公司名稱                                    853 non-null object
員工薪資總額(仟元)                              853 non-null object
員工人數-加權平均(人)                            853 non-null object
員工薪資-平均數(仟元/人)                          853 non-null object
每股盈餘(元/股)                               853 non-null float64
同業公司員工薪資-平均數(仟元/人)                      853 non-null object
同業公司平均每股盈餘(元/股)                         853 non-null float64
非經理人之全時員工薪資平均數未達50萬元                    66 non-null object
公司EPS獲利表現較同業為佳，惟非經理人之全時員工薪資平均數低於同業水準    142 non-null object
公司EPS較前一年度成長，惟非經理人之全時員工薪資平均數較前一年度減少     0 non-null float64
公司經營績效與員工薪酬之關聯性及合理性說明                   207 non-null object
dtypes: float64(3), int64(1), object(9)
memory usage: 86.7+ KB
```

觀看前面幾筆資料長相：

```
$ df_sii.head()
```

![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo6.png)

將 columns 名稱調整成英文，方便分析操作

```
headers = ['industry','company_code', 'company_people_count', 'company_name', 'company_total_salary', 'company_average_salary', 'company_eps', 'industry_average_salary', 'industry_average_eps', 'is_under_50', 'high_eps_low_salary', 'growth_but_low_salary', 'low_salary_reason']
df_sii.columns = headers
df_otc.columns = headers

df_sii.columns

Index(['industry', 'company_code', 'company_name', 'company_people_count',
       'company_total_salary', 'company_average_salary', 'company_eps',
       'industry_average_salary', 'industry_average_eps', 'is_under_50',
       'high_eps_low_salary', 'growth_but_low_salary', 'low_salary_reason'],
      dtype='object')
```

調整 column 過後：

```
df_sii.info()
<class 'pandas.core.frame.DataFrame'>
RangeIndex: 853 entries, 0 to 852
Data columns (total 13 columns):
industry                   853 non-null object
company_code               853 non-null int64
company_name               853 non-null object
company_total_salary       853 non-null object
company_people_count       853 non-null object
company_average_salary     853 non-null int64
company_eps                853 non-null float64
industry_average_salary    853 non-null object
industry_average_eps       853 non-null float64
is_under_50                66 non-null object
high_eps_low_salary        142 non-null object
growth_but_low_salary      0 non-null float64
low_salary_reason          207 non-null object
dtypes: float64(3), int64(2), object(8)
memory usage: 86.7+ KB
```

將員工薪資-平均數(仟元/人) , 去除並將字串 str 轉為 float
```
df_sii['company_average_salary'] = df_sii['company_average_salary'].str.replace(',', '').astype(int)
df_otc['company_average_salary'] = df_otc['company_average_salary'].str.replace(',', '').astype(int)
```

新的資料長相：

```
df_sii.head()
```

![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo7.png)

# 探索性資料分析 Explore Data Analytics
探索性資料分析是資料分析的一個步驟，透過敘述統計、圖表分析等方式來了解資料（例如：最大、最小值、平均值、標準差、離群值等），找出假設和可能的特徵值。

在進行完資料前處理後我們要來進行探索性資料分析，看看有哪些有趣的資料組合、相關性和圖表。

1. 首先我們來看看一些基本的敘述統計：
    ```
    df_sii.describe()
    ```

    ![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo8.png)

    喔喔，853 家上是公司平均薪資達 82 萬，挺不錯的呀！但記得這是平均數（首富跟你的平均）。EPS 平均有 2.73 左右。來看看薪資排名：

    ```
    df_sii.sort_values(['company_average_salary'], ascending=False)
    ```

    最大值當然就是媒體爭相報導的發哥聯發科的 270 萬，郭董事長的鴻海也是榜上有名啦，當然我們台灣之光台積電也是名列前茅（讓人肅然起敬的還是台積電員工數量也是相當多）：

    ![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo9.png)

    最小值則是食品公司興泰的 34 萬（嗯，月薪約 NT 28,500，比 22k 好一點）

    ```
    df_sii.sort_values(['company_average_salary'], ascending=True)
    ```

    ![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo10.png)

    去[台灣股市資訊網](https://goodinfo.tw/StockInfo/StockList.asp?MARKET_CAT=%E7%86%B1%E9%96%80%E6%8E%92%E8%A1%8C&INDUSTRY_CAT=%E5%85%AC%E5%8F%B8%E7%B8%BD%E5%B8%82%E5%80%BC%E6%9C%80%E9%AB%98%40%40%E5%85%AC%E5%8F%B8%E7%B8%BD%E5%B8%82%E5%80%BC%40%40%E5%85%AC%E5%8F%B8%E7%B8%BD%E5%B8%82%E5%80%BC%E6%9C%80%E9%AB%98&SHEET=%E5%85%AC%E5%8F%B8%E5%9F%BA%E6%9C%AC%E8%B3%87%E6%96%99) 弄來的台灣前 20 大市值公司名單：

    ```
    top20_valuable_stock_list = [2330, 2317, 6505, 2412, 1301, 3008, 1303, 1326, 2882, 2454, 1216, 2881, 2886, 2891, 2308, 2002, 3045, 2912, 3711, 2892]
    df_sii[df_sii['company_code'].isin(top20_valuable_stock_list)]
    ```

    嗯，看來公司市值比較高，薪水不一定比較高。
    
    台股前二十大市值公司薪資排行：`聯發科、鴻海、台積電、中鋼、中華電、台達電、兆豐金、台塑石化、台塑、台化` <= 抓到了，公司市值大薪資又高的公司！讀者準備 CV 中（大誤）

    ![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo11.png)

    最後來關心一下，咱們網路軟體業的薪資水平好了。若以純網路軟體可以看一下目前獨立出來的上市櫃公司的電子商務類別：

    ```
    df_otc[df_otc['industry'].isin(['電子商務'])].sort_values(['company_average_salary'], ascending=False)
    ```

    好吧，革命尚未成功，同志仍需努力。台灣半導體、電子業還是很強，網路軟體業還是得加點油。

    ![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo12.png)

    ```
    df_otc_ec = df_otc[df_otc['industry'].isin(['電子商務'])]
    df_otc_ec.company_average_salary = pd.to_numeric(df_otc_ec.company_average_salary)
    df_otc_ec.loc[:, ['company_code', 'company_average_salary']].plot(kind='bar', x='company_code', y='company_average_salary', title ="TW EC", figsize=(15, 10), legend=True, fontsize=12)
    ```
    ![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo13.png)


2. EPS 和 薪資的相關性：
    接著我們想了解公司賺錢是否有合理分配到員工身上，因為依照正常的邏輯假設來講，公司賺錢（這邊簡單用 EPS 每股盈餘來看，稅後 EPS = 淨利/在外流通股數），應該員工的薪資會相對較高，但真的是這樣嗎？

    在畫散佈圖之前，我們先把 dataFrame 取子集合（僅含公司平均薪資和公司 eps）

    ```
    df_sii_average_salary_eps = df_sii[['company_average_salary', 'company_eps']]
    ```

    sort 發現有 大立光、國巨、華新科 index [488, 252, 372] 這幾個 eps 超高的 outliner，簡單起見先把它們去除：

    ```
    df_sii_average_salary_eps.sort_values(['company_eps'], ascending=False)
    ```

    ![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo14.png)

    ```
    filter_df_sii_average_salary_eps = df_sii_average_salary_eps.drop(df_sii_average_salary_eps.index[[488, 252, 372]])
    filter_df_sii_average_salary_eps.sort_values(['company_eps'], ascending=False)
    ```

    看起來正常多了：

    ![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo15.png)

    將 company_eps 當作 x 軸，company_average_salary 當 y 軸畫出 scatter，看起來大部分公司 eps 都在 0-10 之間，但 eps 和薪資相關性看起來沒辦法很確定是否相關：

    ```
    filter_df_sii_average_salary_eps.plot(kind='scatter', x='company_eps', y='company_average_salary')
    ```

    ![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo16.png)

    使用 [Pearson correlation coefficient](https://zh.wikipedia.org/zh-tw/%E7%9A%AE%E5%B0%94%E9%80%8A%E7%A7%AF%E7%9F%A9%E7%9B%B8%E5%85%B3%E7%B3%BB%E6%95%B0) 來看看 eps 和薪資的相關程度：

    > 皮爾森相關分析用於探討兩連續變數 (X, Y) 之間的線性相關性，若兩變數之間的相關係數絕對值較大，則表示彼此相互共變的程度較大。一般而言，若兩變數之間為正相關，則當 X 提升時，Y 也會隨之提升；反之，若兩變數之間為負相關，則當 X 提升時，Y 也會隨之下降。一般研究學者認為，相關係數 0.3 以下為低相關，0.3-0.7 為中等相關，0.7 以上為高度相關。

    ```
    filter_df_sii_average_salary_eps.corr(method ='pearson')
    ```

    ![使用 Python 資料分析和視覺化上市櫃公司薪水公開資料](/img/kdchang/python-data-analytics/demo17.png)

    結果是 0.307071。所以台灣上市公司 eps 和公司薪資應該可以算是中等相關，也就是說公司賺錢和你賺不賺錢不一定高度相關，還要看老闆產業和老闆摳不摳門呀！


3. 公司經營績效與員工薪酬之關聯性及合理性說明：
    這次證交所公布的公開資料蠻有趣的，特別要求若是屬於低薪的公司需要公布低薪理由。若有興趣的讀者可以研究看看，可以拓展視野，開了不少眼界XD，也更加印證了：

    `高薪的理由只有一個（產業、公司賺錢發大財），低薪的理由卻可以有很多種。`

    > 本公司地處偏鄉，人才要求不易致外籍員工人數較高拉低平均薪資，且對個人的學經歷及技能要求無法提高，僅能於就職過程中予以訓練及培養，雖無法達到規定金額仍盡力提高員工所得。

    `為何外籍員工只能低薪？可以雇用高級技術員工？`

    > 1.全球經濟不景氣2.過度教育讓學歷貶值、同儕效應使薪資過低3.外勞人數多，拉低平均薪資4.派遣員工的比例高，拉低對員工平均薪資的成長

    `過度教育讓學歷貶值？全球經濟不景氣？（千錯萬錯都是別人的錯？黑人問號？）`

    > 因總公司座落在「新竹科學園區」外，相較薪資水準與園區內的同業較低。且公司大部份員工是在地居民，所以薪資福利項目屬於外縣市之交通、住宿補助福利較少。

    `新竹科學園區 OS：低薪怪我囉～`


# 總結
以上我們透過簡單使用 Python 資料分析和視覺化上市櫃公司薪水公開資料，進行資料分析的步驟：

1. 資料蒐集 Data Collect / Data Scraping
2. 資料前處理 Data Preprocessing
3. 探索性資料分析 Explore Data Analytics

主要是透過一個難得的公開資料集進行簡單的資料分析（政府部門德政應該好好的鼓勵一下，也期許自己未來有機會也不要成為員工口中的慣老闆？）。未來我們也可以更進一步整合更多資料集來進行選股、公司低薪理由文字分析或是分析各區域的薪資地圖等更多有趣的資料分析應用。我們下回見啦！

# 參考文件
1. [證交所公布本國上市公司107年度非擔任主管職務之全時員工薪資資訊](https://www.twse.com.tw/zh/news/newsDetail/ff8080816b543823016b9cf0c75d013c)
2. [台灣公司總市值](https://goodinfo.tw/StockInfo/StockList.asp?MARKET_CAT=%E7%86%B1%E9%96%80%E6%8E%92%E8%A1%8C&INDUSTRY_CAT=%E5%85%AC%E5%8F%B8%E7%B8%BD%E5%B8%82%E5%80%BC%E6%9C%80%E9%AB%98%40%40%E5%85%AC%E5%8F%B8%E7%B8%BD%E5%B8%82%E5%80%BC%40%40%E5%85%AC%E5%8F%B8%E7%B8%BD%E5%B8%82%E5%80%BC%E6%9C%80%E9%AB%98&SHEET=%E5%85%AC%E5%8F%B8%E5%9F%BA%E6%9C%AC%E8%B3%87%E6%96%99)
3. [皮爾森積差相關分析(Pearson Correlation)-說明與SPSS操作](https://www.yongxi-stat.com/pearson-correlation/)

（image via [twimg](https://pbs.twimg.com/media/Dsa6TcAVAAEUcTw.jpg)、[topjavatutorial](http://www.topjavatutorial.com/wp-content/uploads/2016/04/LRU-Cache.png)、[Hacker Noon
](https://cdn-images-1.medium.com/max/2600/1*fvlMpkpIKmPm6IF_QnmjmQ.jpeg)）

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
