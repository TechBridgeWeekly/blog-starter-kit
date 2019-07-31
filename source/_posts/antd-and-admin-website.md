---
title: 打造後台管理系統的好幫手：Ant Design
date: 2018-04-28 06:12:52
tags: 
    - antd
author: huli
---

## 前言

很多公司都會需要一個僅供內部使用的後台系統，來管理公司內的一些事情。不過也因為是公司內的產品，投入的資源比起給外面使用者的產品往往都來得較少，身為一個前端工程師，你很有可能必須身兼設計，決定整個 UI 的長相。

這時候呢，如果能找到一套現成的 Library 當然就最棒了！

這篇主要會介紹由螞蟻金服開源出來的 Ant Design 這套 UI 的 Library，並且讓大家看看幾個範例。

## 後台系統的 UI

在我認識 Ant Design 之前，我都是用這套叫做 [AdminLTE](https://adminlte.io/themes/AdminLTE/pages/forms/general.html)當作模板來刻 UI。找這種現成的模板好處顯而易見，那就是很多時候你只要一直複製貼上就好，可以省略大部份的 CSS，大概只有 20% 需要自己寫。

除此之外呢，也能在沒有設計師的幫助下就刻出風格一致的介面。

![](/img/huli/antd/lte.png)

這一套是專門給後台系統用的，如果你要找的只是 UI Library，或許 [Bootstrap](https://getbootstrap.com/) 跟 [Material-UI](https://www.material-ui.com/#/) 就很夠用了。

在開始想說要用哪一套 Library 之前，可以先問自己兩個問題：

> 1. 對於後台系統，我需要哪些元件？
> 2. 最主要的功能會是什麼？

第一點的話很容易，其實我們可以簡單歸納出以下幾種：

1. 基本 Layout 元件與網格系統
2. 基本元件，像 Button、Icon、List、Card 等等
3. 表單元件，像 Input、Form、Select、Checkbox 等等
4. 表格

第二點的話呢，以我自己接觸過的後台系統來說，我認為後台系統最需要的是以下兩點：

1. 需要支援複雜的表單，提供許多不同的 Component 像是 AutoComplete、DatePicker、TimePicker、Rating 等等
2. 需要有強大的 Data Table，可以排序、篩選、搜尋、分頁或甚至是 Inline Edit

如果你把上面兩點考慮進來的話，現成的 UI Library 大概都被排除了。不過其實也很合理，因為你會發現我們要的東西其實包含「功能面」，例如說 Data Table，這些都是純 UI 的 Library 不會去支援到的，你必須自己另外再找 Library 才能實現。

總結以上幾點，我理想中的 Library 大概長這樣：

1. 要支援功能面，不只有 UI
2. UI 要漂亮
3. 最好能有 React 元件（個人偏好）

有這樣的 Library 嗎？有，就是等等要介紹的這套。

## Ant Design

![](/img/huli/antd/hp.png)

Ant Design 是一套由中國知名網路公司螞蟻金服所開源出來的 UI Library，但它跟其他套不同的點在於它不只是一套 UI Library，還提供了一些設計語言（Design Language），簡單來講就是給了你一份指南，跟你說你遇到什麼情形的時候應該怎麼做，以及每個設計背後代表的意義。

除此之外呢，我覺得 Ant Design 的優點還有以下幾項：

1. 支援各種常用組件，連很不常見的都有
2. 有提供 React Component，使用起來很方便
3. Table 功能強大，支援分頁、篩選跟排序
4. 中英文文件齊全，要跟外國同事共事也沒問題
5. 由螞蟻金服維護，更新頻繁穩定度高

這邊有一個開源的 demo，就是基於 Ant Design 來開發的：[zuiidea/antd-admin](https://github.com/zuiidea/antd-admin)：

![](/img/huli/antd/admin.png)

我自己覺得介面很 OK，看起來也很舒服。最重要的是你會用到的 Component 它基本都實作了，很少時候需要自己手寫 CSS。

以下是它所有的 Component。

### 常用及排版

1. Button 按鈕
2. Icon 圖標
3. Grid 網格
4. Layout 佈局

### 導覽

1. Affix 固釘
2. Breadcrumb 麵包屑
3. Dropdown 下拉選單
4. Menu 選單
5. Pagination 分頁
6. Steps 步驟條

### Data Entry

1. AutoComplete 自動完成
2. Cascader 層級選擇
3. Checkbox 多選
4. DatePicker 日期選擇
5. Form 表單
6. Input 輸入
7. InputNumber
8. Mention 提及
9. Rate 評分
10. Radio 多選
11. Slider 滑動條
12. Switch 開關
13. TreeSelect 樹選擇
14. TimePicker 時間選擇
15. Transfer 穿梭框
16. Upload 上傳

### Data Display

1. Avatar 頭像
2. Badge 徽章
3. Calendar 日曆
4. Card 卡片
5. Carousel 跑馬燈
6. Collapse 折疊
7. List 列表
8.  Popover 氣泡卡片
9.  Tooltip 文字提示
10. Table 表格
11. Tabs 分頁
12. Tag 標籤
13. Timeline 時間軸
14. Tree 樹

### Feedback

1. Alert 警告提示
2. Modal 對話框
3. Message 全局提示
4. Notification 通知
5. Progress 進度條
6. Popconfirm 氣泡確認
7. Spin 載入中

### 其他

1. Anchor 錨點
2. BackTop 回到頂部
3. Divider 分隔線
4. LocaleProvider 國際化

提供了將近 50 種的組件任你使用！所以我才說 Ant Design 的涵蓋範圍真的很大，所有常用的 Component 都找得到，有些比較少見的也直接附在裡面了。

再讓大家看幾張截圖：

在更改/新增資料時可以跳出 Modal 取代掉換頁（如果內容不多的話）：
![](/img/huli/antd/modal.png)

結合其他 Component 做出一個額外的 Filter：
![](/img/huli/antd/filters.png)

基本表單以及表單驗證：
![](/img/huli/antd/form_validate.png)

## 實作上要注意的地方

首先呢，第一個是表單的挑選，因為後台系統裡面會有超級多表單，所以決定要用哪一個 Library 是很重要的。

Ant Design 有提供自己的 Form HOC，跟 Ant Design 其他組件的整合很不錯。講到表單不得不提的 Redux Form 則是自由度高了許多，可以跟任意的 UI Library 互相搭配使用。

基於自由度高以及功能完整這兩點，我們最後選擇了 Redux Form 來當作處理表單相關的 Library。

第二點要注意的是，可以在 Ant Design 跟真的 App 之間放一層中間層，我稱之為 Component Wrapper。

![](/img/huli/antd/wrapper.png)

為什麼要用這一層？不是多此一舉嗎？

這一層的目的是把 App 跟 UI Library 的相依性降低，舉個例子來說好了，假設現在你所有頁面都直接引用 antd 的元件，然後下一個版本中 antd 把某個 props 從 text 改成 value，你該怎麼辦呢？

還能怎麼辦，就是搜尋然後取代囉，一個個檢查有沒有沒改到的或是不該改的。

但假設我們中間多了一層 Wrapper，我們就能在這一層裡面自動把這個 props 換掉，省掉很多麻煩。

又或是原本 antd 的 Modal 並沒有支援 loading 這個狀態，只能搭配 Spin 自己實現，但如果我們有了中間這層，就可以在中間來實作，而不是在 App 的某幾個地方。

所以程式碼會長的像這樣，我們自己加了一個`isLoading`的 props：

![](/img/huli/antd/code.png)

我還滿喜歡這個 idea，把原本 antd 的元件都先包起來，這樣如果你想做什麼改變，可以在中間那層改，改一個地方就能夠影響到全 App。

## 結論

自從我們內部系統改用 Ant Design 來開發之後，效率上增快了很多，UI 也看起來舒服很多。身為一個被指派設計師任務的工程師，也覺得輕鬆許多。你只要用他們的 Component，就可以保證 UI 的一致性。

而且當你在 UI/UX 上有疑問時，還可以去翻他們的指南，有時候會寫說他們推薦應該要怎麼處理會比較好。

如果你要客製化的話也很方便，官方有提供一個 less 檔案，你只要把裡面你想換的變數換掉就好，或者是你自己 override css 也行，很自由。Ant Design 目前的更新頻率是每一週都會修一些 bug，每個月會有一些新的功能，我覺得更新得很頻繁修復的速度也很快，應該是有個 team 專門在負責這個。

總之呢，誠心跟大家推薦這套 Library，下次要開發後台系統時可以考慮看看。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好