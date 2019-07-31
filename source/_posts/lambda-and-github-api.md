---
title: AWS Lambda + GitHub API + Google Sheet = 自動化簽到系統
date: 2018-09-14 20:10:30
tags:
  - lambda
  - aws
author: huli
---

## 前言

這一年間我利用閒暇之餘嘗試進行了幾場[教學實驗](https://medium.com/hulis-blog/mentor-program-s2-f876c4e13d3b)，希望能透過持續的教學改進自己的教材，從學生的反饋當中得到一些心得。

而在進行這些教學實驗的時候，我常常在想可以透過哪些現成的服務減少我的負擔，畢竟身為工程師嘛，很想把一些瑣碎的事務自動化，長期下來可以節省的時間是很可觀的。

半年前有了第一次嘗試，也在這邊分享了心得：[利用 Github Classroom 加 Travis CI 打造改作業系統](https://blog.techbridge.cc/2018/02/03/github-classroom-and-travis-ci/)，有了自動改作業系統以後，確實省下不少麻煩。

這次要來分享的是兩週前用大概一兩天的時間快速實作出來的自動化簽到系統。

## 為什麼要簽到系統？

為了掌握學生的進度以及確保學生是真的有心要持續上課，我在上一次教學實驗時就引進了我在公司裡面每天必做的 Daily Stand-up meeting，每個人快速分享一下自己昨天做了什麼、今天要做什麼以及有沒有任何事情阻止你完成這些任務。

那時候我用的方法是讓每個人在 Slack 的 channel 裡面用固定格式 po 這些東西：

![](/img/huli/lambda/daily.png)

優點是集中在一個地方，很方便觀看，而缺點就是我只能用肉眼看而已，沒辦法記錄下來。意思就是說如果我想要製作一個有哪些學生沒有 po 的表單，我只能一個一個自己填。

上一次時因為學生人數少，而且 po 這個也不是強制的，所以不成問題，但在這一次的教學實驗裡面我引入了淘汰制，在一定期間內如果缺交進度報告太多次是會被淘汰退出課程的。

既然有了這個機制，我就必須要把這些進度報告給記錄起來，要記錄哪些人哪些日子是沒有報告的。如此一來，就必須要有一個更完整的簽到系統才行，我想達成的目的有：

1. 我要可以在 slack channel 裡面看見每個人的進度報告
2. 我要有個地方可以記錄每天每個學生有沒有 po 進度報告（就把這個稱作簽到）

## 該如何製作這個簽到系統？
我的第一個想法就是使用 Google 試算表，畢竟這個東西最方便嘛，橫軸填入每個學生的帳號，縱軸填入每天的日期，如果有報告就給一個記號，沒報告就空白，這樣我就可以很明確地看出簽到紀錄了，成品長得像這樣：

![](/img/huli/lambda/sheet.png)

這樣子就完成第二個需求了，剩下來的就是想說如何完成第一個需求。最簡單的方式就是跟之前一樣，讓學生在某個 channel 裡面每天自己 po 就好，然後我寫個程式來串 Slack 的 webhook，接收到資料就寫進 Google 試算表裡面。

這個解決方案看似不錯，可是有個問題是課程用的 workspace 是免費版的，訊息到一定量之後就會被吃掉，就看不到以前的進度報告，我覺得這是有點可惜的，所以這個方案行不通。

接著我就想到另一個更不錯的解決方案：

1. 讓學生在 GitHub 的 Issue 下面留言
2. 留言同步到 Slack channel
3. 串接 GitHub Webhook，同步把紀錄寫到試算表裡面

這樣子的好處就是紀錄可以永久保存，而且還可以分天！可以很容易的就找到某一天所有人的進度報告，這個是直接 po 在 Slack 裡沒辦法做到的。

先給大家看一下成品，成品長得會像這樣：

![](/img/huli/lambda/issue.png)
![](/img/huli/lambda/comment.png)

有了對整個簽到系統的概念以後，接下來我們可以把在技術上要做的事情分成以下三項：

1. 每天要在 GitHub 開一個 Issue，標題是今天的日期
2. 每個 Issue 下的留言要被同步到 Slack
3. 要串接 GitHub Webhook，同步把紀錄寫到試算表裡面

再來就是實作時間了！

## 1. 每天要在 GitHub 開一個 Issue，標題是今天的日期

一看到「每天」這個關鍵字，就知道這是 Cron Job 可以搞定的事情，原本我想在自己的機器上寫個簡單的小程式讓它每天跑，可是我腦海中突然冒出一個關鍵字：AWS Lambda。

如果你還不知道這是什麼，我簡單說一下，這是近年很流行的 Serverless 的概念之一，不是說沒有 Server，而是指說你不用自己去管 Server，你唯一要做的就是把你的 application 寫好，剩下那些跟 Server 跟機器有關的事情你都不用管。

而 AWS Lambda 就是這樣的一個服務，你只要把你的程式碼放上去就好了，剩下的你都不用管，計費方式是程式的執行時間，可能是因為還在推廣期的關係，所以一個月在一定時數以內都不用錢。

如果不用 Lambda，我要自己把東西傳到我的 Server，然後自己設定 Cron Job 來跑，如果主機出了什麼事情還要自己來修，可是我想做的就只是這麼簡單的一件事情而已阿！用 Lambda 可以幫我省掉很多麻煩，絕對是最佳選擇。

確定要放在 Lambda 上之後，就是要按照它的要求把程式碼寫好放上去，其實這要求也很簡單啦，就是把你要執行的 function 用 `exports.handler` 給 export 出去就好。

下面是寫好的程式碼：

``` js
var axios = require('axios')
var moment = require('moment')
  
var token = process.env.token
var endpoint = 'https://api.github.com/repos/Lidemy/mentor-daily-report/issues?access_token=' + token
var today = moment().format('YYYY-MM-DD')
  
var content = [
  '在下面請按照此格式提供本日進度報告：',
  '```',
  '## 昨天',
  '- 寫作業 hw2-1',
  '- 練習 JavaScript 迴圈使用',
  '## 今天',
  '- 研究什麼是 callback',
  '- 寫作業 hw2-1（繼續）',
  '```'
].join('\n')
  
const createIssue = async (event) => {
  try {
    const result = await axios.post(endpoint, {
      title: '[進度報告] ' + today,
      body: content
    }, {
      headers: {
        'Accept': 'application/vnd.github.v3+json'
      }
    })
    return 'success'
  } catch (err) {
    return err;
  }
}
  
exports.handler = createIssue
  
```

一個非常簡單的程式，一執行就會透過 GitHub API 去 po 一個新的 Issue，標題就是今天的日期。

有一點需要注意的是這邊有用到其他 npm 的 library，應該有方法是可以只傳 package.json 上去，Lambda 就幫你執行 `npm install` 把那些套件抓下來，但我懶得查了，我就直接把`node_modules`包進壓縮檔裡面丟上去。

我上一次用 Lambda 大概兩三年前，這個服務才剛推出沒多久，因為好奇所以隨意玩了一下，發現介面很陽春然後很多東西不知道怎麼設定。

事隔多年，這次再看到它的介面真的嚇到我了，進步超級多！

首先是觸發條件這個部分一目瞭然：

![](/img/huli/lambda/l1.png)

因為我是要每天固定執行，在 AWS 上你可以用 CloudWatch 來設定一個排程，要注意的是在 Lambda 上面這個排程的時間會以 UTC 為準，也就是 +0 的時區，所以你如果是寫說每天 00:00 跑，其實就代表說是在台灣的早上 08:00 跑。

我這邊設定的表達式是：`5 0 ? * MON-FRI *`，在台灣時間平日早上 08:05 都會觸發 Lambda，執行 po issue 的 function。

然後在 Lambda 的介面上如果你的程式碼沒有很大，可以直接編輯，有功能完整的編輯器（我一直覺得很眼熟，後來才想到應該是 Amazon 買了 Cloud9 的關係，以前上 CS50 都用 Cloud9 的 IDE，難怪這麼熟悉）：

![](/img/huli/lambda/l2.png)

最後把環境變數 token 設定好之後就完成了，測試一下發現 Issue 有成功被建立，第一個任務就這樣輕鬆完成了，感謝 Lambda 的努力。

## 2. 每個 Issue 下的留言要被同步到 Slack

這是三個任務當中最簡單的，因為 Slack 本來的優勢就是可以跟很多現成的東西串接，只要在 Slack 上面安裝 GitHub App，就可以用指令來 subscribe 指定的 repo 跟事件。

因為這個實在是太簡單，所以我就沒必要再多介紹了，直接給大家看成果：

![](/img/huli/lambda/github.png)

## 3. 要串接 GitHub Webhook，同步把紀錄寫到試算表裡面

這個任務我們一樣用 Lambda 搭配其他 AWS 的服務就可以輕鬆實作出來，流程是這樣的：

1. 學生留言，觸發 GitHub Webhook
2. GitHub Webhook 打到 AWS API Gateway
3. 經由 API Gateway 觸發 Labmda function
4. Lambda function 透過 Google Sheet API 寫入試算表

我們先來把要丟給 webhook 的 API 給準備好，這邊利用 API Gateway 來觸發 Lambda，像是這樣：

![](/img/huli/lambda/api.png)

API Gateway 設定上也超級方便，你就設定要用什麼 HTTP method，他就會給你一個網址，假設我設定的是 GET，那你用 GET 打這個 API 它就會觸發 Lambda，用超短的時間就能做出一個 Webhook，比起自己架 Server 還要設定 domain 跟 https 方便得多。

再來就是要串接 Google Sheet API 了，我稍微看了一下官方的 API 發現還是維持一慣的風格，就是文件很完整但是講得很複雜，沒辦法一眼就看出我到底要怎麼實作出我要的功能，後來就找了一套別人包裝過的：[Simple Google Spreadsheet Access (node.js)](https://github.com/theoephraim/node-google-spreadsheet)，用起來簡單很多。

最麻煩的權限管理裡面也有教你怎麼實作，基本上就是去開一個`Service Account`，設定成對 Google Drive API 有權限，然後再去產生這個帳號的 token，用那一組 token 就行了。

主程式要做的基本上就是先過濾資料，接著從 GitHub 丟過來的資料裡找到帳號，把帳號跟日期丟進我另外寫好的 function，最後回傳結果就結束了：

``` js
var updateSheet = require('./lib')
  
exports.handler = async (event, context, callback) => {
    if (!event.body) return 'no body'
    const body = JSON.parse(event.body) || {}
    if (!body || body.action !== 'created') return response(callback)
    const title = body.issue.title.split(' ')
    if (!title.length) return response(callback)
    const date = title[1]
    const account = body.comment.user.login
    console.log('log:', date, account)
    try {
        await updateSheet(date, account)
        return callback(null, {
            statusCode: 200,
            body: date + account
        })
    } catch (err) {
        console.log('error:', err)
    }
      
    return response(callback)
};
  
const response = (cb) => {
    cb(null, {
        statusCode: 200,
        body: 'ok'
    })
}
```

`updateSheet` 這個 function 做的事也很簡單，就是根據日期還有帳號找到正確的位置，把那一格的值更改成 O 就好了，這邊附上部分程式碼供大家參考：

``` js
async function searchAccount(sheet, account) {
  const firstRow = await getCells(sheet, {
    'min-row': 1,
    'max-row': 1
  })
  const length = firstRow.length
  for(var i=0; i<length; i++) {
    if (firstRow[i].value === account) {
      return {
        col: firstRow[i].col,
        batchId: firstRow[i].batchId
      }
    }
  }
  return null
}
  
async function setValue(sheet, row, col, value) {
  const cells = await getCells(sheet, {
    'min-row': row,
    'max-row': row,
    'min-col': col,
    'max-col': col,
    'return-empty': true
  })
  if (cells && cells[0]) {
    cells[0].value = value
    cells[0].save(function(err) {
      if (err) {
        console.log('err', err)
      }
    })
  }
}
  
async function updateSheet(date, account) {
  try {
    const sheet = await getSheet()
    const accountPosition = await searchAccount(sheet, account)
    const datePosition = await searchDate(sheet, date)
    console.log('position:', accountPosition, datePosition)
    if (!accountPosition || !datePosition) return
    await setValue(sheet, datePosition.row, accountPosition.col, 'O')
  } catch (err) {
    console.log('err', err)
  }  
}
```

最後只要在 GitHub 那邊把 webhook 的網址設定好，一切就大功告成了！

## 在 Lambda 上要如何 debug？

雖然我上面寫的輕鬆寫意，但實際開發的時候我其實有碰到幾個小問題，第一個就是 debug 不像你在電腦上那樣可以直接看到結果，而且 webhook 通常又是比較難 debug 的一個。

有關於這部分，其實 Lambda 都會把 log 送到 CloudWatch 去，所以就是自己在 app 裡面寫 log，再去看 CloudWatch 分析一下那些 log，如果是簡單的應用的話還滿容易的，再複雜一點的話應該就是 function 要切更細，不然 debug 起來應該滿麻煩的。

另一個碰到的問題是 Google Sheet API 那邊速度比較慢，整個過程做完大概要 5 秒左右，預設的 timeout 好像是 3 秒左右，記得要自己把 timeout 加大不然會一直失敗。

## 總結

這次對於 AWS Lambda 的使用體驗滿不錯的，開發過程中沒碰到什麼太大的困難，可能跟我想實作的東西比較簡單也有關係，但這種簡單的東西我覺得超級適合走這種 Serverless 的方案，因為沒有 Server 真的超方便的，少了很多麻煩。

以後如果還有類似簡單的小需求，我想我應該還是會走這種解決方案，直接用現成服務一個串一個，把東西全部串起來就結束了。也推薦大家如果要做一些小東西，不妨來試試看用這些現成的服務搞定，可以節省很多時間。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好