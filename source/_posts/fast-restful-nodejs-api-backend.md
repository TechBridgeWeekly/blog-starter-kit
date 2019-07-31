---
title: 用 Node.js 快速打造 RESTful API
date: 2016-04-23 09:58:08
tags:
  - Nodejs
  - RESTful
  - epilogue
  - Sequelize
  - Express
author: huli
---

## 前言

現今有些網站採用了 Single Page Application 的方案，後端只負責提供 API 讓前端抓取資料，達成完全的前後端分離。前端的選擇有很多種，你可以用 `Angular`，可以用 `Ember.js`，也可以用 `React + Redux`。至於後端的 API，必須符合固定格式，才能讓前端的人員比較好抓取。而這個「固定格式」，最常見的就是我們今天的重點：`RESTful`。

## 什麼是 RESTful？
與起從硬生生的文字解釋下手，不如先從實際範例著手。假設現在你要寫一個部落格網站的後端 API，十個人可能會有十種寫法；例如說「抓取所有文章」這個功能：

1. /api/blog/getList
2. /api/blog/getAllArticle
3. /api/blog/article/getAll
4. /api/blog/fetchAll
5. /api/blog/all

但如果是採取 `RESTful` 的方案，就會符合一定的格式：

| 操作 | Method    | URL    |
|----------|--------|----------------|
| 所有文章 | GET    | /api/posts     |
| 單一文章 | GET    | /api/posts/:id |
| 新增文章 | POST   | /api/posts     |
| 刪除文章 | DELETE | /api/posts/:id |
| 修改文章 | PUT/PATCH | /api/posts/:id |

在這個例子裡，文章（posts）是一個 `Resource`，你可以透過 HTTP 提供的幾種方法搭配不同的 URL 存取這個 `Resource`。

如果你對 `RESTful` 很有興趣，這邊是一些值得參考的文章：

1. [什麼是REST跟RESTful?](https://ihower.tw/blog/archives/1542)
2. [淺談 REST 軟體架構風格](http://blog.toright.com/posts/725)
3. [理解RESTful架構](http://www.ruanyifeng.com/blog/2011/09/restful.html)

## ORM
ORM 的全稱是：Object Relational Mapping  
如果以資料庫來說的話，就是把你的資料庫對應到程式裡的物件。舉上面的部落格的例子，你的資料庫 table 可能是這樣：

| 欄位 | 類型    | 說明    |
|----------|--------|----------------|
| id | int    | id    |
| title | text    | 標題 |
| content | text   | 內文   |
| created_at | timestamp   | 建立時間   |

對應到 Node.js 裡面的物件，你可以這樣：

```js
// 建立文章
Post.create({
  title: 'Hello Excel',
  content: '測試'
})

// 刪除 id 為 1 的文章
Post.find(1).delete();
```

也就是說，你今天根本不用管背後的資料庫用的是哪一種，也不用管 table 的名稱到底是什麼，你只要對你知道的這個 `Post` 物件做操作即可。

[Sequelize](http://docs.sequelizejs.com/en/latest/)是一套很好用的 ORM Library，只要先定義好一份`schema`，就可以幫你把物件跟資料庫關連起來。

## 為什麼突然提到 ORM？
有些讀者可能已經想到，其實 RESTful API 跟 ORM 之間，是有某種程度的關聯的。怎麼說呢？  

假設我今天要寫一個留言板的後端 API，而且我又同時採用 RESTful 跟 ORM，我的程式就會長這樣：

```js

// 抓取所有留言
// GET /api/messages
Message.findAll();

// 抓取單一留言
// GET /api/messages/:id
Message.find(id);

// 新增留言
// POST /api/messages
Messages.create({
  content: content
})

// 刪除留言
// DELETE /api/messages/:id
Messages.find(id).delete();

// 修改留言
// PUT /api/messages/:id
Messages.find(id).update({
  content: new_content
})

```

那如果我今天是寫一個部落格的後端 API 呢？  
把上面的 messages 全部換成 posts，搞定！  
從以上例子可以看出，其實這兩樣東西是很適合搭配在一起的，因為兩個都能夠符合差不多的規則。

## 兩個願望一次滿足，epilogue

[epilogue](https://github.com/dchester/epilogue) 是一套 Node.js 的 Library，它結合了 `Sequelize` 跟 `Express`，主要目的就是讓我們能快速打造出 RESTful 的 API。

讓我們直接來看看官網的範例：

首先，你要先定義好的資料庫，跟你的 schema

```js
var database = new Sequelize('database', 'root', 'password');
var User = database.define('User', {
  username: Sequelize.STRING,
  birthday: Sequelize.DATE
});
```

再來，初始化 express 跟 epilogue

```js
var express = require('express'),
    bodyParser = require('body-parser');

var app = express();
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: false }));
server = http.createServer(app);

epilogue.initialize({
	app: app,
	sequelize: database
});
```

最後，靠 epilogue 把 url 跟資料庫關連起來，你要提供它你想要的 endpoint 跟要關連的 model

```js
var userResource = epilogue.resource({
  model: User,
  endpoints: ['/users', '/users/:id']
});
```

就是這樣簡單三個步驟，你就有了一個 RESTful API！是不是很簡單呢？

## 還不只這樣
在實際的開發過程中，其實往往沒有那麼順利，例如說你的回傳格式可能跟資料庫的格式不一樣，或是你的某些 API 需要經過認證才能呼叫。沒關係，epilogue 都幫你想好了。

epilogue 提供了七種行為的 hook，包括 start, auth, fetch, data, write, send, complete，再搭配上 before, action, after 三種，你可以在任何一個階段做你想做的事情。

例如說你想在傳回結果之前做一點小小的變更，就是`userResource.list.send.before`，或是你可能想對某個 API 做驗證，那就是`userResource.delete.auth`。

這邊提供兩個官網的完整範例：

```js
// 禁止刪除 user
userResource.delete.auth(function(req, res, context) {
  throw new ForbiddenError("can't delete a user");
})

// 先看有沒有 cache，有的話直接返回 cache 的內容
userResource.list.fetch.before(function(req, res, context) {
  var instance = cache.get(context.criteria);

  if (instance) {
    // keep a reference to the instance and skip the fetch
    context.instance = instance;
    return context.skip;
  } else {
    // cache miss; we continue on
    return context.continue;
  }
})
```

## 總結
若是你的後端 API 沒有很複雜，都是基本的 CRUD 的話，那 epilogue 絕對是很適合你的一套框架，只要你把資料庫的 schema 開出來，程式碼複製貼上一下就能夠完成一個 API。若是讀者之後有相關的需求，不妨試試看吧！


關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好