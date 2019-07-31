---
title: 記一次軟體工程師面試心得
date: 2017-01-27 17:06:06
tags:
    - interview
    - frontend
    - job
    - resume
author: huli
---

## 前言
我自己很喜歡看一些面試的文章，因為可以從裡面學到許多，如果有面試題目的話那就更棒了，可以邊看邊想，測測看自己的實力。這篇文章是我前一陣子面試某間公司的歷程跟一些心得，希望對大家會有幫助。順帶一提，最後是有拿到 offer 的。

我面試的職缺叫做 Software Engineer，軟體工程師，四次面試都是中文的（我原本以為會有英文）。HR 有先跟我聊過，我有說我比較想要做前端，所以有些面試的考題會比較偏前端。但因為這間公司也很注重 CS 的基礎，所以依然會有很多資工本科的問題。（作業系統、資料庫、資料結構與演算法等等）

因為公司不在台灣，所以面試的形式都是採用 Skype 視訊加上某個線上 coding 的網站。現在很多公司的遠端面試都很喜歡採用這種形式，就是出題目給你讓你即時 coding，面試官那邊看得到你打字（就跟 hackpad 或是 Google docs 一樣），也可以直接 compile 看看答案對不對。

## 一面
一面的面試官是個前端工程師，所以問題會比較偏前端。程式語言的部分我是選擇用 JavaScript 來作答。

### 第一題
在 Facebook 上面，對於按讚這個功能，通常會有以下幾種描述：
1. 只有一個人按讚：`A likes this`
2. 有兩個人按讚：`A and B like this`
3. 有三個人按讚：`A, B and C like this`
4. 有四個人以上按讚：`A, B and 2 others like this`

寫一個 function，會給你一個叫做 names 的陣列，根據 names 輸出結果。

這一題很基本，我覺得主要是想看你會怎麼解決這個問題，以及讓你小試身手一下（怕你太緊張）。我自己的話是直接寫 if...else 去判斷。

另外，記得把一些 edge case 也考慮一下，例如說 names 有可能是空陣列嗎？如果是的話應該輸出什麼？

``` js
function like(names) {
    var output = " likes this";
    if(names.length<=0) return "";
    if(names.length===1) {
        return names[0] + output;
    }
    
    output = " like this";
    if(names.length==2) {
        return names[0] + " and " + names[1] + output;
    }
    if(names.length==3) {
        return names[0] + ", " + names[1] + " and " + names[2] + output;
    }
    return names[0] + ", " + names[1] + " and " + names.length-2 + " others" + output;
}
```

### 第二題
幫 array 加一個 `any` 的 function，可以輸入任一 function，如果任何一個元素丟進那 function 裡面的值是 true，就輸出 true，否則就 false。

例如說現在有個 function 叫做：isGreaterThanZero，範例就會是這樣：

``` js
[1, 2, 3, 4, 5].any(isGreaterThanZero); //=> true
[-1, 0].any(isGreaterThanZero); //=> false
```

這題應該主要是考會不會用 `Array.prototype` 來新增函式上去，如果會的話，實作的部分則沒有什麼難度。一樣附上我的解答：

``` js
Array.prototype.any = function(func){
    var len = this.length;
    for(var i=0; i<len; i++) {
      if(func(this[i])) return true;
    }
    return false;
}
```


### 第三題

對於這一行程式碼：`var MyObject = new MyClass();`可以試著講講看背後做了什麼事嗎？有沒有可能不用 `new` 但是卻達成一樣的效果？

這一題我答的頗差的，主要是考對 JavaScript prototype 機制的理解程度，想瞭解更多的可以參考：[JavaScript 語言核心（15）函式 prototype 特性](http://openhome.cc/Gossip/CodeData/EssentialJavaScript/Prototype.html)。

### 第四題
有些很慢的 function，我們想把它 cache 起來，下次執行時就不會那麼慢，例如說：

```
complexFunction('a', 'b'); //=> 10s
complexFunction('a', 'b'); //=> 10s

var ccf = cache(complexFunction);

ccf('a', 'b'); //=> 10s
ccf('a', 'b'); // Instant
```
寫出一個 cache 的 function，可以把傳進來的 function 加上 cache 功能

這一題其實考的點滿多的，第一個是考你會不會回傳一個 function（雖然這也滿基本的啦），第二個是考你知不知道怎麼做 deep equal。

因為他範例給的是兩個字串，所以乍看之下好像不是很難，但再追問下去，就會知道應該要寫出一個能夠 cache 所有參數的版本，意思就是如果參數是 Object 的話也應該能做 cache。所以就要紀錄參數跟輸出的組合，去查一下有沒有已經算過的參數，有的話就直接輸出結果。

這題我覺得要做到完美的話其實也挺難的，可以再優化的點應該很多。

我最後完成的不完美的版本：

``` js
function cache(func) {
    var cached = [];
    
    function deepEqual(obj1, obj2) {
        var obj1Type = typeof obj1;
        var obj2Type = typeof obj2;
        if(obj1Type !== obj2Type) {
          return false;
        }
        
        // same type
        if(obj1Type!=='object') {
          return obj1===obj2;
        }
        
        // object
        for(var key in obj1){
            if(!obj2[key]){
              return false;
            }
            // have the same key
            if(!deepEqual(obj1[key], obj2[key])){
                return false;
            }
        }
        return true;
    }
    
    return function(){
        var isCached = false;
        var cacheResult = null;
        
        for(var i=0; i<cached.length; i++){
            if( deepEqual(cached[i].arguments, arguments) ) {
                isCached = true;
                cacheResult = cached[i].result;
                break;
            }
        }
        
        if(isCached) {
            return cacheResult;
        } else {
            var result = func.apply(null, arguments);
            cached.push({
              arguments: arguments,
              result: result  
            })
            return result;
        }
    }
}
```

### 第五題
用 CSS 實現一個很簡單的網格系統，假設一行有 12 格好了，請問像是`col-4`這種 class 的 CSS 應該怎麼寫？

因為我 CSS 原本就比較弱，所以還滿怕碰到 CSS 問題，也沒有自己實作過網格系統。所以這一題我就答說：`display: inline-block; width: 25%`。

後來就被提示說，概念差不多，可是實作上會碰到一些問題，因為預設會有 margin，所以四個雖然看起來會排滿，但其實會排成兩行。要再加上一些修正才行。

一面差不多就到這邊結束了，問的都是一些基本的 JavaScript 跟 CSS 的題目，我覺得沒有特別困難的。所以我自己覺得一面的表現算是還不錯啦。

## 二面
考的問題都是一些比較偏資工本科系的問題了，問題大致如下：

1. tcp 連線時有三次握手，那關閉時候的流程是什麼（考你四次揮手的流程）
2. 有寫過 linux 上的程式嗎（應該是考你有沒有寫過系統程式）
3. thread 跟 process 的差別（經典考題）
4. mysql 的 index 有哪些，差在哪？（考對資料庫的熟悉程度）
5. hashmap 怎麼實現（考資料結構）
6. virtual memory 在幹嘛（考作業系統）
7. 資料庫欄位 TEXT 跟 VARCHAR 差異（考資料庫）

可以看到考題還滿廣的，網路、系統程式、作業系統、資料庫、資料結構全部都有問到。這邊是我的弱項，所以都答得滿差的...

## 三面
原本以為二面完就掰掰了，但我自己猜測因為我一面的表現還不錯，綜合起來還是進到了第三面。結果第三面考的依舊是我不太在行的部分。

第三面我有印象的題目只有一題而已，考完之後上網查了一下發現也是滿經典的題目。

假設現在有很多 url 的資料，每筆一行，請找出重複次數最多的前 k 個，並講一下時間複雜度是多少？

其實就是一個純粹考資料結構加演算法的題目，看你要用哪些組合來處理這個問題。但光是知道還不夠，也要能夠把時間複雜度一併講出來才行。我記得我最後給出的解法應該是維護一個有 k 個元素的陣列，然後用 map 來存 url 的出現次數，邊讀資料邊更新出現次數，然後也更新那 k 個元素的陣列，找到最後就是答案了。但這個解法我的時間複雜度也沒有回答得很好，有些地方不太清楚時間複雜度是多少。

對這類型的題目有興趣的話，可參考：[十道海量数据处理面试题与十个方法大总结](http://blog.csdn.net/v_july_v/article/details/6279498)。

## 四面

最後一面依舊是問了很多技術，這一輪面試被問的問題最多，而且也有針對我以前做過的東西去問。我覺得廣度跟深度都有顧及到。

### 第一題
先小試身手，寫一個 reverse 的 function，反轉字串

``` js
function reverse(s) {
    var output = "";
    var len = s.length;
    for(var i=len-1; i>=0; i--){
        output+=s[i];
    }
    return output;
}
```

### 第二題
寫一個 function：

```
var main = [1,2,3,4,5,6];
var sub = [3,5];
removeSubArray(main, sub) -> [1,2,4,6]
```

這一題比較特別的點是要直接改變傳進來的 array 而不是 return 新的

``` js
function removeSubArray(main, sub) {
   for(var i=0; i<main.length; i++){
       var temp = sub.indexOf(main[i]);
       
       //remove
       if(temp>=0) {
           sub.splice(temp, 1);
           main.splice(i, 1);
           i--;
       }
   }
}
```

### 第三題
寫一個 copy 的 function，複製傳進來的 object

這一題雖然看似簡單但其實陷阱很多，例如說：如果碰到循環引用的 case 怎麼辦？  
例如說

```
var a = {
  test: {
  }
};
a.test = a;
```

有些 library 提供的 deep clone 函式，碰到這個 case 一樣也會掛掉。但在面試的時候這個 case 我依舊解不掉，需要再花點時間想想。

### 第四題
寫一個 closure（什麼樣子的都可以）  

這一題還滿簡單的啦，只要你知道什麼是 closure 應該都寫得出來。

``` js
var increment = function(){
 var times = 0;
 return function(){
    return ++times;
 }
}

```

### 第五題
問我之前做過的 Android 熱更新怎麼做的，問很多技術細節。

### 第六題
問我之前做的搶紅包活動是怎麼設計的？這邊的實作在我之前寫過的文章：[資料庫的好夥伴：Redis](http://blog.techbridge.cc/2016/06/18/redis-introduction/)裡面有提到過。

那個活動的需求是：

1. 中午 12 點開放使用者進入網站，並且回答一題問題
2. 回答完後會看到自己的排名（依答題時間排序），照名次獲得獎品
3. 只有前 300 名有獎品，之後都沒有

評估過後我決定引入 Redis 來減輕資料庫的負擔。面試官問的問題是：`有沒有辦法寫成：300 名以後連答題頁面都看不到，直接擋在外面`。這邊我想很久之後還是沒有想到一個比較好的解法，就只想到把 Database lock 起來，但這樣又會造成很多性能的損耗。最後我有問面試官這題應該怎麼解，他說他也還沒有想法，只是問問看有沒有可能而已XD

### 第七題
講一下 session 跟 cookie 的差別，還有 session 存在哪裡、如果有兩台機器怎麼管理 session？  

剛好這一塊我之前有弄懂過，所以回答的還滿完整的。在之前公司也有碰過兩台機器要共用 session 的情況，我先給了他一個 AWS EC2 上面的 sticky session 的解法，他不太滿意。再來給了用 Redis 統一儲存 cache 再搭配 Redis Replication 來做 HA 的解法，他就很滿意的說：「嗯，這樣可以」

### 第八題
問資料庫的index的概念，為什麼要有 index？index 應該怎麼建？

因為資料庫跟後端那邊我沒有很熟，所以這題也是回答得亂七八糟的。

## 結論
其實這樣四次面試下來，每一次都問技術也是還滿累的，但每次面試也都是可以檢驗自己技術能力的好時刻。你會越面試越知道自己哪一個部分不足，例如說我就是對於資工本科系的那些必修比較不熟，所以之後就應該往這個方向加強。

還有，很多時候我都喜歡自己默默思考，思考出一個自己覺得可行的解法以後才跟面試官講。可是我後來發現這樣做不太好，因為你不說話，他也不說話，整個面試過程就會有很多空白，兩個都不講話。所以可以的話，應該要邊想邊講話我認為是比較好的。例如說：「我剛剛提出的解法是 O(n^2)，我現在在想說有沒有可能把複雜度壓到 O(n log n)，我目前的方向大概是朝...去思考」

儘管最後沒有答出來，有把想法講出來，總比只給一句：「我不知道」來得好。

面試的時候，其實我也都是抱著一種學習的心態。因為知道自己的實力真的不強，所以也希望能從面試官那邊得到一些建議。很多題目我在答不出來之後都會問他們說能不能提供一些方向，因為我真的很想知道答案（不過有些不一定會給就是了）。

大致上就是這樣囉，希望這篇心得對大家之後的面試有幫助。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好