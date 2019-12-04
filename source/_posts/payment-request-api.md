---
title: 統一網頁支付介面：Payment Request API
date: 2017-10-14 15:43:00
tags:
  - payment request
author: huli
---

# 前言

之前在 [Hacker News](https://news.ycombinator.com/item?id=15272374) 上面看到了這一篇文章：[Payment Request API — Now Being Implemented in All Major Browsers](https://www.w3.org/blog/wpwg/2017/09/14/payment-request-api-now-being-implemented-in-all-major-browsers-advances-on-the-recommendation-track/)，大意就是 `Payment Request API ` 這一個東西將會在主流瀏覽器上面被實作出來。

在這之前，我完全沒有聽過這個東西，完全不知道它在做什麼。但經過我稍微研究之後，發現這個原來是網頁支付介面的未來。

# Payment Request API 簡介

在瞭解一項新事物以前，我習慣先從「目的」下手，如果你知道這個東西是為了解決什麼問題而誕生，就能對它有最基本的了解。

而 Payment Request API 誕生的原因很簡單，就是為了解決支付問題，尤其是在手機上的支付。

先不要談手機，我們先來談電腦端的支付就好。

現在每個購物網站都有不同的支付介面，串接著不同的金流廠商。假設我今天在蝦皮買了《純粹理性批判》，填了信用卡號碼與收貨地址，蝦皮貼心的幫我記住，於是下一次我再購物時，就不必再填收貨地址了。

可是，如果今天 PChome 商家砸錢放送各種優惠，我決定轉到 PChome 上面購物，我買了一本《夢的解析》，就要再填了一次信用卡號碼跟收貨地址。

問題是什麼？

問題是每一間公司、每一個網站的結帳流程跟介面都不一樣，雖然大同小異，可是那些資料都是沒有辦法共用的。就算我在 100 個網站都填了收貨地址，我在第 101 個網站還是要重新填一次，因為他沒有我的資料。

上面講的是每一個網站有差異的地方，那有任何相同的地方嗎？

有，那就是你都用同一個瀏覽器購物。

# 瀏覽器的初次嘗試：自動填入

上面說的情景其實不太準確，因為你應該會發現其實瀏覽器會自動幫你記憶地址跟信用卡，就可以很方便地使用自動填入的功能。

例如說刷卡的時候只要刷過一次，就可以讓 Chrome 把這一張的卡的資訊記在瀏覽器裡面，下次再到別的網站刷卡時，你只要點一下輸入卡號的輸入框，Chrome 就會提示你說可以用之前的那張卡來付費。

地址也是如此，都有瀏覽器幫你記憶起來，這樣你就只要填一次之後，之後都可以由 Chrome 幫你自動填入。

可是，這樣還有一個問題，那就是結帳的流程跟介面還是不統一，大家都有自己不同的實作，有的支付介面簡直慘不忍睹，尤其是在手機上！

根據 Google 的[統計](https://developers.google.com/web/fundamentals/payments/payment-request-ux-considerations)，有 65.9% 的使用者在手機上購物時，還沒完成所有流程就離開了。這已經超過一半的用戶了，代表許多網站在手機的支付介面這一塊還有很大的努力空間。

而這一次，瀏覽器決定自己跳下來解決這個問題。

# 瀏覽器：都交給我吧！

瀏覽器要怎麼解決這個痛點？

簡單！只要由瀏覽器提供一個統一的結帳介面就好，連流程都一併統一。商家的網頁可以根據需求的不同，帶入不同的參數，但最終都是呼叫瀏覽器提供的 API（也就是我們今天的主角：Payment Request API），叫出瀏覽器原生的介面。

當這個 API 普及並且大家都發現比較好用的時候，所有的網頁都會跟進，都會採取一樣的方式。這樣就能夠確保所有網站的支付流程都統一了。

# 所以 Payment Request API 到底是什麼？

> 簡單來說，就是瀏覽器提供的 API，當網頁端以 JavaScript 呼叫以後，就會出現瀏覽器原生的結帳介面，用來取代原有的商家自有的結帳流程。

直接讓你看一張圖就會理解了：

![](/img/huli/payment/intro.png)

這個就是呼叫 API 之後的樣子。

要特別注意的一點是 Payment Request API 跟後端「完全無關」，你後端就跟以前一樣接收資料即可。有變動的地方是前端，原本你在前端需要寫的那些結帳頁面，都可以交由瀏覽器來 render 原生的 UI，你只要負責呼叫 Payment Request API 即可。

呼叫以後可以拿到使用者填入的那些資料，把那些資料像以前一樣發送到 Server 端就好。

但要注意的是這個 API 還不普及，根據 [caniuse.com](https://caniuse.com/#feat=payment-request) 的資料，只有 Chrome 61 版、Edge 15 跟 Opera 48 以上才支援，其他的瀏覽器還要再等等。

# 使用流程

說了這麼多，現在就來實際跑一次流程吧！

我們先建立一個簡單的 [demo 頁面](https://aszx87410.github.io/payment-request-demo/)，偵測是否支援 Payment Request 以及放置購買按鈕跟回傳的結果：

![](/img/huli/payment/interface.png)

## 第一步：創造 Payment Request 物件

`PaymentRequest`接受三個參數，付款方式、交易資訊跟其他。

``` js
var request = new PaymentRequest(
  methodData, // 支援的付款方式
  details,    // 交易相關的詳細資訊
  options     // 其他，例如說運送方式等等
);
```

我們先實作一個簡單的 function 回傳建立好的 PaymentRequest：

``` js
function createPaymentRequest () {
  var methodData = [{
    supportedMethods: ['basic-card'], // 支援信用卡
    data: { // 指定更詳細的資訊
      supportedNetworks: ['jcb', 'mastercard', 'visa'], 
      supportedTypes: ['debit', 'credit', 'prepaid']
    },
  }];
  var details = {
    displayItems: [ // 購買的品項
      {
        label: "TechBridge Weekly 專業版一年份",
        amount: { currency: "TWD", value : "3000.00" }
      },
      {
        label: "早鳥優惠",
        amount: { currency: "TWD", value : "-300.00" }
      }
    ],
    total:  {
      label: "總額",
      amount: { currency: "TWD", value : "2700.00" }
    }
  };
  
  return new PaymentRequest(methodData, details);
}
```

這邊有一點要特別注意，那就是`total`那邊的總額，系統「不會幫你自己算好」，所以儘管上面總和是 2700，你要輸入其他數字也可以。

還有另一個條件是這個 API 不支援退款，所以總和必須是正數。但是每一個品項可以是負數，這樣就可以放一些折扣相關的東西。

## 第二步：呼叫 API，顯示結帳頁面

建立完 PaymentRequest 之後，可以用`.show()`顯示結帳 UI，會回傳一個 Promise，使用過後可以拿到使用者的相關資料。我們在購買按鈕按下去之後來進行結帳流程。

``` js
function onClick () {
  var request = createPaymentRequest();
  request.show().then(function(PaymentResponse) {
    handleResponse(PaymentResponse);
  }).catch(function(err) {
    console.log(err);
  });
}
```

## 第三步：處理資料及回傳結果

最後一步就是處理上一步拿到的資料，把那些資訊發送到 Server 去完成結帳流程，並且傳回結果，讓 UI 顯示成功或是失敗。我們在這邊只是範例，所以就省略上述步驟，並且直接把上一步拿到的資料轉成 JSON 顯示出來。

``` js
function showResponse (response) {
  $res.innerHTML = JSON.stringify(response, undefined, 2);
}
  
function handleResponse (paymentResponse) {
  // 可以在這裡把結果回傳 server
  // 只是示範，所以我們直接將資料顯示出來
  showResponse(paymentResponse);

  // 模擬 API 的延遲
  setTimeout(function () {
    // 結帳成功
    paymentResponse.complete("success");
  }, 2000);
}
```

![](/img/huli/payment/result.png)

（這邊的卡號是我在 [http://www.getcreditcardnumbers.com/](http://www.getcreditcardnumbers.com/) 隨便產生的）

只要上面簡單的三個步驟，就能夠取得使用者的資料並且完成結帳。比起原先每個網站建立的自有結帳流程，使用 Payment Request API 的好處就是可以帶給使用者原生的結帳體驗，進而增加轉換率。

而上面的三個步驟，最重要的就是第一個帶入參數的部分，這邊還有很多細節可以調整，例如說貨幣種類、要求運送地址，並且可以根據使用者選擇的地址判斷接受或是不接受（例如說不接受送貨到國外，就能在那邊判斷）。

支付方式也可以指定某幾間的信用卡，或甚至是決定要不要支援 debit card。

如果你對這些細項有興趣，可以參考 Google 提供的非常詳細的教學：[Deep Dive into the Payment Request API](https://developers.google.com/web/fundamentals/payments/deep-dive-into-payment-request)。

# 原生結帳 UI

如果你想自己跑一遍結帳流程，可以直接去 [demo 網頁](https://aszx87410.github.io/payment-request-demo/) 試試看。

在這邊我直接截圖給大家看在電腦以及手機上面的結帳流程。

## 電腦

按下按鈕之後的畫面：
![](/img/huli/payment/intro.png)

點進訂單摘要：
![](/img/huli/payment/intro-details.png)

新增信用卡：
![](/img/huli/payment/add-credit-card.png)

新增地址：
![](/img/huli/payment/add-address-done.png)

按下支付之後，要求輸入末三碼：
![](/img/huli/payment/code.png)

結帳失敗：
![](/img/huli/payment/fail.png)

## 手機

按下按鈕之後的畫面：
![](/img/huli/payment/mobile-intro.png)

點進訂單摘要：
![](/img/huli/payment/mobile-intro-details.png)

新增信用卡：
![](/img/huli/payment/mobile-add-card.png)

新增地址：
![](/img/huli/payment/mobile-add-address.png)

結帳失敗：
![](/img/huli/payment/mobile-fail.png)

# 總結

Payment Request API 正在被其他瀏覽器（例如說 Safari）實作，可以預期到將來必定會被廣泛支援。

其實國外的金流廠商 Stripe [已經支援](https://stripe.com/docs/payment-request-api)使用 Payment Request API 了。而 [PaymentRequest Sample](https://googlechrome.github.io/samples/paymentrequest/) 這個網站也可以看到更多樣化的範例。

這篇文章主要目的是把這項新的標準帶到大家面前，如果覺得很有興趣想要深入研究，底下有附上許多相關資源。

參考資料：

1. [MDN - Payment Request API](https://developer.mozilla.org/en-US/docs/Web/API/Payment_Request_API)
2. [Deep Dive into the Payment Request API](https://developers.google.com/web/fundamentals/payments/deep-dive-into-payment-request)
3. [Introducing the Payment Request API](https://developers.google.com/web/fundamentals/payments/)
4. [PaymentRequest Credit Cards Sample](https://googlechrome.github.io/samples/paymentrequest/credit-cards/)
5. [w3c/payment-request-info FAQ](https://github.com/w3c/payment-request-info/wiki/FAQ)

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好