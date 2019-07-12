---
title: 理解 JavaScript 中物件的比大小行為
date: 2019-06-16 14:11:21
tags:
    - JavaScript
author: yiyu0x
---

## 前言

在 JavaScript 中很多行爲用其他程式語言的角度或是直觀角度來看都非常不合理。有些人知道這些行為，但不了解內部運作方式。而我自己在學習時，會嘗試了解內部的行為。了解過後，之後遇到同樣行為時，很自然的就知道某行為應該回傳什麼樣的值，而不再認為這是一顆地雷。

這篇文章要探討的是 JavaScript 在兩個物件中互相比較所用的依據為何。網路上大部分的文章都只有討論 `==`, `!=`, `===` 以及 `!==`。這篇文章要探討的是 ***在兩個物件之間使用大於小於去比較*** JavaScript 如何處理這一件事情。

## 先看幾種常見的情況

先複習一下幾種不同型態之間的比較依據

```JavaScript
> 100 > 10
true
> 100 > "10" //(字串與數字比較，會將字串轉為數字來比較，使用 Number()轉型)
true
> "100" > "10" //(兩者都是字串，按照 ascii code 比大小，如果相等就往下一個字元比。切記！並不是將兩個字串都轉為數字來比較)
true
> "abc" > "aaa" //(同上述)
true
> "ab" > "ac"
false
```

## 物件的比較規則

### 來點科普

這次的重點在於兩個物件的比較是依據何者來比較，首先要先知道 JavaScript 有幾種型態

-   null
-   undefined
-   boolean
-   number
-   string
-   symbol
-   object

所有 JavaScript 的型態都在這七種之內。然而，除了 object 以外，都屬於基本型態([primitive data type](https://developer.mozilla.org/zh-TW/docs/Glossary/Primitive))。

### 比較的依據

物件的比較流程為以下(若成功則停，失敗則往下一步)：

1. 呼叫物件內的 valueOf 方法求得 return 值(值必須為 primitive data type)

(若非 primitive data tpye 或是沒有 valueOf 方法則往下)

2. 呼叫 toString 方法求得 return 值(值必須為 primitive data type)

(若非 primitive data tpye 或是沒有 valueOf 方法則往下)

3. 拋出錯誤 (TypeError: Cannot convert object to primitive value)

## 進行實驗

1. 實驗一 (object 內的 valueOf)：

```JavaScript
let obj = {
    valueOf() {
        return 123;
    },
    toString() {
        return {};
    }
};
if (obj > 100) console.log("great!");
//output: great!
```

2. 實驗二 (valueOf 回傳值不是 primitive data type)：

```JavaScript
let obj = {
    valueOf() {
        return {};
    },
    toString() {
        return 123;
    }
};

if (obj > 100) console.log("great!");
//output: great!
```

3. 實驗三 (valueOf 與 toString 的比較順序)：

```JavaScript
let obj = {
    valueOf() {
        return 200;
    },
    toString() {
        return 50;
    }
};

if (obj > 100) console.log("great!");
else console.log("QQ");
//output: great!
```

```JavaScript
let obj = {
    valueOf() {
        return 50;
    },
    toString() {
        return 200;
    }
};

if (obj > 100) console.log("great!");
else console.log("QQ");
//output: QQ
```

4. 實驗四（兩者回傳值都非為 primitive data type，拋出錯誤）

```JavaScript
let obj = {
    valueOf() {
        return {};
    },
    toString() {
        return {};
    }
};

if (obj > 100) console.log("great!");
//output:
//if (obj > 100) console.log("great!");
//        ^

//TypeError: Cannot convert object to primitive value
//    at Object.<anonymous> (/Users/yiyuchang/dev/tmp/article/example.js:10:9)
//    ......
```

## 空物件預設的 valueOf, toString 為何

```JavaScript
let obj = {};
console.log(obj.valueOf());
console.log(obj.toString());
if (obj == "[object Object]") console.log("great!");
//output:
//
//{}
//[object Object]
//great!
```

預設情況下物件是的 valueOf 是回傳空物件 {}，而 toString 則是回傳 [object Object] 字串！

## 結論

透過以上的實驗，除了知道物件依據何者來比較大小。相信也對 `valueOf` 以及 `toString` 有更深一層的了解。如此一來對物件這個屬性的行為也能加以掌握。

JavaScript 的型態簡單來說就如同前面所說，一類為 primitive data type 另外一類就是物件(object)。很多新手常常搞混的行為其實也就只是這兩種屬性不太了解罷了。

像是:

```JavaScript
let str = 'hi'
let str2 = String('hi')
```

這兩個字串的內容到底一樣不一樣？用 `==` 以及 `===` 的回傳值為何？

1. 兩個變數內容都是 'hi'，所以用 `==` 的結果會是 `true`
2. 他們都是 primitive data type，所以用 `===` 包含型態也一起比較當然也會一樣

但如果用 `new String('hi')` 產生出來是一個物件，所以與前面兩者用 `===` 比較結果則會是 `false`。

理解這些平常不會仔細思考的問題更有助於掌握這門語言，日後我也會時常將我的學習歷程寫成文章，發佈於我的部落格與此，謝謝大家仔細看完這篇文章。

## 關於作者

大家好，我是 yiyu。在網路上都用 [yiyu0x](https://github.com/yiyu0x) 作為我的 id。目前是資訊工程學系大三的學生，平常喜歡寫程式，用程式解決日常生活中的問題。
JavaScript 是一門近期我非常熱愛的語言，相較於其他語言，豐富的套件以及異步的特性是我熱愛它的原因。此篇文章也同時刊登於我的個人 [blog](https://blog.yiyu0x.tk/)。