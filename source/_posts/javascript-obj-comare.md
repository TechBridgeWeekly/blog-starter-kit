---
title: 理解 Javascript 中物件的比大小行為
date: 2019-06-16 14:11:21
tags:
    - javascript
author: yiyu0x
---

## 前言

在 Javescript 中很多比較的情況非常不符合邏輯，但是卻是合法行為。了解比較的依據，可以讓我們更加掌握 Javascipt 的運作方式，揭穿他的神秘面紗。

## 先看幾種常見的情況

```javascript
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

這次的重點在於兩個物件的比較是依據何者來比較，首先要先知道 Javascript 有幾種型態

-   null
-   undefined
-   boolean
-   number
-   string
-   symbol
-   object

所以 Javascript 的型態都在這七種之內。然而，除了 object 以外，都屬於基本型態([primitive data type](https://developer.mozilla.org/zh-TW/docs/Glossary/Primitive))。

### 比較的依據

物件的比較流程為以下(若成功則停，失敗則往下一步)：

1. 呼叫物件內的 valueOf 方法求得 return 值(值必須為 primitive data type)

(若非 primitive data tpye 或是沒有 valueOf 方法則往下)

2. 呼叫 toString 方法求得 return 值(值必須為 primitive data type)

(若非 primitive data tpye 或是沒有 valueOf 方法則往下)

3. 拋出錯誤 (TypeError: Cannot convert object to primitive value)

## 進行實驗

1. 實驗一 (object 內的 valueOf)：

```javascript
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

```javascript
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

```javascript
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

```javascript
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

```javascript
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

## 物件預設的 valueOf, toString

```javascript
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

希望這篇物件比較的規則有幫助大家更了解 Javascript ， 此篇文章也同時刊登於我的個人 [blog](https://blog.yiyu0x.tk/)
