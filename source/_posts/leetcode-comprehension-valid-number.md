---
title: 記一次 Leetcode 刷題體悟 - Valid Number
date: 2019-08-03 16:51:34
tags:
- Algorithm
- Leetcode
- Software Engineer
- 軟體工程師面試
author: pojenlai
---

## 前言

身在大 CS 時代，可能很多人有刷題的經驗，也可能像筆者一樣正經歷刷 Hard 題的各種撞牆。但在這種撞牆的時刻，我們反而可以來觀察自己的思考方式是不是有問題，才會導致撞牆。

今天，就讓我們一起來看一題令許多人抓狂的 valid number。

## 題目介紹 - [Valid Number](https://leetcode.com/problems/valid-number/)

題目敘述如下：

![img](https://i.imgur.com/l7UEQW5.png)

基本上就是要判斷一個字串是不是可以被當作一個數字。如果你試著去解解看這題，你可能會發現一件事，就是你很容易不斷漏考慮一些 case。但你不孤單，只要看看這題精美的通過率和 dislike 數就可見一班XD

![img](https://i.imgur.com/RhzYnE7.png)

![img](https://i.imgur.com/wSYdZNU.png)

但如果我們也像大家一樣覺得這題出得很爛就不想學，那就非常可惜了。因為如果你堅持下去，會看到很不一樣的風景。接下來，就讓我們一起看下去。

## 解法一 - 將各階段分類好漸進處理

這種題目最麻煩的地方就在於，如果沒有把規則想好，就會涵蓋不到一些 edge cases，進而為了處理這些奇怪 case 使得邏輯變得很混亂，甚至會發生修了 A bug 結果又產生 b 跟 c bug 的慘況。

所以在有模糊的概念就開始寫 code 之前，我們可以先想想合理的 case 應該要長什麼樣子，先寫出第一版的 valid pattern（[] 裡面表示是 optional，類似 [linux command line tool 的說明](https://stackoverflow.com/a/21503966)）：

```
[+/-] 數字(可以是小數) [e數字(不能是小數)]
```

而不屬於這個 valid pattern 的其他字串都視為 invalid。

接下來我們可以用題目中提供的 test cases 檢視一下我們的 pattern 是否已經足夠，會發現有幾個例子沒有被涵蓋到：

- " 0.1 " => true (最前面跟最後面都可以有 space)
- " 6e-1" => true (e 後面的數字可以有 +/-)

```
[spaces][+/-]數字(可以是小數)[e[+/-]數字(不能是小數)][spaces]
```

所以我們的邏輯應該要依序處理：

1. Skip spaces
2. Check '+'/'-'
3. Check digital(can contain ".")
4. Check exponent
a. Check '+'/'-'
b. Check digital(cannot contain ".")
5. Check space

這時再開始寫程式，就覺得邏輯清晰，輕舟已過萬蟲山：

```cpp
class Solution {
public:
  bool isNumber(string s) {
    int i=0;
    
    // Skip the spaces
    for(; s[i] == ' '; i++) {}
    
    // Check for '+'/'-'
    if(s[i] == '+' || s[i] == '-') i++;
    
    // Check digital
    int num, pt;
    for(num=0, pt=0; (s[i]<='9' && s[i]>='0') || s[i]=='.'; i++)
      s[i] == '.' ? pt++ : num++;
    
    if(pt>1 || num<1) // no more than one point, at least one digit
    return false;
    
    // Check exponent
    if(s[i] == 'e') {
      i++;

      // Check '+'/'-'
      if(s[i] == '+' || s[i] == '-') i++;

      // Check digital(cannot contain ".")
      for(num=0; (s[i]<='9' && s[i]>='0'); i++, num++) {}
      if(num<1) // at least one digit
        return false;
    }
    
    // Skip spaces
    for(; s[i] == ' '; i++) {}
    
    // must reach the end of string
    return s[i]=='\0';
  }
};
```

但到這邊還沒完，我們還可以對這題挖得更加深入。

## 解法二 - Regular Expression

在解法一寫出合理 pattern 的時候，我就想到了 regular expression。

雖然 regular expression 的寫法我已經忘了，但從這個影片 [Regular Expressions (Regex) Tutorial: How to Match Any Pattern of Text](https://www.youtube.com/watch?v=sa-TUpSx1JA) 重新學習，再參考一下相關的 [討論串](https://leetcode.com/problems/valid-number/discuss/23821/Regex-with-detailed-explanation.-How-can-we-write-the-regular-expression
) ，就可以寫出一個神簡潔的程式碼 ：

```cpp
class Solution {
public:
  bool isNumber(string s) {
    //[spaces][+/-]數字(可以是小數)[e[+/-]數字(不能是小數)][spaces]
    //regex pattern("\\s*[+-]?(([0-9]*\.?[0-9]+)|([0-9]+\.?[0-9]*))([e][+-]?[0-9]+)?\\s*");
    regex pattern("\s*[+-]?(([0-9]*\.?[0-9]+)|([0-9]+\.?[0-9]*))([e][+-]?[0-9]+)?\s*");
    
    return regex_match(s, pattern);
  }
};
```

這個版本無法通過 leetcode 上的 test cases，不過我用 Sublime Text 測試時結果正確。

雖不確定是什麼問題，但現在的目的不是逼自己要把所有東西都寫對，我只是想欣賞欣賞簡潔解的美好，所以 bug 先擱著。看看這程式碼真的覺得神清氣爽。

## 解法三 - Deterministic Finite Automaton

基於好奇的心理，我又去翻了一下討論區，結果看到一個超猛的東西 - DFA (Deterministic Finite Automaton)。這傢伙基本上就是一個狀態機，根據你的輸入會在不同狀態間跳來跳去，所以你最後只要看有沒有跳到 valid state 就好，超酷。

```cpp
class Solution {
public:
  bool isNumber(string s) {
    int state = 0;
    vector<vector<int> > transTable = {
      {2, 1, -1, 3, 0, -1}, // Q0
      {2, -1, -1, 3, -1, -1}, // Q1
      {2, -1, 5, 4, 8, -1}, // Q2
      {4, -1, -1, -1, -1, -1}, // Q3
      {4, -1, 5, -1, 8, -1}, // Q4
      {7, 6, -1, -1, -1, -1}, // Q5
      {7, -1, -1, -1, -1, -1}, // Q6
      {7, -1, -1, -1, 8, -1}, // Q7
      {-1, -1, -1, -1, 8, -1} // Q8
    };
    
    bitset<9> validStates("110010100");
    
    for(char c: s) {
      int type = inputType(c);
      state = transTable[state][type];
      if(state == -1) return false;
    }
    
    return validStates[state];
  }
private:
  int inputType(char c) { // use type ID as index to get next state in the transition table.
    if(isdigit(c)) return 0; // T0
    if(c == '+' || c == '-') return 1; // T1
    if(c == 'e') return 2; // T2
    if(c == '.') return 3; // T3
    if(c == ' ') return 4; // T4
    else return 5; // T5
  }
};
```

這種解法真的有夠帥，假設你今天沒有要找工作或得把這東西學會的壓力，是不是肯定會感受到一股開心的感覺，因為被開眼界了！而不是一堆自己得學會，矮唷我怎麼想不出來等等的各種延伸情緒。

因為我之前也沒有真正學過 DFA 的理論，就不多說了，有興趣的讀者可以參考一下這兩個討論串，應該就能清楚：

1. [[C++] My thought with DFA](https://leetcode.com/problems/valid-number/discuss/23725/C%2B%2B-My-thought-with-DFA)
2. https://leetcode.com/problems/valid-number/discuss/23798/Cleanest-C%2B%2B-Solution-using-DFA-(impress-your-interviewer-with-elegant-code!!)

## 總結

今天跟大家分享了一次刷 Hard 題的心路歷程，主要也是希望大家更把心情放輕鬆，享受刷題過程，畢竟人生苦短，既然有緣來刷題，不如就開開心心地學習，越寫越開心才會越寫越厲害，希望這篇文章對於正在題海中奮鬥的工程師們有些幫助。

## 延伸閱讀

1. [刷题需要hard一起刷嗎](https://www.1point3acres.com/bbs/thread-537289-1-1.html)
2. [讓刷題幸福感提高的一百個心得](https://www.1point3acres.com/bbs/thread-521357-1-1.html)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
