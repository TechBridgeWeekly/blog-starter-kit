---
title: ZOOX 自動駕駛公司面試題目
date: 2019-07-05 16:51:34
tags: ZOOX, Online Assessment, Interview
author: pojenlai
---

## 前言

[ZOOX](https://zoox.com) 是一間做自動駕駛的新創公司，今天就來跟大家分享一下他們常出的 OA（Online Assessment）題目跟解法（用 C++ 實作）。題目出自[一畝三分地論壇的分享](https://www.1point3acres.com/bbs/thread-438894-1-1.html)。

## 題目一 - Arithmetic expression evaluation

### 題目敘述

```
Write a program that takes a single line of input,
representing an arithmetic expression.

The expression will consist of numeric digits (0-9),
the plus operator (+) and the multiplication operator (*).
The given expression will be a valid arithmetic expression
(ie. "*2++" is not valid).

Your task is to evaluate the arithmetic expression,
following the normal rules of operator precedence,
and print the result without any leading or trailing whitespace.

The resulting numbers will fit in a normal integer.

Example Input
20+2*3
Example Output
26
```

### 題意理解與連結

這題基本上就是要寫一個基礎版的計算機(只有 0-9, +, *)，對應到的 leetcode 題目是： [227 - Basic Calculator II](https://leetcode.com/problems/basic-calculator-ii/)。

### 解法

要思考這個題目，最關鍵的地方在於我們需要處理的 operator - '+' 跟 '\*'，這兩個 operator 有一些性質：

1. \+ 跟 \* 都符合交換律，A+B+C = (A+B)+C = A+(B+C)，A\*B\*C = (A\*B)\*C = A\*(B\*C)。

2. \* 要比 \+ 先被計算，不然會得到錯的結果。

其中 2 是我們要比較注意的，以高階的概念來思考：

1. 要嘛就是先處理掉所有 \* 的運算，最後就只剩 +，那就把剩下的所有數字加總就好
2. 不然就是要能夠在遇到 \* 的時候，把 \* 前面的數字再拿出來，跟 \* 後面的數字相乘完，然後加回去

而如果要實作成程式，2 的做法是比較容易的(因為我們可以順著 input 的方向 iterate 過 input 一次就好)，比如說 20+2\*3，如果我們可以先用一個資料結構存 [20, 2]，那當我們遇到 3 的時候，我們就可以把 2 從資料結構中拿出來，跟 3 相乘之後再放進資料結構，變成存 [20, 6]。

這樣做的好處是感覺可以擴展，例如 20+2\*3\*5，遇到 5 的時候，就從 [20, 6] 拿出 6，跟 5 相乘完再放回資料結構，資料結構裡面存的東西就變成 [20, 30]。然後最後再把資料結構中的的所有數字加總就好。

所以到目前為止，我們想出了一個可能的演算法：

- 如果遇到數字，就先計算這個數字的值（要記住我們現在的 input 是字串，所以需要把數字的部分處理完才能得到 int）
- 如果數字前面的 operator 是 \+，就先把目前的數字放進資料結構
- 如果數字前面的 operator 是 \*，就把資料結構中最晚放進去的數字拿出來，跟現在的數字相乘完，再放進資料結構
- 處理完字串，加總資料結構中所有數字

看到這邊，我們會發現這個資料結構主要得有兩個功能：

1. 記錄所有的數字
2. 能夠吐出最後放進去的數字

那顯然就是一個 stack。

把以上的想法寫成程式碼，就可以實作出下面的 code 囉：

```cpp
class Solution {
public:
  int calculate(string s) {
    // 如果字串裡沒東西，那答案就是 0
    int len = s.length();
    if(len == 0) return 0;

    stack<int> st;
    int num = 0;
    char sign = '+';
    
    for(int i=0; i<len; i++){
      // 如果遇到數字，就先計算這個數字的值
      if(isdigit(s[i]))
        num = num*10 + (s[i]-'0');
      
      if((!isdigit(s[i]) && ' ' != s[i]) || i==len-1){
        // 如果數字前面的 operator 是 +，就先把目前的數字放進資料結構
        if(sign=='+') { st.push(num); }
    
        // 如果數字前面的 operator 是 *，就把資料結構中最晚放進去的數字拿出來
        // 跟現在的數字相乘完，再放進資料結構
        if(sign=='*') { int t = st.top()*num; st.pop(); st.push(t); }
        
        sign = s[i];
        num = 0;
      }
    }

    // 處理完字串，加總資料結構中所有數字
    int res = 0;

    while (!st.empty()) {
      res += st.top();
      st.pop();
    }
    return res;
  }
};
```

如果有興趣探討更多 calculator 相關問題的話，可以把 Leetcode 上的 Basic Calculator 系列都做一做，然後可以參考這篇討論 - [Development of a generic solution for the series of the calculator problems](https://leetcode.com/problems/basic-calculator-iii/discuss/113592/Development-of-a-generic-solution-for-the-series-of-the-calculator-problems)。

## 題目二 - 馬叫聲的錄音

### 題目敘述

```
There are a lot of horses in the yard,
and we want to count how many there are.

Unfortunately, we've only got a recording of the sounds from the yard.
All the horses say "neigh".

The problem is they can "neigh" many times.
The recording from the yard is sadly all mixed together.
So, we need to figure out from the overlapping sounds how many horses there could be.

For example, we've got two horses in the yard, and we hear this "neigneighh".
From this recording, we can successfully deduce there are 2 horses.

Another example is "neighneigh".
From this example, we can only tell there is one horse in the yard.

As an additional complexity, our recording might not be perfect.
If it's a bad recording, we should give "Invalid" as the response.

The input will be given as a string on one line.
The output should be printed on it's own line.

Sample Input
nenigehnieighgh

Sample Output
2
```

### 題意理解與連結

這題主要就是要看目前錄到的字串中，到底可能同時有幾匹馬在叫。而我們要知道同時有幾匹馬在叫，就看到底有幾個 "neigh" 同時在進行，比如下例就是有 5 匹馬同時在叫：

"nnnnneeeeeiiiiiggggghhhhh"。

### 解法一 - 簡單暴力法

看完上面的題目敘述，最直覺的想法應該是可以看現在有幾個 n，知道已經有幾匹馬叫了，然後看到 h，就知道有一匹馬叫完了，而我們只要計算整個 record 中最多同時有幾個還沒被 h 結束的 n，就知道最多同時有幾匹馬在叫了。

這個想法要實作也很簡單，程式碼如下：

```cpp
#include <iostream>
#include <string>

using namespace std;

int parse_record(string s) {
  int max = 0, cur = 0;

  for(int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == 'n') { cur++; }
    if (c == 'h') { cur--; }
    if (cur > max) { max = cur; }
  }
  
  return max;
}

int main() {
  string s = "nenigehnieighgh";
  
  int horses = parse_record(s);
  
  cout << "There are at least " << horses << " horses in the yard.\n";
  return 0;
}
```

不過題目如果這麼簡單，那就太好了XD

大家如果記得題目，裡面有說

```
As an additional complexity, our recording might not be perfect.
If it's a bad recording, we should give "Invalid" as the response.
```

但我們的解法很明顯不可能產生 invalid，因為我們假設了每一個 n 一定都會有 一個 h 結尾。於是我們要來探討解答二。

## 解法二 - Depth First Search

既然剛剛的解法一不適用於不 valid 的情況，那我們就要來想想有哪些是需要考慮的 invalid case：

1. neigh 這 5 個 char 在整個字串中出現的次數不一致 (如果只有這種 invalid case，那可以先走過一次字串確認出現次數都一樣，再用解法一)
2. neigh 這 5 個 char 在整個字串中出現的次數正確，但順序不對 (例如可能出現 eighn，雖然每個 char 出現的次數都對，但內容錯了)

因為有這兩種 invalid case，所以我們不能只是去算次數，而是得去追蹤到底每一個字母對應到哪匹馬，從而確認到底是不是 valid，並且在過程中持續記錄目前同時有幾匹馬在叫。

在下面的實作當中，我們用 expect 來記錄目前有在追蹤的馬叫到哪一個字母，然後在遇到錄音裡面新的字母時，我們就可以根據 expect 裡面記錄的字母來判斷新的字母可以是分配給哪一匹馬。

實作如下：

```cpp
#include <iostream>
#include <string>

using namespace std;

char next(char c) {
  char n;
  
  switch(c) {
    case 'n' :
      n = 'e';
      break;
    case 'e' :
      n = 'i';
      break;
    case 'i' :
      n = 'g';
      break;
    case 'g' :
      n = 'h';
      break;
    case 'h' :
      n = '#';
      break;
  }
  return n;
}

void helper(string s, int pos, string expect, int local_max, int &max_horses) {
  if (pos == s.length() && expect.length() == 0 && local_max > max_horses) {
    max_horses = local_max;
    return;
  }
  if (pos >= s.length()) {
    return;
  }
  
  char c = s[pos];
  if(c == 'n') {
    string new_expect = expect + c;
    int new_local_max = local_max;
    if(new_expect.length() > local_max) {
      new_local_max = new_expect.length();
    }
    helper(s, pos+1, new_expect, new_local_max, max_horses);
  }
  else {
    for(int i=0; i<expect.length(); i++) {
      char val = expect[i];
      if (c == next(val)) {
        if(c == 'h') {
          string new_expect = "";
          for(int j=0; j<expect.length(); j++) {
            if(j == i) continue;
            new_expect += expect[j];
          }
          helper(s, pos+1, new_expect, local_max, max_horses);
        } else {
          expect[i] = next(val);
          helper(s, pos+1, expect, local_max, max_horses);
          expect[i] = val;
        }
      }
    }
  }
}

int main() {
  string s = "neigneighh";

  int max_horses = 0;
  helper(s, 0, "", 0, max_horses);

  if(max_horses == 0) cout << "Invalid record.\n";
  else cout << "There are at least " << max_horses << " horses in the yard.\n";

  return 0;
}
```

## 總結

今天跟大家介紹了 ZOOX 喜歡出的 OA 題目跟解答，雖然分享了這篇文章，但我想傳達的概念並不是 "面試前，要把這公司所有考古題都練習過"，而只是單純地好奇 ZOOX 會出什麼樣的題目，這些題目怎麼用基礎的演算法 & 資料結構能力來處理。

關於準備面試的 mindset，我推薦可以看看 [我是如何拿到矽谷頂級科技公司的10 個 offer 的](https://www.1point3acres.com/bbs/thread-526023-7-1.html)，裡面講到把 Leetcode 前 150 題融會貫通就能應付無窮題目變化的觀念，非常值得體會跟實作。

## 延伸閱讀

1. [[面試經驗] 自動駕駛公司 Aurora Onsite](https://www.1point3acres.com/bbs/thread-460462-1-1.html)
2. [[面試經驗] DeepScale 自動駕駛公司 OA ](https://www.1point3acres.com/bbs/thread-297228-1-1.html)
3. [Top Self Driving Car Startups](https://angel.co/newsletters/top-self-driving-car-startups-022318)
4. [Zoox Can Cruise San Francisco Without Drivers. Now It Needs Money](https://www.bloomberg.com/news/articles/2019-03-18/zoox-can-cruise-san-francisco-without-drivers-now-it-needs-money)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
