---
title: Leetcode 刷題 pattern - Sliding Window
date: 2019-09-28 16:51:34
tags:
- Algorithm
- Leetcode
- Software Engineer
author: pojenlai
---

## 前言

身在大 CS 時代，有越來越多人投入刷題的行列，在眼花撩亂的題海中，要想有效率地刷題，掌握並通達題目解法背後，可以不斷被拿來使用的觀念，才能做到以簡禦繁。

繼上次的 Two Pointer，今天要來跟大家介紹另一種演算法的 pattern - Sliding Window。

## Sliding Window 的第一個範例 - Leetcode #209 - Minimum Size Subarray Sum

### 題目

我們先看一下題目的敘述：

![img](https://i.imgur.com/hyxLArY.png)

這題是要找到一個最小的 subarray，而且這個 subarray 的 element 總和必須要 >= s。

### 暴力法

最直覺的方法當然就是暴力法啦，我們可以列舉出所有可能的 subarray，檢查每個 subarray 的總和是否 >= s，如果 >= s，再跟已經出現過滿足條件最小的 subarray 比大小，如果更小，那就可以更新最小值。

假設 input 的 nums 裡面有 n 個 element，這樣做的時間複雜度是 $O(n^3)$，因為除了要花 $O(n^2)$ 的時間列舉所有 subarray，還得重複計算每個 subarray 的 sum。

### Sliding Window 解法

暴力法雖然簡單，可是真的太慢了！如果我們仔細觀察暴力法的過程，就會發現有很多 subarray sum 是重複計算的！我們看看下面這個例子：

```
nums = [2,3,1,2,4,3]
暴力法列舉出的 subarrays =
[2]
[2,3]
[2,3,1]
[2,3,1,2]
[2,3,1,2,4]
[2,3,1,2,4,3]
 
[3]
[3,1]
[3,1,2]
[3,1,2,4]
[3,1,2,4,3]
 
...
...
```

不難發現，其實以 2 為開頭的 subarray 跟以 3 為開頭的 subarray 其實只差在開頭有沒有那個 2，後面的數值應該要可以重複利用！

所以我們可以用 windowStart 跟 windowEnd 兩個指向 subarray 邊界的指針，來控制我們現在要看的 subarray。演算法就是要先一直擴張 windowEnd，如果發現 windowSum 已經比 s 大，那就開始縮減 window（也就是把 windowStart 往右移），直到走到 nums 的尾巴。實做出來的程式碼如下：

```cpp
class Solution {
public:
  int minSubArrayLen(int s, vector<int>& nums) {
    int windowSum = 0, windowStart = 0;
    int minWindowSize = numeric_limits<int>::max();

    for(int windowEnd = 0; windowEnd < nums.size(); windowEnd++) {
      windowSum += nums[windowEnd];
      
      // 如果 subarray sum >= s，那就開始縮減 subarray
      while(windowSum >= s) {
        minWindowSize = min(minWindowSize, windowEnd-windowStart+1);
        windowSum -= nums[windowStart];
        windowStart++;
      }
    }
 
    return minWindowSize == numeric_limits<int>::max() ? 0 : minWindowSize;
  }
};
```

使用這個方法，就完全去除掉冗餘的計算，讓時間複雜度下降到 $O(n)$！剛開始學到這個演算法的時候會懷疑這樣真的能走過所有可能的 subarray 嗎？

我覺得大家可以透過這個例子觀察：

```
nums = [2,3,1,2,4,3], s = 7
暴力法列舉出的 subarrays =
[2]
[2,3]
[2,3,1]
[2,3,1,2] // WindowStart 會開始往右移
[2,3,1,2,4]
[2,3,1,2,4,3]
 
...
```

一開始 windowStart 指向 2，然後 windowEnd 會慢慢擴張，當擴張到 [2,3,1,2] 這個情況時，因為 sum 已經 >= 7，所以 windowStart 會開始右移。也就是說，原本暴力法會考慮的 [2,3,1,2,4] 跟 [2,3,1,2,4,3] 就不會被考慮到。

就是因為這種情況，直觀下會覺得**我們這樣不就少考慮到很多情況嗎**？

但大家可以再仔細想想，我們現在要求的是 sum >= s 的最小 subarray，如果 [2,3,1,2] 已經滿足條件了，我們繼續看 [2,3,1,2,4] 跟 [2,3,1,2,4,3] 又有什麼意義呢？畢竟這兩個 subarray 都大於 [2,3,1,2] 啊！

只要把這點想通了，就不會再有用 Sliding Window 沒有考慮到所有 case 的這種讓心裡隱約覺得不對的想法！接著讓我們繼續看下去，更加熟悉 Sliding Window 可以應用的場景。

## Sliding Window 的第二個範例 - Leetcode # 340 - Longest Substring with At Most K Distinct Characters

### 題目

我們來看一下題目：

![img](https://i.imgur.com/RW0eDUR.png)

我們要找的是最多有 K 個不同 character 的最長 substring。注意，這題是 Hard 題，但寫完會覺得沒那麼 Hard。

### 暴力法

這題的暴力法應該不難想到，我們可以列舉出所有的 substring，一一檢查每個 substring 是否只有 <= K 的 distinct characters。時間複雜度一樣是 $O(n^3)$。

### Sliding Window 解法

跟上面那題很像，暴力法冗餘之處在於重複檢查 characters 是否 distinct。所以我們可以在擴張 substring 的時候，將 substring 裡面的 character 和出現次數存起來，利用 Hash Table 來記錄目前 substring 是否最多只有 K 個 distinct characters。

這邊之所以要用 Hash Table，而不是用 set ，是有原因的，大家可以先想一下，再往下看原因。


好！想完了嗎？答案是，因為要處理 substring 裡有 duplicate character 的情況，舉個例子，假設目前 substring 是：

```
a, c, a, b
```

假設把 windowStart 往右移，就會刪掉 windowStart 的 a，如果是用 set，這時就會以為 substring 裡沒有 a 了，但其實後面還是有個 a。所以若用 set，我們就會誤以為刪掉了 windowStart 的 a 之後，就沒有 a 了。

使用 Hash table 實作如下：

```cpp
class Solution {
public:
  int lengthOfLongestSubstringKDistinct(string s, int k) {
    int maxLength = 0, windowStart = 0;
    unordered_map<char, int> table;
 
    for(int windowEnd = 0; windowEnd < s.length(); windowEnd++) {
      table[s[windowEnd]] ++;
 
      // table.size() > k 表示有超過 k 個 distinct character
      while(table.size() > k) {
        if(--table[s[windowStart]] == 0)
          table.erase(s[windowStart]);
          
        windowStart++;
      }
 
      // 經過上面的 while 迴圈處理，這時 window 必定滿足條件
      maxLength = max(maxLength, windowEnd-windowStart+1);
    }
 
    return maxLength;
  }
};
```

程式碼是不是很簡潔呢？這可是一道 Hard 題，如果對 Sliding Window 不夠了解，或是無法靈活地跟 Hash Table 合併使用（Combo 技！），這題可是沒那麼簡單喔。

## Sliding Window 的第三個範例 - Leetcode #3 - Longest Substring Without Repeating Characters

### 題目

我們先看一下題目的敘述：

![img](https://i.imgur.com/kdtv0Rw.png)

### 暴力法

暴力法我就不贅述了，一樣也是列舉出所有的 substring，然後檢查 substring 有沒有 repeating character，最後就能找到 longest substring without repeating characters。

### Sliding Window 解法

基本上，Sliding Window 的寫法跟前面很像，都是需要設置 windowStart 跟 windowEnd，但不一樣的地方在於，我們得先確定 windowEnd 的 char 不在 substring 中，才能擴張 window。

實際做法上，我們可以用一個 set 來儲存目前 window 裡面有的 char，然後每次都要確定 window 裡已經沒有重複的 char，才會繼續擴張 window。實作如下：

```cpp
class Solution {
public:
  int lengthOfLongestSubstring(string s) {
    int n = s.length();
    set<char> st;
    int maxLen = 0, windowStart = 0, windowEnd = 0;
 
    while(windowEnd < n) {
      if(st.find(s[windowEnd]) == st.end()) {
        st.insert(s[windowEnd]);
        maxLen = max(maxLen, windowEnd-windowStart+1);
        windowEnd++;
      }
      else {
        st.erase(st.find(s[windowStart]));
        windowStart++;
      }
    }
    
    return maxLen;
  }
};
```

實作起來是不是變得很簡單了呢？如果你有這種感覺，那恭喜你，你已經開始習慣 Sliding Window 的演算法運作了！

## 總結

希望大家看完之後，可以感受到 Sliding Window 的方便和效率。體驗到這個演算法好用、厲害，才會在該用的時候，自然而然地使用，比起用背的（例如看到...，就要用...），我覺得去體驗通達各種解法，覺得酷到不自覺笑出來、感受到讚讚讚，可能就是讓演算法功力進到下一個境界的現象。

上面提供的三題是讓大家初步體會一下 Sliding Window 的威力，而且可以初步掌握 Sliding Window 的模板要怎麼寫 - **設置 windowStart 跟 windowEnd，最外面的 for 迴圈每一輪都擴張 windowEnd，但是當某些條件滿足時，就要移動 windowStart 來縮減 window**。

如果你對這個 pattern 有興趣，可以再去看看延伸閱讀的筆記，裡面記錄了不少 Sliding Window 的題目，而且從簡單到越來越難，如果把這些題目一次寫完，對於 Sliding Window 的掌握度應該就大大提升了！

## 延伸閱讀

1. [我的 Leetcode 刷題筆記 - Sliding Window pattern](https://po-jen.gitbooks.io/coding-practice-advanced-topics/content/sliding-window.html)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
