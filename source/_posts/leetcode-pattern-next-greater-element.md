---
title: Leetcode 刷題 pattern - Next Greater Element
date: 2019-10-26 16:51:34
tags:
- Algorithm
- Leetcode
- Software Engineer
author: pojenlai
---

## 前言

身在大 CS 時代，有越來越多人投入刷題的行列，在眼花撩亂的題海中，要想有效率地刷題，掌握並通達題目解法背後，可以不斷被拿來使用的觀念，才能做到以簡禦繁。

繼之前寫過的 [Two Pointer](https://blog.techbridge.cc/2019/08/30/leetcode-pattern-two-pointer/) 跟 [Sliding Window](https://blog.techbridge.cc/2019/09/28/leetcode-pattern-sliding-window/)，今天要來跟大家介紹另一種演算法的 pattern - Next Greater Element。

## Next Greater Element 的第一個範例 - Leetcode #496 - Next Greater Element I

### 題目

我們先看一下題目的敘述：

![img](https://i.imgur.com/gS5iiZr.png)

### 暴力法

一開始最直覺的想法就是先走過 nums1 的每一個 element，然後對每個 element 都去 nums2 搜尋位置，並找到 next greater element，時間複雜度是 O(mn)，空間複雜度是 O(1)。

### Stack 解法

上面的暴力法雖然可以解決，但這題其實可以用更好的時間效率 - O(m+n) 的方式解掉，主要的方法就在於搜尋 nums2 next greater element 的方法，舉例來說：

```
nums1 = [1,2]
nums2 = [6,2,1,5,4]
```

我們可以看到，5 同時是 2 跟 1 的 next greater element，所以如果用上面的暴力法，我們會

1. 先在 nums2 中找到 1，然後往後搜尋，找到 5。
2. 在 nums2 中找到 2，往後搜尋，找到 5。

當你仔細觀察這兩步，會發現有些冗餘的地方，因為在 nums2 中，1 在 2 後面，但卻是比 2 小的，所以其實在遇到 5 時再一次記錄 5 是 1 跟 2 的 next greater element 就好。

而要做到這種處理，就很容易聯想到 stack，因為 stack 可以依序儲存看到的 element，然後從最新放入的開始處理。

所以，基本概念就是，只要還沒遇到更大的 element，我們就把它放進 stack，直到遇到比 stack.top() 更大的，就可以把 stack 依序 pop 出來，並更新答案。

如果已經走完 nums2，但還留在 stack 中，我們就知道這些 element 沒有 next greater element。

另外因為要 output 的答案是要 follow nums1 的 index，所以我們需要先用一個 Hash Table 來儲存 nums1 的 `<value, idx>` pair。

我們來看一下 C++ 的實作：

```cpp
class Solution {
public:
    vector<int> nextGreaterElement(vector<int>& nums1, vector<int>& nums2) {
        // Use a hash table to store the <value, idx>
        unordered_map<int, int> table;
        for(int i = 0; i < nums1.size(); ++i) {
            table[ nums1[i] ] = i;
        }
        
        // Use a stack to get next greater element efficiently
        vector<int> ans(nums1.size(), -1);
        stack<int> st;
        for(int i = 0; i < nums2.size(); ++i) {
            while( !st.empty() and st.top() < nums2[i] ) {
                int cur = st.top();
                st.pop();
                
                // If cur exists in table, it means cur is in nums1
                // so we should update nums2[i] as the next greater element of cur
                if(table.count(cur)) {
                    ans[ table[cur] ] = nums2[i];
                }
            }
            
            st.push(nums2[i]);
        }
        
        return ans;
    }
};
```

這題也算是 stack 的一個實用案例，讀者可以再稍稍體會一下如果沒有 stack 這種操作資料的模式，處理起來是不是會比較麻煩。

## Next Greater Element 的第二個範例 - Leetcode #503 - Next Greater Element II

### 題目

![img](https://i.imgur.com/zcARwRb.png)

這題跟上一題有兩個差異：

1. nums 可以有 duplicate number
2. 這題的 array 是 circular 的

### Stack 解法

既然已經學會了剛剛的 stack 解法，我們就不看暴力法了，直接來修改 stack 解法。

首先，因爲這題會有 duplicate number，所以我們不能再用 Hash Table 來儲存 value 跟 index pair，而是要把 idx 資訊也直接放到 stack 中，這邊我們只要在 stack 裡面存一個 pair 就好。

再來要處理的東西就是 circular，不過這也不會很麻煩，最多其實只需要走 nums 兩次，就可以處理完畢。

程式碼實作如下：

```cpp
class Solution {
public:
    vector<int> nextGreaterElements(vector<int>& nums) {
        // Use a stack to get next greater element efficiently
        vector<int> ans(nums.size(), -1);
        stack< pair<int, int> > st; // store <value, index> to deal with duplicate values
        
        // Go through array twice to handle circular property
        for(int i = 0; i < 2 * nums.size(); ++i) {
            int idx = i % nums.size();
            while( !st.empty() and st.top().first < nums[idx] ) {
                pair<int, int> cur = st.top();
                st.pop();
                
                // Because we go through nums twice
                // we might update some ans twice (which we do not desire)
                // so we only update if ans[cur.second] == -1
                if(ans[cur.second] == -1) {
                    ans[cur.second] = nums[idx];
                }
            }
            
            if(ans[idx] == -1) {
                st.push( make_pair(nums[idx], idx) );
            }
        }
        
        return ans;
    }
};
```

## Next Greater Element 的第三個範例 - Leetcode #739 - Daily Temperatures

### 題目

![img](https://i.imgur.com/xpiHka4.png)

這題要求的有一點點不一樣，是要求 next greater element 跟目標 element 的 index 差值。

### Stack 解法

這題的解法跟第一題其實差不多，只差在更新 ans 的時候不是存進 temperature，而是 index 的差值，算是一個小小的變形，可以再幫助大家熟悉一下這個 pattern。

```cpp
class Solution {
public:
    vector<int> dailyTemperatures(vector<int>& T) {
        // Use stack to store <temperature, idx> pair for O(n) calculation
        vector<int> ans(T.size(), 0);
        stack< pair<int, int> > st; // store <temperature, index>
        
        // Go through T to search next greater element(temperature)
        for(int i = 0; i < T.size(); ++i) {
            while(!st.empty() and st.top().first < T[i]) {
                pair<int, int> cur = st.top();
                st.pop();
                
                // i is the next greater temperature of cur
                ans[cur.second] = i - cur.second;
            }
            
            st.push(make_pair(T[i], i));
        }
        
        return ans;
    }
};
```

## 總結

希望大家看完之後，可以感受到用 stack 的方便和效率。體驗到這個演算法好用、厲害，才會在該用的時候，自然而然地使用，比起用背的（例如看到...，就要用...），我覺得去體驗通達各種解法，覺得酷到不自覺笑出來、感受到讚讚讚，可能就是讓演算法功力進到下一個境界的現象。

上面提供的三題是讓大家初步體會一下 stack 的威力，如果你對這個 pattern 有興趣，可以再去看看延伸閱讀的好 stack 題列表。裡面有不少 stack 的題目，如果把這些題目一次寫完，對於 stack 的掌握度應該就大大提升了！

## 延伸閱讀

1. [lee215 大大分享的好 stack 題列表](https://leetcode.com/problems/next-greater-element-ii/discuss/98270/JavaC%2B%2BPython-Loop-Twice)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習 [用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟 [不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。

