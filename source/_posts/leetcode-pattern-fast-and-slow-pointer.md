---
title: Leetcode 刷題 pattern - Fast & Slow Pointer
date: 2019-11-22 21:51:34
tags:
- Algorithm
- Leetcode
- Software Engineer
author: pojenlai
---

## 前言

身在大 CS 時代，有越來越多人投入刷題的行列，在眼花撩亂的題海中，要想有效率地刷題，掌握並通達題目解法背後，可以不斷被拿來使用的觀念，才能做到以簡禦繁。之前 Huli 寫的 [程式解題新手入門注意事項](https://blog.techbridge.cc/2019/11/02/before-start-leetcode/) 也講得非常好，寫題目是為了學會解題的思考方法，確保自己掌握重要的資料結構跟演算法。這也是為什麼我想要寫這系列的文章，把多個散落在各處的題目銜接起來，以後看到相似的問題就可以舉一反三，而不是去背各題目的解法。

舉例來說，之前遇過一題電話面試，問到的題目是：

```
Input: vector<bool> holidays, int pto
holidays 表示平日或假日，例如 0000011 表示前面 5 天是平日，後面 2 天是假日。
pto 表示最多可以放幾天假。
Output: 計算在可以用完 pto 的情況下，最久可以放多長的假。

範例：holidays = {0,0,0,0,0,1,1}, pto = 2, output = 4 
     因為可以放 {0,0,0,1,1,1,1}
```

基本上因為之前有寫過 Sliding Window 的 pattern，所以這題很快就寫出來了，也順利進到下一關，所以大家不需要追求把題目都刷完，而是掌握好重要的基礎，接下來就是應用這些基礎就可以面對很多變化題（當然還是會有一些解法很巧妙的題目，但其實大部分公司不會硬出巧妙題）。上面這題還有一個 follow-up 問題，可以看我的 [面經分享](https://www.1point3acres.com/bbs/thread-558279-1-1.html)。

那繼之前寫過的 [Two Pointer](https://blog.techbridge.cc/2019/08/30/leetcode-pattern-two-pointer/)、[Sliding Window](https://blog.techbridge.cc/2019/09/28/leetcode-pattern-sliding-window/) 跟 [Next Greater Element](https://blog.techbridge.cc/2019/10/26/leetcode-pattern-next-greater-element/)，今天要來跟大家介紹另一種演算法的 pattern - Fast & Slow Pointer。

## Fast & Slow Pointer 的第一個範例 - Leetcode #876 - Middle of the Linked List

### 題目

![img](https://i.imgur.com/Yk5H50w.png)

### 暴力解法

這題非常簡單，我們只要先走過一次 linked list，計算整個 list 的長度 - len，然後接下來再重新走一次 len/2 的 list 就好。

```cpp
/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode(int x) : val(x), next(NULL) {}
 * };
 */
class Solution {
public:
    ListNode* middleNode(ListNode* head) {
        /// Calculate the length of list
        ListNode *ptr = head;
        int len = 0;
        
        while(ptr != nullptr) {
            ++len;
            ptr = ptr->next;
        }
        
        // Move ptr to the middle of list
        ptr = head;
        for(int i = 0; i < len/2; ++i) {
            ptr = ptr->next;
        }
        
        return ptr;
    }
};
```

### Fast & Slow Pointer 解法

雖然上面的解法 OK，但如果被問到只能走一次 list 就要解決，我們就需要使用 Fast & Slow Pointer 了。基本上呢，做法就是指定兩個 pointer - fast 跟 slow，一開始 slow 跟 fast 都指向 head，接下來，在 fast 走到 linked list 的底端前，fast 一次走兩步，slow 一次走一步，當 fast 走到底的時候，slow 就會在中間。

不過我們還需要注意一下，linked list 長度有 even 跟 odd 兩種，以下分別解析：

- Odd:
```
1->2->3->NULL
```
當 slow 走到 2，fast 已經走到 3，下一次不會再進 while 迴圈，這時 return slow 就好。
- Even:
```
1->2->3->4->NULL
```
當 slow 走到 2，fast 走到 3，下一次因為 fast->next->next 就是 NULL 了，fast 不能再走兩步，所以不會再進 while 迴圈，這時要 return slow->next。

實作出來的程式碼如下：

```cpp
/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode(int x) : val(x), next(NULL) {}
 * };
 */
class Solution {
public:
    ListNode* middleNode(ListNode* head) {
        ListNode *slow = head, *fast = head;
        
        while(fast->next != nullptr && fast->next->next != nullptr) {
            slow = slow->next;
            fast = fast->next->next;
        }
        
        return (fast->next == nullptr) ? slow : slow->next;
    }
};
```

## Fast & Slow Pointer 的第二個範例 - Leetcode #141 - Linked List Cycle

### 題目

剛剛那題算是非常簡單，只是想讓大家了解一下 Fast & Slow Pointer 的基本概念，我們接著看難一點點的題目：

![img](https://i.imgur.com/rLVfEd6.png)
![img](https://i.imgur.com/Hw8kywa.png)

### 暴力法

要解這個題目，最直覺的想法就是記錄每一個走過的 node。而要知道有沒有 cycle，只要繼續走下去，如果有看到記錄過的 node，就知道有 cycle；但如果走到底了都還沒遇到有記錄過的 node，那就知道沒有 cycle。

以實作上，我們可以用一個 set 記錄已經走過的 node，程式碼如下：

```cpp
/**
* Definition for singly-linked list.
* struct ListNode {
* int val;
* ListNode *next;
* ListNode(int x) : val(x), next(NULL) {}
* };
*/
class Solution {
public:
    bool hasCycle(ListNode *head) {
        unordered_set<ListNode*> nodeSeen;

        while(head != nullptr) {
            if(nodeSeen.count(head)) { 
                return true; 
            }
            else { 
                nodeSeen.insert(head); 
            }
            head = head->next;
        }
        
        return false;
    }
};
```

這個解法雖然正確，但因為需要記錄，所以要額外花 O(n) 的空間來儲存，所以我們可以使用今天要學的 pattern 來做到 O(1) 的空間複雜度。

### Fast & Slow Pointer 解法

這題要怎麼應用 Fast & Slow Pointer 就比較不直覺，接著讓我們一步步來分析。

首先，我們一樣先假設有兩個 pointer - fast 跟 slow，fast 跟 slow 一開始都指向 head，之後，fast 每次走兩步，slow 每次走一步。按照這個邏輯，我們可以分成兩種情況：

1. 沒有 cycle：如果沒有 cycle，fast 走到底的時候，slow 才走到一半，所以如果 fast 走到底，就知道沒有 cycle
2. 有 cycle：如果有 cycle，fast 會先進去 cycle，然後就一直在 cycle 裡面跑，而雖然慢了一些，但 slow 也會進到 cycle，跟 fast 一起在 cycle 裡面跑（腦海中怎麼突然出現天竺鼠跑步的畫面）。

上面的分析讓我們知道怎麼判斷沒有 cycle，可是如果有 cycle，那 slow 跟 fast 不就會無窮地跑下去？

答案是不會，因為只要有 cycle，slow 跟 fast 一定會相遇，原因是，當 fast 跟 slow 都在 cycle 裡面跑，fast 遲早會追到離 slow 一步或兩步。

1. 差一步時：fast 移兩步，slow 移一步，兩者相遇。
2. 差兩步時：fast 移兩步，slow 移一步，兩者只差一步，下一次就會相遇。

所以，只要有 cycle，slow 跟 fast 一定會相遇！

經過上面的分析，實作就變得相當簡單：

```cpp
/**
* Definition for singly-linked list.
* struct ListNode {
* int val;
* ListNode *next;
* ListNode(int x) : val(x), next(NULL) {}
* };
*/
class Solution {
public:
    bool hasCycle(ListNode *head) {
        ListNode *slow = head, *fast = head;
        
        while(fast != nullptr && fast->next != nullptr) {
            fast = fast->next->next;
            slow = slow->next;
            
            if(slow == fast) {
                return true;
            }
        }
        
        return false;
    }
};
```

很明顯，這個演算法不需要花額外的 O(n) 空間，在 memory 使用吃緊時就是一個好解法。

## Fast & Slow Pointer 的第三個範例 - Leetcode #143 - Reorder List

### 題目

![img](https://i.imgur.com/K6uBNQq.png)

### 暴力解法

暴力解法滿直觀的，就先把所有 node 存到一個 array 中，剩下就只是用兩個 pointer 分別從頭跟尾開始，更新這些 node 的 next 就好。實作如下：

```cpp
class Solution {
public:
    void reorderList(ListNode* head) {
        // Corner case
        if(!head || !head->next) return;
        
        // Store all nodes into a vector
        vector<ListNode*> vl;
        while (head) {
            vl.push_back(head);
            head = head->next;
        }

        // Reassign the next pointer
        int i = 0, j = vl.size() - 1;
        while ( i + 1 < j ) {
            vl[i]->next = vl[j];
            vl[j]->next = vl[i+1];
            ++i, --j;
        }

        vl[j]->next = nullptr;
    }
};
```

### Fast & Slow Pointer 解法

同樣的問題來了，我們能不能不花額外的空間來解決這題呢？

答案是可以！只要你會 Fast & Slow Pointer！因為如果我們先仔細觀察題目，就會發現這題是要把後半段的 list 先反轉，然後再跟前半段的 list 交錯。所以我們可以先用 Fast & Slow Pointer 找到 list 中點，把後半 list reverse，然後就依序連接前半跟後半的 node。於是空間複雜度就降到 O(1)。

```cpp
/**
* Definition for singly-linked list.
* struct ListNode {
* int val;
* ListNode *next;
* ListNode(int x) : val(x), next(NULL) {}
* };
*/
class Solution {
public:
    void reorderList(ListNode* head) {
        if(!head or !head->next) return;
        
        // Find the middle of list
        ListNode *slow = head, *fast = head;
        
        while(fast != NULL && fast->next != NULL) {
            slow = slow->next;
            fast = fast->next->next;
        }
        
        // Reverse the second half of the list
        ListNode *second = reverse(slow);
        ListNode *first = head;
        
        // Construct the new list by interleaving two lists
        while(first != nullptr && second != nullptr) {
            ListNode* tmp = first->next;
            first->next = second;
            first = tmp;
            tmp = second->next;
            second->next = first;
            second = tmp;
        }
        
        if(first != NULL)
            first->next = NULL;
    }
    
private:
    ListNode* reverse(ListNode* head) {
        ListNode* prev = nullptr;

        while(head != nullptr) {
            ListNode* next = head->next;
            head->next = prev;
            prev = head;
            head = next;
        }
        
        return prev;
    }
};
```

## Fast & Slow Pointer 的第四個範例 - Leetcode #457 - Circular Array Loop

### 題目

![img](https://i.imgur.com/vTgRLVP.png)
![img](https://i.imgur.com/D7zy1eD.png)

### Fast & Slow Pointer 解法

這題雖然不是 linked list 的題目，但其實有 cycle 的概念，就可以運用 Fast & Slow Pointer。直觀的做法就是依序把每個 element 當作起點，只要有一個起點滿足 circular 的條件，就可以 return true，反之如果沒有一個起點可以構成 circular，那就 return false。

因為另外要注意 cycle 必須有超過一個 element，還有必須一直同向兩個條件，所以不能只是去找有沒有 cycle，同時還得檢查這兩件事。不過基本的框架其實就是 Fast & Slow Pointer，大家可以看看下面的實作體會一下：

```cpp
class Solution {
public:
    bool circularArrayLoop(vector<int>& nums) {
        for (int i = 0; i < nums.size(); i++) {
            bool isForward = nums[i] >= 0;  // if we are moving forward or not
            int slow = i, fast = i;

            // If slow or fast becomes '-1' this means we can't find cycle for this number
            do {
                slow = findNextIndex(nums, isForward, slow);  // move one step for slow pointer
                fast = findNextIndex(nums, isForward, fast);  // move one step for fast pointer
                if (fast != -1) {
                    fast = findNextIndex(nums, isForward, fast);  // move another step for fast pointer
                }
            } while (slow != -1 && fast != -1 && slow != fast);

            if (slow != -1 && slow == fast) {
                return true;
            }
        }

        return false;
    }
    
private:
    int findNextIndex(const vector<int> &nums, bool isForward, int currentIndex) {
        bool direction = nums[currentIndex] >= 0;
        if (isForward != direction) {
          return -1;  // change in direction, return -1
        }

        // wrap around for negative numbers
        int nextIndex = (currentIndex + nums[currentIndex] + nums.size()) % nums.size();

        // one element cycle, return -1
        if (nextIndex == currentIndex) {
          nextIndex = -1;
        }

        return nextIndex;
    }
};
```

## 總結

今天跟大家介紹了一個新的 pattern - Fast & Slow Pointer，上面提供的四題是讓大家初步體驗一下，雖然這個 pattern 的變化不是很大，但在一些需要特別優化的地方還是派得上用場。

如果你對這個 pattern 有興趣，可以再去看看延伸閱讀的筆記。裡面有更多同樣 pattern 的題目，如果把這些題目一次寫完，之後看到類似的題目應該就比較容易想到這種解法！

## 延伸閱讀

1. [我的 Leetcode 刷題筆記 - Fast & Slow Pointer pattern](https://po-jen.gitbooks.io/coding-practice-advanced-topics/content/fast-and-slow-pointer.html)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習 [用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟 [不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
