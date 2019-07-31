---
title: C++ Header Guard 簡介
date: 2017-09-30 22:03:25
tags:
    - C++
    - header file
    - header guard
author: pojenlai
---

## 前言

C++ 的 header guard (中文譯作表頭哨兵，聽起來超帥) 是在開源/大型專案中常常用到的功能，所以如果想要踏入開源專案的世界，看到 header guard 就覺得親切熟悉，絕對可以幫助你更把心思放在體會到這些開源程式碼的美！

而且這個例子正巧是有朋友去面試美國某間新創公司時，實際被問到的問題，所以才興起想要動筆寫下這篇文章的念頭。畢竟很多小地方有沒有真正弄通是技術實力紮不紮實的重要指標！

我剛剛隨便選了一個 C++ 的開源專案 \(Point Cloud Library\) 的 [header 檔](https://github.com/PointCloudLibrary/pcl/blob/master/people/include/pcl/people/person_classifier.h)，裡面就有用到

```cpp
#ifndef PCL_PEOPLE_PERSON_CLASSIFIER_H_
#define PCL_PEOPLE_PERSON_CLASSIFIER_H_

#include <pcl/people/person_cluster.h>
#include <pcl/people/hog.h>

...

#include <pcl/people/impl/person_classifier.hpp>
#endif /* PCL_PEOPLE_PERSON_CLASSIFIER_H_ */
```

## 預處理器簡介

首先我們需要對 header file 的使用有個初步的認識，這就不能不介紹一下預處理器。我們都知道，要使用 header file 就是要在程式碼中 `#include` 它，而 `#include` 就是預處理器的一部分。

預處理器存在的目的是為了處理程式碼內的文本，例如 header file 的內容，之所以稱做 "預" 處理器是因為他在編譯氣執行之前就會先做事了。以 header file 的例子來說，預處理器會以 header file 的內容取代 `#include` 那一行。

## 那為何需要 header guard

原因是，header file 也常常需要 `#include` 其他 header file，例如寫程式時可能會寫到如下的程式碼：

main.cpp:
```cpp
#include <string>
#include "myHeader1.h"
#include "myHeader2.h"
...
```

myHeader1.h:
```cpp
#include <string>
...
```

如果在 myHeader1.h 裡面也需要用到 string，那可想而知經過預處理器的處理後，main.cpp 裡面就會有兩次 `#include <string>`，但這種情況根本超級常見啊，我們總不可能都去檢查過 myHeader1.h、myHeader2.h 跟他們 include 的所有 header file 再決定自己要不要 `#include <string>` 吧。所以就要有個機制，讓 header file 即便被 include 多次，也不會有其內定義的 classes 和 objects 被多次定義。而通常，我們就是用 header guard 來做這件事。

## header guard 怎麼運作

這時候就讓我們再回過頭來看看前言提到的例子：

```cpp
#ifndef PCL_PEOPLE_PERSON_CLASSIFIER_H_
#define PCL_PEOPLE_PERSON_CLASSIFIER_H_

#include <pcl/people/person_cluster.h>
#include <pcl/people/hog.h>

...

#include <pcl/people/impl/person_classifier.hpp>
#endif /* PCL_PEOPLE_PERSON_CLASSIFIER_H_ */
```

這邊使用到的是名為預處理器變數（preprocessor variables）的東西，在此例中為 `PCL_PEOPLE_PERSON_CLASSIFIER_H_`，而預處理器變數只有兩種狀態；已定義和未定義。所以上面的寫法就是用 `#ifndef` 來判斷是否已經定義過 `PCL_PEOPLE_PERSON_CLASSIFIER_H_`，如果沒有，那就先定義 `PCL_PEOPLE_PERSON_CLASSIFIER_H_`，然後才 `#include` 後面的東西。

看到這邊讀者們應該不難推敲，只要在需要 `#include` 的地方都加上類似的預處理器變數檢查，那就只有在預處理器變數還沒被定義時才會 `#include`，一旦被定義過了，就不會再重複定義了。

再回過頭來，如果你去看 [string.h 的內容](http://www.nongnu.org/avr-libc/user-manual/string_8h_source.html)，你就會看到：

```cpp
#ifndef _STRING_H_
#define _STRING_H_ 1
...
```

這樣一切就都串起來了！

## 總結

以前在學生時代學習程式語言的時候，常常會有學了某個東西不知道好用在哪裡的感覺，header guard 就是一個好例子。透過開源專案，我們很容易欣賞到某個功能的美，也希望大家看完這篇之後，每次看到 header guard 都能感覺世界真美好 XD

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人跟電腦視覺有>少許研究，最近在學習[看清事物的本質與改進自己的觀念](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)

