---
title: Go Web 程式設計入門教學：語法基礎之流程控制（flow control）篇
date: 2018-02-17 11:23:23
author: kdchang
tags:
    - GO
    - GO lang
    - golang
    - web
    - go web
    - 自學程式
    - 自學程式設計
    - 程式設計
    - 線上自學
    - coding
    - coder
    - programming
    - computer
    - computer science
    - code
    - 電腦科學
    - 學寫程式
    - 學程式
---

![Go Web 程式設計入門教學](/img/kdchang/golang101/logo.png)

# 前言
之前我們曾介紹了如何建置 Go 變數和資料型別的使用，接下來的單元我們將透過 Golang Web 程式設計來學習 Go 這個程式語言。而在這個單元中我們將介紹 Go 的流程控制的使用。程式語言唯有動手作才能學的好，你可以參考之前的文章建置開發環境或是[使用線上開發環境](https://play.golang.org/)進行學習。

# 流程控制簡介
一般而言，程式語言是由上往下執行，若我們需要程式依照我們設計的邏輯方向執行就需要流程控制的協助。在這邊主要談的流程控制有三部分：條件判斷、迴圈循環、跳躍控制。

我們使用制做菜脯蛋食譜步驟來講解範例：

1. 放點油
2. 打蛋
3. 如果喜歡蔥花可以加入蔥花，如果喜歡菜脯可以加入菜脯（程式術語：`if...else` 條件判斷）
4. 放入少許鹽巴
5. 中火快炒，翻五次面（程式術語：`for` 迴圈）
6. 當看到蛋面呈現金黃色時可以起鍋，結束料理（程式術語：`while` 迴圈）
7. 好吃的蔥花蛋或菜脯蛋上桌

首先介紹，條件判斷，如果 if...。條件判斷是一種邏輯判斷，當符合條件就怎麼樣，不符合條件就怎麼樣。

1. if...else

    ```go
    package main

    import "fmt"

    var taste = "菜脯蛋" // 喜歡菜脯蛋

    if taste == "菜脯蛋" {
        fmt.Println("加入菜脯")
    } else {
        fmt.Println("加入蔥花")
    }
    ```

    Go 語言中除了條件判斷不用加 `()` 外，也可以在條件判斷敘述宣告變數，但只會存活於條件邏輯區塊使用上。

    ```go
    package main

    import "fmt"

    if taste := "蔥花蛋"; taste == "菜脯蛋" {
        fmt.Println("加入菜脯")
    } else {
        fmt.Println("加入蔥花")
    }

    fmt.Println(taste) // 編譯錯誤，條件判斷敘述宣告的變數，只會存活於條件邏輯區塊使用上
    ```

2. goto
    Go 中還提供了 `goto` 語法，讓你可以直接跳躍到某定義區塊，但建議小心使用。

    ```go
    package main

    import "fmt"

    func main() {
        num := 91

        if num >= 60 {
            fmt.Println("hi true")
            goto PASS
        } else {
            fmt.Println("hi false")
        }
        PASS:
            fmt.Println("hi pass")
    }
    ```

3. for 迴圈
    在 go 中 for 迴圈的功能強大，既可以被當做循環讀取資料，又可以當做 while 來控制邏輯，還能反覆操作運算，以下介紹 for 語法：

    ```go
    for 變數宣告或函式回傳值; 條件判斷; 每輪結束時操作 {
        // ...
    }
    ```

    我們接著使用食譜範例，中火快炒，翻五次面：

    ```go
    package main

    import "fmt"

    func main() {
        for index := 0; index < 5; index++ {
            fmt.Println("翻面", index + 1, "次")        
        }
    }
    ```

    當我們不知道具體要迴圈幾次時可以使用 `while`，但由於 Go 中沒有 `while` 關鍵字，當忽略了 `;`，就可以當 `while` 迴圈使用：

    ```go
    package main

    import "fmt"

    // 從 1 加到 10
    func main() {
        sum := 0
        i := 0
        for i <= 10 {
            sum += i
            i++
        }
        fmt.Println(sum)
    }
    ```

    使用 break 和 continue 可以跳出整個循環和跳過當下：

    ```go
    package main

    import "fmt"

    // 從 1 加到 10
    func main() {
        sum := 0
        i := 0
        for i <= 10 {
            if i == 3 {
                break; // continue 跳過 3，break 到 3 整個循環跳出
            }
            sum += i
            i++
        }
        fmt.Println(sum)
    }
    ```

    善用 `for` 也可以把 `slice` 和 `map` 資料一一取出：

    ```
    package main

    import "fmt"

    for key, value := range map { // 也可以忽略其中一值，for _, value := range map
        fmt.Println(key, value)
    }
    ```

4. switch
    當我們需要寫很多 `if...else` 時就是使用 `switch` 的好時機：

    ```
    package main

    import "fmt"

    func main() {
        i := 2
        switch i {
            case 1:
                fmt.Println("hello:", i) // go 中不用加 break 會自己跳出不會往下執行
            case 2:
                fmt.Println("hello:", i) // 加上 fallthrough 可以強制往下執行
                fallthrough
            default:
                fmt.Println("hello: default")        
        }	
    }
    ```

# 總結
以上就是關於 Go 程式語言的基本語法中流程控制介紹，接下來的文章我們將透過 Golang Web 程式設計來學習 Go 這個程式語言的方方面面。

# 參考文件
1. [A tour of go](https://tour.golang.org/welcome/1)
2. [Go Tutorial](https://www.tutorialspoint.com/go/)
3. [Go by Example](https://gobyexample.com/)
4. [Go Programming Language: An Introductory Tutorial](https://www.toptal.com/go/go-programming-a-step-by-step-introductory-tutorial)
5. [Go tour Exercise](https://github.com/davidhoo/go-tour)
6. [Ubuntu Go install](https://github.com/golang/go/wiki/Ubuntu)
7. [從Go看現代程式語言](http://www.ithome.com.tw/voice/99698)

（image via [cuelogic](http://www.cuelogic.com/blog/wp-content/uploads/2017/06/go_lang1.png)）

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
