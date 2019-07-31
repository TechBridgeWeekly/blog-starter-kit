---
title: Go Web 程式設計入門教學：基礎介紹與環境建置
date: 2017-09-09 00:23:23
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

![Go Web 程式設計入門教學](/img/kdchang/logo.png)

# 前言
[Go 程式語言](https://en.wikipedia.org/wiki/Go_(programming_language) 是 Google 推出的靜態程式語言，其特色在於核心語法和關鍵字非常精簡（全部只有 25 個關鍵字！）並擷取了靜態語言的效能和動態語言的開發效率的優點，具備垃圾回收、快速編譯等特性，且針對平行化程式設計在使用上非常方便。接下來的文章我們將透過 Golang Web 程式設計來學習 Go 這個程式語言。程式語言唯有動手作才能學的好，你可以參考本篇文章建置開發環境或是[使用線上開發環境](https://play.golang.org/)進行學習。

# Go 環境建置

安裝方式：
1. [官網套裝安裝](https://golang.org/)，線上也有官方提供的 [playground](https://play.golang.org/) 線上執行環境可以使用
2. 使用套件管理工具 homebrew（mac）、apt-get（linux） 等進行安裝

    例如，在 linux 上安裝：

    ```
    $ sudo apt-get install golang-go
    ```

    或是主動宣告版本

    ```
    $ sudo apt-get install golang-1.8-go
    ```

    在 mac 上安裝：

    ```
    $ brew install go
    ```

3. 使用 gcc、MinGW（windows）等編譯器編譯原始 Go 檔案

對於一般初學者來說，可以使用官網或是套件管理工具來安裝，可以留意`環境變數`是否有設定成功。

若有成功安裝，可以打開終端機執行 go version 會出現相關版本訊息，以下是 mac 的範例：

```
$ go version
go version go1.9 darwin/amd64
```

在 Go 中也提供了許多方便指令，方便我們編譯、測試和執行程式：

![Go Web 程式設計入門教學](/images/golang101/go-command.png)

編譯檔案
```
$ go build
```

執行單元測試（unit testing），Go 一開始就內建了測試的機制，執行 go test 的話，會自動讀取套件目錄中的 *_test.go 來進行編譯、測試
```
$ go test
```

讓程式可以格式化 formatting，符合 go 的 convention
```
$ go fmt
```

安裝套件
```
$ go get
```

靜態分析潛在 bug
```
$ go vet
```

可以快速 build 並執行程式
```
$ go run
```

展示 go 相關文件
```
$ godoc
```

重新命名變數和函式
```
$ gorename
```

產生程式碼
```
$ go generate
```

# 你的第一個 Go 程式

在 Go 中程式運行的入口是套件 main。在這個程式中使用並導入了套件 "fmt"，在 Go 程式中程式執行入口是 main function，若成功在終端機執行 `$ go run`，則會在終端機印出 `Hello, World!` 的 Go 程式，恭喜完成你的第一個 Go 程式了！

```go
// 宣告程式屬於哪個 package
package main

// 引入套件
import (
    "fmt"
)

// 程式執行入口
func main() {
    // 使用 fmt 套件印出字串 word，使用 := 簡化原本變數宣告 var word string = "Hello, World!"
    word := "Hello, World!"
    fmt.Println(word)
} 
```

另外一個範例是引入了 `math/rand` 套件產生隨機整數（由於環境中 seed 一樣所以會印出同樣值）

```go
package main

import (
  "fmt"
  "math/rand"
)

func main() {
	fmt.Println("My favorite number is", rand.Intn(10))
}
```

Go 有支援許多網路程式開發的套件，可以用很簡單的幾行就完成網路伺服器的建置：

```go
package main

import (
    "io"
    "net/http"
)

// 處理 request 和 response 的函式
func hello(w http.ResponseWriter, r *http.Request) {
    // 印出 hello world
    io.WriteString(w, "Hello world!")
}

func main() {
    // router 讓網址可以對應處理函式
    http.HandleFunc("/", hello)
    // 監聽 8000 port
    http.ListenAndServe(":8000", nil)
}
```

# 總結
以上就是關於 Go 程式語言的基本教學介紹和環境建置，接下來的文章我們將透過 Golang Web 程式設計來學習 Go 這個程式語言的方方面面。

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
