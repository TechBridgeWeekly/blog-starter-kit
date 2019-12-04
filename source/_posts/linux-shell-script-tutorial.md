---
title: 簡明 Linux Shell Script 入門教學
date: 2019-11-15 10:23:23
author: kdchang
tags: 
    - linux
    - 指令
    - 命令列
    - command line
    - 軟體工程師
    - 軟體工程
    - software engineering
    - bash
    - shell
    - script
    - unix
    - mac
    - os
    - windows
    - curl
    - shell script
---

![簡明 Linux Shell Script 入門教學](/img/kdchang/shell-script/shell-script.png)

# 前言
Shell 是一種讓使用者可以和作業系統 Kernel（核心用來控制 CPU、記憶體、硬碟等硬體）互動溝通的橋樑。Shell Script 主要是使用在 Linux 和 MacOS 等 Unix-like 作業系統的自動化操作指令的程式語言。其透過 Unix shell 命令列直譯器來執行（我們這邊主要使用 bash shell，其他的 Unix shell 觀念大致類似），使用方式有點類似直譯式程式語言（不用編譯直接執行）。在 Windows 系列家族也有類似的使用方式：Batch file。

![簡明 Linux Shell Script 入門教學](/img/kdchang/shell-script/kernel.png)

一般情況 Shell Script 常用於系統管理、自動化操作檔案、自動化重複的指令碼、分析 log 等文件檔案、列印呈現我們想要的資料等，透過程式語言的使用來減少重複瑣碎的工作，所以若能妥善使用將提升不少開發者和軟體工程師的日常工作效率。接著我們將透過日常生活常用的使用情境，帶領讀者們進入入門 Shell Script（讀者需要具備基本 Linux 指令碼的基本觀念，若你需要複習常見 Linux 指令可以參考 [Linux Command 命令列指令與基本操作入門教學](https://blog.techbridge.cc/2017/12/23/linux-commnd-line-tutorial/)）。

# Shell Script 初體驗
在撰寫 Shell Script 之前我們先來了解 Shell Script 撰寫的流程和架構。一般我們會使用 `.sh` 副檔名來命名 Shell Script 檔案。然後將該檔案設定為可執行：

```
chmod +x demo.sh
```

可以透過檢視檔案詳細資料觀看是否已有 `+x` 的執行權限：

```
ls -l demo.sh

# -rwxr-xr-x  1 user  staff  106 Nov 16 10:41 demo.sh
```

執行 Shell Script 檔案：

```
./demo.sh
```

接著，我們先利用一個簡單的範例：將目前執行 process 的 PID 依照數字大小排序，取出前 10 名，來了解撰寫 Shell Script 的基本架構。

```
# 宣告使用 /bin/bash
#!/bin/bash

echo "=== 將目前執行 process 的 PID 依照數字大小排序，取出前 10 名 === "

# ps 為列出 process 相關資訊，透過 | pipe 管線傳遞資料。awk 可以根據 pattern 進行資料處理（這邊印出第一欄 PID）而 sort 是進行排序，其排序時，預設都是把資料當作字串來排序，若想讓資料根據實際數值的大小來排序，可以加上 -n 參數。-r 則是由大到小排序，預設是由小到大
ps | awk '{print $1}' | sort -rn | head -10
```

執行結果：

```
$ ./demo.sh
=== 將目前執行 process 的 PID 依照數字大小排序，取出前 10 名 ===
83784
83783
75956
75955
75954
75952
74069
74068
73543
37621
```

恭喜你，你已經完成了第一個 Shell Script 程式了！

# 變數
一般來說程式語言中變數是用來暫存接下來會使用到的資料或是儲存指到物件的參考位置。在 Shell Script 可以使用以下三種方式來宣告變數並給定值：

```
#!/bin/bash

variable1=value
# 若是值內有空白則需要使用 '' 或 "" 包裹起來
variable2='value 2'
variable3="value 3"
```

> 註解使用 #，因為沒有多行註解，所以需要使用多單行註解達到

使用變數方式為 `${變數名稱}`，花括號主要是輔助了解變數的範圍：

```
#!/bin/bash

pathName=demo.sh
# echo 是列印值，印出變數 pathName 內容 demo.sh
echo ${pathName}
```

更新變數直接重新 assign 值即可：

```
#!/bin/bash

pathName=demo1.sh
# 印出 demo1.sh
echo ${pathName}

pathName=demo2.sh
# 印出 demo2.sh
echo ${pathName}
```

刪除變數使用 `unset`：

```
#!/bin/bash

pathName=demo.sh
# 印出 demo.sh
echo ${pathName}
unset pathName
# 空值
echo ${pathName}
```

> 注意系統環境變數為全域變數、區域變數則為 Shell Script 內部程式使用，不能跨檔案使用。

# 運算式
運算式是當運算子和運算元計算結果回傳後賦值給變數。在 Bash Shell 中內建原生不支援運算式，但我們可以使用 expr、awk 等指令來支援實現運算式。

## 算式
我們可以使用四則運算來賦值：

```
#!/bin/bash

result=`expr 10 + 2`

# 12
echo "Result: $result"
```

# 條件判斷
在 Shell Script 中同樣可以使用 if..else 條件判斷，特別注意的是在 Shell Script 中使用 `fi` 為結尾（為 `if` 的倒寫法，同樣的接下來討論的 `case` 也有類似用法），代表條件判斷結束。`==` 為等於，`!=` 為不等於運算子。

## if

```
#!/bin/sh

x=20
y=30

if [ $x == $y ]; then
   echo "value x is equal to value y"
fi

if [ $x != $y ]; then
   echo "value x is not equal to value y"
fi
```

## if else
在 Shell Script 可以使用 `-gt` （greater than 縮寫）和 `-lt` （less than 縮寫）代表`大於`和`小於`，而 `-ge` （greater equal 縮寫）和 `-le`（less equal 縮寫）則是`大於等於`和`小於等於`的運算子符號。

> 記得比較條件需要放在 [] 中，前後要留空白

```
#!/bin/bash

if [ $x -gt $y ]; then
   echo "value x is greater than value y"
else
   echo "value x is not greater than value y"
fi

if [ $x -lt $y ]; then
   echo "value x is not less than value y"
else
   echo "value x is not less than value y"
fi

if [ $x -ge $y ]; then
   echo "value x is greater or equal than value y"
else
   echo "value x is not greater than value y"
fi

if [ $x -le $y ]; then
   echo "value x is not less or equal than value y"
else
   echo "value x is not less or equal than value y"
fi
```

## if elif else
若有多個條件需要判斷，可以使用 `if elif else`：

```
#!/bin/bash

value1=20
value2=30
value3=30

if [ $value1 -gt $value2 ]; then
   echo "value1 is greater than value2"
elif [ $value1 == $value3 ]; then
   echo "value1 is equal to value3"
else
   echo "other result"
fi
```

## case ... esac
若要使用類似一般程式語言的 `switch` 來處理多種條件判斷時，可以使用 `case` 來進行判斷：

```
#!/bin/bash

language='Java'

case $language in
    Java*) echo "是 Java！"
            ;;
    Python*) echo "是 Python！"
            ;;
    C*)     echo "是 C！"
            ;;
    *)      echo "沒 match 到！"
esac
```

# 迴圈
當我們需要重複某些瑣碎的任務或是迭代取得資料就需要迴圈來支援。此時可以使用 `for`、`while` 和 `until` 迴圈進行迭代。

## for
Shell Script 的 `for` 使用方法和一般程式語言類似，同樣可以針對條件使用 `break`、`continue` 來跳出或是跳過迴圈。

```
#!/bin/bash

for loop in 1 2 3; do
    echo "number: $loop"
done
```

## while
若是需要設定一個條件直到該條件為止，可以使用 `while`，但要注意避免無限迴圈狀況：

```
#!/bin/bash

counter=0
while [ $counter -le 5 ]; do
    counter='expr $counter+1'
    echo $counter
done
```

## until
直到某個條件結束可以使用 `until` 來進行：

```
#!/bin/bash

# 從 0 印出數字直到 10
counter=0
until [ $counter -gt 10 ]; do
   echo $counter
   counter=`expr $counter + 1`
done
```

# 函式
隨著我們的程式越來越大，我們需要透過模組化或是將重複使用的程式碼改成函式。函式基本架構如下：

1. 函式名稱（`function` 關鍵字為選擇性）
2. 是否有傳入參數
3. 函式內操作
4. 是否有回傳值

```
function function_name () {
    # 做一些事情
    [ 回傳值 ]
}
```

函式範例：

```
#!/bin/bash

function echoHello() {
    # hello world, rock!!
    echo "${0} hello ${1}, ${2}!!"
}

echoHello 'world' 'rock'
```

以上我們可以看到使用 `""` 雙引號把字串和變數取出來印出（你可以試試看使用單引號會發生什麼事情），與一般程式語言比較不同的是其函式呼叫不需要有小括號傳入參數，直接以空白當作參數傳遞的格式。注意參數從 1 開始，`${0}` 為檔名。

# 特殊變數
在 Shell Script 檔案和函式往往需要透過傳入參數來設定執行程式的內容。在 Shell Script 支援許多好用的特殊變數，可以方便我們透過使用變數方式來設置程式執行的流程。

| 指令 | 描述 | 註解 |
| -------- | -------- | -------- |
| $0     | 目前的檔案檔名     |      |
| $n     | n 從 1 開始，代表第幾個參數     |      |
| $#     | 傳遞到程式或函式目前有幾個參數     |      |
| $*     | 傳遞到程式或函式所有參數    |      |
| $@     | 類似 $* 但是在被雙引號包含時有些許不同   |      |
| $?     | 上一個指令退出狀態或是函式的返回值   |      |
| $$     | 目前 process PID   |      |

透過執行 `./demo_special_var.sh var1 var2`：

```
#!/bin/bash

echo "$0"
echo "$1"
echo "$#"
echo "$*"
echo "$@"
echo "$?"
echo "$$"
```

印出結果：

```
./demo.sh
var1
2
var1 var2
var1 var2
0
80417
```

# 總結
以上我們透過循序漸進入門了 Shell Script 並撰寫了我們第一個 Shell Script 程式，並了解如何在 Shell Script 中使用變數、條件判斷、迴圈、函式以及特殊變數。Shell Script 常用於系統管理、自動化操作檔案、自動化重複的指令碼、分析 log 等文件檔案、列印呈現我們想要的資料等，透過程式語言的使用來減少重複瑣碎的工作，快把 Shell Script 放入你的工具箱中吧！若能妥善使用將提升不少開發者和軟體工程師的日常工作效率。


# 參考文件
1. [Wiki Shell 指令碼](https://zh.wikipedia.org/zh-tw/Shell%E8%84%9A%E6%9C%AC)
2. [Wiki 核心 (電腦科學)](https://zh.wikipedia.org/zh-tw/%E5%86%85%E6%A0%B8)
3. [鳥哥的 Linux 私房菜](http://linux.vbird.org/)

（image via [stackoverflow](https://i.stack.imgur.com/jJgjc.png)）


關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 
