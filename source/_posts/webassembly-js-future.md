---
title: 了解 WebAssembly 的基礎使用方法
date: 2017-06-17 14:02:44
tags: Web, WASM, JavaScript
---

## 前言

時間過得很快，記得第一次聽到 WebAssembly 這個名詞是在 2015 年，小弟還在服役...當時看到的文章以聳動的標題訴說著 JavaScript 即將要被取代，各家大廠紛紛投入開發...（我就不轉貼這種文章了）害我想說是不是退役後會找不到工作...

還好記者說的總是不一定對，WebAssembly 當然不是來取代 JavaScript 的，可以看看 [JavaScript 的發明人 Brendan Eich 怎麼說](https://brendaneich.com/2015/06/from-asm-js-to-webassembly/) 以及 [他在 Fluent conference 的 keynote](https://www.youtube.com/watch?time_continue=1108&v=aZqhRICne_M)。

但即便知道 WebAssembly 並非要取代 JavaScript，我其實也還是一直搞不太懂身為開發者，到底要如何使用 WebAssembly，只知道它似乎讓 C/C++ 跑在 Browser 上這件事變成可行，也能大幅提升 JavaScript 的效能。

直到前陣子發現一部限時免費的教學影片 - [Get Started Using WebAssembly (wasm)](https://egghead.io/courses/get-started-using-webassembly-wasm)，我才稍稍領悟了一些。這部由 [Guy Bedford](https://egghead.io/instructors/guy-bedford) 製作的影片在 egghead.io 上，短短 56 分鐘，以實際範例告訴你如何使用 WebAssembly，以及與 JS 進行效能比較，也介紹了許多方便你測試的工具，有時間的話我強烈推薦把它看完，不過現在已經要是 egghead 的 pro member 才能看得到了... (作者有 open source 他所有的範例 code 在 [guybedford/wasm-intro](https://github.com/guybedford/wasm-intro) 與 [guybedford/wasm-demo](https://github.com/guybedford/wasm-demo))

可能你沒有時間也沒有多餘的錢能付費觀看，但沒關係，希望憑著我的記憶，透過這篇文章融合一些影片的重點，讓大家快速了解 WebAssembly 是什麼，以及要如何與 JavaScript 搭配使用。

## 什麼是 WebAssembly (wasm)？

> WebAssembly or wasm is a new, portable, size- and load-time-efficient format suitable for compilation to the web. -- [WebAssembly Design](https://github.com/WebAssembly/design)

1. 一種二進位表示的新語言，但有另外的 text format 可以讓你編輯與 debug。

2. Compile Target：顧名思義，只要透過特定的 Compiler，你就能將你自己慣用的語言編譯成 WebAssembly，然後執行在瀏覽器上！目前可以透過 [Emscripten(LLVM to JS compiler)](http://kripken.github.io/emscripten-site/index.html) 來編譯 C/C++ 的程式。

3. 提供增強 JavaScript 程式的方法：你可以將 performance critical 的程式部分用 WebAssembly 撰寫，或是用第 2 點提及的 C/C++編譯成 WebAssembly，然後像一般 import js module 一般，導入你的 JavaScript Application。透過 WebAssembly，你能夠自由控制 Memory 的存取與釋放。

4. 當 Browser 能夠支援運行 WebAssembly 的時候，由於二進位格式以及事先編譯與優化的關係，勢必能夠產生比 JavaScript 運行速度更快、檔案大小更小的結果。

5. 語言的安全性 WebAssembly 當然也很重視，在 JavaScript VM 中， WebAssembly 運行在一個沙箱化的執行環境，遷入 web 端運行時會強制使用 Browser 的 Same-Origin 和 permissions security policies。此外，wasm 的實作設計中更特別提及他是 [memory-safe](https://github.com/WebAssembly/design/blob/master/Security.md#memory-safety) 的。

6. [Non-Web Embeddings](https://github.com/WebAssembly/design/blob/master/NonWeb.md)：雖然是為了 Web 設計，但也希望能在其他環境中運行，因此底層實作並沒有 require Web API，讓其擁有良好的 portability，不管是 Nodejs, IoT devices 都可使用。

WebAssembly 目前由 [W3C Community Group](https://www.w3.org/community/webassembly/) 設計開發，成員包含所有 major browsers 的代表。

WebAssembly 有許多 [High-Level Goals](http://webassembly.org/docs/high-level-goals/)，目前 release 的版本主要為 [MVP(Minimum Viable Product)](http://webassembly.org/docs/mvp/)，提供先前 `asm.js` 的多數功能，並先以 C/C++ 的編譯為主。

## 等等，第一點就有問題了，你說他是二進位表示的語言，那該怎麼寫？！text format 又是長什麼樣子？

問得好，這就是本篇的重點，WebAssembly 的檔案格式為 `wasm`，舉一個例子來看，一個用 c++ 撰寫的加法函數：

```c add.c
#include <math.h>
int add(int num1, int num2) {
    return num1 + num2;
}
```


若編譯為 `wasm` 會長這個樣子（為節省空間我轉成 Hex）：

```wasm add.wasm
00 61 73 6d 01 00 00 00  01 87 80 80 80 00 01 60
02 7f 7f 01 7f 03 82 80  80 80 00 01 00 04 84 80
80 80 00 01 70 00 00 05  83 80 80 80 00 01 00 01
06 81 80 80 80 00 00 07  95 80 80 80 00 02 06 6d
65 6d 6f 72 79 02 00 08  5f 5a 33 61 64 64 69 69
00 00 0a 8d 80 80 80 00  01 87 80 80 80 00 00 20
01 20 00 6a 0b
```

當然我們很難去編輯這樣的東西，所以有另一種 `text format` 叫做 `wast`，上述的 .wasm 轉成 .wast 後：

```wast add.wast
(module
  (table 0 anyfunc)
  (memory $0 1)
  (export "memory" (memory $0))
  (export "add" (func $add))
  (func $add (param $0 i32) (param $1 i32) (result i32)
    (i32.add
      (get_local $1)
      (get_local $0)
    )
  )
)
```

這樣就好懂多了，我們一行一行來解釋：

`line 1` 的 module 就是 WebAssembly 中一個可載入、可執行的最小單位程式，在 runtime 載入後可以產生 Instance 來執行，而這個 module 也朝著與 ES6 modules 整合的方向，也就是說以後能透過 `<script src="abc.wasm" type="module" />` 的方式載入。

`line 2 ~ 3` 分別宣告了兩個預設的環境變量: `memory` 與 `table`，memory 就是存放變數的記憶體物件，而 table 則是 WebAssembly 用來存放 function reference 的地方，在目前 MVP 的版本中，table 的 element type 只能為 `anyfunc`。

接著 `line 4 ~ 5` 把 memory 與 add function export 出去。之後在 JavaScript 中，我們可以取得這兩個被 export 出來的物件與函式。

最後是加法函式的宣告與實作內容，其中 `get_local` 是 WebAssembly 中取得 memory 中 local 變數的方法。

不知道會不會有人好奇 i32 是什麼？

i32 指的就是 32位元的整數，在 WebAssembly 的世界中，是強型態的，必須明確指定變數型態，寫習慣 JS 的要多加注意。

<!-- ## 工具包 -->
## 那到底怎麼將 C/C++ 編譯成 wasm 或 wast 呢？

[WebAssembly.org](http://webassembly.org/getting-started/developers-guide/) 中介紹我們使用 [Emscripten](https://github.com/kripken/emscripten)，Emscripten 的安裝與使用方法大家可以從官網上看到，就不贅述。

安裝好後執行 `emcc add.c -s WASM=1 -o add.html` 即可，唯一要注意的是 `WASM=1` 這個 flag 要設定，否則 `emcc` 預設會跑 asm.js。

如果只是想嚐鮮一下，可能看到要安裝這些東西就會把網頁關掉了...

不過不用擔心！現在也已經有很方便的 online tool 可以使用：

[WasmFiddle](https://wasdk.github.io/WasmFiddle)

![WasmFiddle](/img/arvinh/wasmFiddle.png)

<!-- Fiddle 的限制 -->
WasmFiddle 可以幫你把 C code 轉成 Wast 與 Wasm (可下載)，然後同時讓你直接利用 JS 進行操作，缺點是沒辦法直接更改 Wast。

[WasmExplorer](http://mbebenita.github.io/WasmExplorer/)：

![wasmExplorer](/img/arvinh/wasmExplorer.png)

<!-- Explorer 的限制 -->
WasmExplorer 一樣能幫你把 C code 編譯成 Wast 與 Wasm，並且可以編輯轉出來的 Wast，缺點是沒有 JS 能直接互動。

### 所以搭配操作的流程...

>先 WasmFiddle 來進行測試，接著把編好的 Wast 複製到 WasmExplorer 進行你想要的編輯，接著再 compile 成 wasm 並下載下來。

<!-- ## 與 JS 的交互使用-->

## 知道怎麼編譯 wasm 後，該說說 JavaScript 了吧

好的，但在那之前，要先提醒大家，除了 Chrome 57, Firefox 52 預設支援 WebAssembly 外，Safari 需要是紫色版本（Preview 版）才能使用，而 Edge 15 則是要開啟 Experimental JavaScript Features。

### 載入 wasm 到 Web 端

在 `<script src="abc.wasm" type="module" />` 還無法使用之前，想要載入 wasm 必須透過 `fetch` API。在 [Guy bedford 的影片範例](https://github.com/guybedford/wasm-intro/blob/master/4-reading-wasm-memory/test.html#L4) 與 [mdn 的 example](https://github.com/mdn/webassembly-examples/blob/master/wasm-utils.js#L6) 中的寫法都差不多：

```js wasm-loader.js
function fetchAndInstantiateWasm (url, imports) {
    return fetch(url) // url could be your .wasm file
    .then(res => {
    if (res.ok)
        return res.arrayBuffer();
    throw new Error(`Unable to fetch Web Assembly file ${url}.`);
    })
    .then(bytes => WebAssembly.compile(bytes))
    .then(module => WebAssembly.instantiate(module, imports || {}))
    .then(instance => instance.exports);
}
```

基本上會實作一個 `wasm-loader` 之類的函式，像上面的 `fetchAndInstantiateWasm`。

內容很簡單，取得 fetch 回來的 result 後，將其轉為 `ArrayBuffer`，利用 `WebAssembly.compile` 這個 Web API 來產生 WebAssembly Module，接著透過 `WebAssembly.instantiate` 來產生 module instance，最後的 instance.exports 就是我們在 wasm 中 export 出來的物件或 function。

除了 `fetch` 以外，`WebAssembly.compile` 與 `WebAssembly.instantiate` 也都是回傳 Promise。

這邊出現一個相信一般前端開發者也比較少看到的 **ArrayBuffer**。

ArrayBuffer 是 JavaScript 的一種 data type，用來表示 generic, fixed-length 的 binary data buffer，屬於 **typed arrays** 的一部分，而關於 **typed arrays** 雖然在 WebAssembly 中很重要，但是難以在這邊詳述，[mdn 的文件](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Typed_arrays)寫得很清楚，值得閱讀。

我們目前只要知道他是一個 array-like 的物件，讓我們能在 JavaScript 中存取 raw binary data，有 `Int8Array`、`Int32Array` 與 `Float32Array` 等 [DataView](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/DataView) 可以使用即可。（又一個名詞...DataView 提供 getter/setter API 來對 buffer 中的 data 做讀取。）

回到主題，如果你剛剛有先點進 [mdn 的 example](https://github.com/mdn/webassembly-examples/blob/master/wasm-utils.js#L6) 看，可能會發現他怎麼沒有 `WebAssembly.compile` 這個步驟？

實際上 `WebAssembly.instantiate` 有兩種 overload 實作：
* `Promise<ResultObject> WebAssembly.instantiate(bufferSource, importObject);`
* `Promise<WebAssembly.Instance> WebAssembly.instantiate(module, importObject);`

差別在於，先透過 `WebAssembly.compile` 後產生的 WebAssembly module，可以存在 indexedDB 中 cache，或是在 web workers 之間傳遞。

此外，WebAssembly.Instance 的第二個參數：`importObject` 是用來傳遞 JavaScript 的參數或 function 到 WebAssembly 程式中使用，後面會有範例。 

## 在 JavaScript 中使用 WebAssembly 實作的 function

有了剛剛的 `fetchAndInstantiateWasm`，取得 WebAssembly function 很方便：

```js
fetchAndInstantiateWasm('add.wasm', {})
    .then(m => {
      console.log(m.add(5, 10)); // 15
    });
```

使用上就是這麼簡單！

## 那能不能在 WebAssembly 中使用 JavaScript 寫的 function 呢？

當然可以！就是透過方才所說的第二個參數 `importObject`。

假設我們想要在剛剛的加法函數內進行 JS 的 `console.log`：

```c add.c
#include <math.h>
void consoleLog (int num);
int add(int num1, int num2) {
    int result = num1 + num2;
    consoleLog(result);
    return result;
}
```

先宣告一個 `consoleLog` 函式，並不需要實作他，因為這會是我們待會要從 JavaScript 那邊 import 進來的部分：

```js
fetchAndInstantiateWasm('./add.wasm', {
    env: {
        consoleLog: num => console.log(num)
    }
})
.then(m => {
    m.add(5, 3) // console.log 8
});
```

在剛剛的 `fetchAndInstantiateWasm` 的第二個參數中，我們定義一個 `env` object，並傳入一個內含 console.log 的函式。`env` 是一個特殊的 key，在剛剛的 add.c 當中，我們宣告的 `void consoleLog (int num)` 轉換到 add.wast 時，他會當作這個函式是從 `env` 中 import 進入的（line 2）：

```wast add.wast
(module
  (type $FUNCSIG$vi (func (param i32)))
  (import "env" "consoleLog" (func $consoleLog (param i32)))
  // ...函數內容省略，可參考前面的範例
)
```

### 難道只能從 env 載入嗎？

當然不是，我們也可以自己定義，但就要去更改 wast 檔案了，其實改過以後會發現邏輯不難懂，有讓我回味到大學修組語的感覺...

```wast add-10-20.wast
(module
  (type $FUNCSIG$vi (func (param i32)))
  (import "env" "consoleLog" (func $consoleLog (param i32)))
  ++(import "lib" "log" (func $log (param i32)))
  (table 0 anyfunc)
  (memory $0 1)
  (export "memory" (memory $0))
  (export "add" (func $add))
  (func $add (param $0 i32) (param $1 i32) (result i32)
    (call $consoleLog // 從 env 中載入的 consoleLog
      ++(i32.add
        (tee_local $1
          (i32.add
            (get_local $1)
            (get_local $0)
          )
        )
        ++(i32.const 20) // 從 env 載入的 consoleLog 多加 20
      )
    )
    ++(call $log // 從我們自己定義的 lib 中載入的 log
      ++(i32.add
        ++(get_local $1)
        ++(i32.const 10) // 從 env 載入的 consoleLog 多加 10
      ++)
    ++)
    (get_local $1)
  )
)
```

前面有加號的就是我們直接在 wast 中修改的程式碼，等同於如下 C 語言的程式：

```c add.c
#include <math.h>
void consoleLog (int num);
int add(int num1, int num2) {
    int result = num1 + num2;
    consoleLog(result + 20);
    log(result + 10); // 多了這個從 lib 匯入的 log 函數
    return result;
}
```


如此一來，我們就能夠像下面這般傳遞 `lib.log` 給我們的 wasm 使用了！

<a class="jsbin-embed" href="http://jsbin.com/mixure/embed?html,console">WASM Test on jsbin.com</a><script src="http://static.jsbin.com/js/embed.min.js?4.0.4"></script>

<!-- 操作 memory -->
## 現在我知道如何在 JS 與 WebAssembly 中互相使用函式了，但前面好像有提到他還能讓你操作 Memory?!

前面範例中的 wast 都有將 memory export 出來：`(export "memory" (memory $0))`
我們可以利用前面提及的 JavaScript Typed Array 來取得 memory buffer，並利用 [TextDecoder](https://developer.mozilla.org/en-US/docs/Web/API/TextDecoder) 這個較新的 Web API 來解碼：

```js
const memory = wasmModule.memory;
const strBuf = new Uint8Array(memory.buffer, wasmModule.getStrOffset(), 11);
const str = new TextDecoder().decode(strBuf);
console.log(str);
```

<a class="jsbin-embed" href="http://jsbin.com/gewayo/embed?html,console">JS Bin on jsbin.com</a><script src="http://static.jsbin.com/js/embed.min.js?4.0.4"></script>

可以讀取到 memory，當然也能寫入：

```js
function writeString (str, offset) {
      const strBuf = new TextEncoder().encode(str);
      const outBuf = new Uint8Array(mem.buffer, offset, strBuf.length);
      for (let i = 0; i < strBuf.length; i++) {
        outBuf[i] = strBuf[i];
      }
    }
```

對於 Memory 的操作部分，[Guy Bedford 的範例](https://github.com/guybedford/wasm-intro)有更多介紹，包含怎麼搭配 `malloc` 來動態調整記憶體。

<!-- 效能呈現 -->

## WebAssembly 對於效能的展現似乎到目前為止都沒有觸及耶？

要能夠展現 JavaScript 與 WebAssembly 的效能差異其實沒有那麼簡單，Guy Bedford 在影片中的範例是在螢幕上畫出多個圓圈，計算他們之間碰撞的狀況來移動，有趣的是，第一次的 Demo 中，JavaScript 的速度比 WebAssembly 實作碰撞計算的要快得多，然而在重新 optimize 演算法後，才讓 WebAssembly 的效能有大幅進展，比起 JavaScript 好上不少（同樣演算法）

這邊放個動態截圖給大家看，想自己跑跑看或是看程式碼的可以移駕 Guy Bedford 的 repo - [Wasm Demo](https://github.com/guybedford/wasm-demo)，載下來直接就能打開 html 執行囉！(要執行這個 Demo 需要 Chrome Canary 並在 chrome://flags 中啟動 Experimental Web Platform Flag)


![Wasm VS JS](/img/arvinh/wasmvsjs.gif)

<!-- ## 結論-->
## 結論

目前 wasm 在 Chrome 與 firefox 都已實作，雖然一定還會有規格上的變更，但了解一下這個勢必會影響未來 Web 開發的東西是有必要的！

本文也只是簡單介紹基礎的使用方法，實際上還有許多相關的議題，像是 **Type Arrays** 與 **WebAssembly Web API** 等等，都需要有所了解。甚至是如何將各種程式語言 compile 成 wasm 也是一門大學問，也有許多我沒有提及的工具可以使用（從資料來源中找得到）。

希望大家看完後可以對 WebAssembly 的使用方式有點概念，文中若有不清楚或是錯誤的地方也歡迎指正！

## 資料來源
1. [WebAssembly.org](http://webassembly.org/)
2. [Get Started Using WebAssembly (wasm)](https://egghead.io/courses/get-started-using-webassembly-wasm)
3. [WebAssembly Design](https://github.com/WebAssembly/design)
4. [W3C Community Group](https://www.w3.org/community/webassembly/)
5. [WebAssembly 系列（四）WebAssembly 工作原理](https://www.w3ctech.com/topic/2024)
6. [guybedford/wasm-intro](https://github.com/guybedford/wasm-intro)
7. [guybedford/wasm-demo](https://github.com/guybedford/wasm-demo))

關於作者： 
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化