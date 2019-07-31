---
title: 使用 JavaScript 處理二進位資料
date: 2017-09-24 10:46:57
tags:
  - JavaScript
  - Endianness
  - Binary Data
  - Memory Alignment
author: weihanglo
---

由於高度封裝與抽象，JavaScript 的執行效率比不上 C 的語言。例如 JavaScript 的 Array 下標（subscript）是根據 hash key 而非實體記憶體位址 offset 取值，雖然方便，卻多了效能開銷。當 Canvas、WebGL、WebVR 開始走紅，效能越來越受重視，如何讓 JavaScript 達到如同 C 指標般操作 binary data 變得至關重要。

存在許久但最近才變為 ES6 標準「**Typed Array**」就是解放 JavaScript 操作 binary data 能力的好工具！一起來了解 Typed Array 吧！

_（基於 ECMAScript 6+，Node.js 8.3）_

<!-- more -->

## Buffer v.s View

ES6 引入的 [Typed Array 家族][mdn-typedarrays]，可以分為兩大類：**Buffer** 與 **View**。

所謂 **Buffer** 是一個指向儲存資料的記憶體區塊之物件，類似於 `malloc` 配置出來的空間，無法直接存取或修改 buffer 內部的資料，在 JavaScript 中 Buffer 的實作就是 `ArrayBuffer`。

如果我們想存取某些 buffer 底下的內容，我們需要 **View**（視圖），透過宣告不同資料型別的 view，電腦就會了解如何操作這段 data chunk，該當作 float32 讀取呢？抑或以 unsigned integer 來操作。

ES6 規範了三個 Typed Array 相關物件，對應類別如下：

- `ArrayBuffer`：Buffer，代表一段記憶體區塊，僅能透過 View 操作其內容。
- `TypedArray`：View，儲存固定型別資料的 Array，例如 `Uint8Array`（8-bit unsigned integer）、`Float64Array`（64-bit IEEE floating point number)。
- `DataView`：View，不限制型別，可自定義從哪個 byte，以什麼型別，用哪種 byte order（endian）存取。

## ArrayBuffer

[ArrayBuffer][mdn-arraybuffer] 代表一段固定大小的記憶體區塊，也稱為 byte-array。主要的功能就是配置實體記憶體來儲存 raw binary data。一般很少直接操作 ArrayBuffer，實際上也只能將其 reference 傳給其他物件，讓其他物件來處理／使用資料。

建立一個 ArrayBuffer 有非常多種方法，可以直接配置，

```javascript
// 直接配置 8 bytes，初始值為 0 的記憶體區塊
const buffer = new ArrayBuffer(8)

// 利用 `slice` 將某些 bytes 複製到另一個 ArrayBuffer
// 這裡複製 `buffer` 倒數四個 bytes 到 bufferCopied 中
const bufferCopied = buffer.slice(-4)
```

或是最常使用的，HTTP response 選擇接收 buffer，

```javascript
// XMLHttpRequest 指定 resopnseType (XMLHttpRequest v2)
const xhr = new XMLHttpRequest()
xhr.open('GET', '/path/to/黑人問號.jpg', true)
xhr.responseType = 'arraybuffer' // 將 reponse 型別設定為 arraybuffer
xhr.onload = function(e) {
  console.log(this.response) // this.response 為 ArrayBuffer
}
xhr.send()

// Fetch API 也提供 Body#arrayBuffer 的方法轉換 Request／Response stream body
const response = await fetch('/path/to/柯P火影.gif')
const buffer = await response.arrayBuffer() // 取得 ArrayBuffer 實例
console.log(buffer.byteLength) // 查看當前這個 ArrayBuffer 有多少 bytes。
```

當然，也可以透過 `File` 與 `FileReader` API，讀取使用者上傳的資料。

```javascript
const input = document.querySelector('input')
input.addEventListener('change', handleFiles, false);

function handleFiles (files) { // files -> FileList 物件，裡面有 File 實例
  if (files[0]) {
    const reader = new FileReader()
    reader.onload = function ({ target: { result } }) {
      console.log(result.byteLength) // result 是一個 ArrayBuffer
    }
    reader.readAsArrayBuffer = files[0] // File 是特殊的 Blob 型別
  }
}
```

## TypedArray

TypedArray 並非任何一個型別，也非全域可取得的建構函數，而是一個抽象概念，對應到許多不同型別的 Array 罷了。老實說，TypedArray 這個的命名已說明一切，讓我來說文解字，先從 Typed 講起。

### Types of TypedArray

所謂的 Typed，意指「**限定型別**」，Array 中的元素都是同一種型別。有哪些型別呢？TypedArray 是為了操作 binary 而生，當然只有最底層以 bytes 為基礎，幾乎沒有什麼抽象概念的型別。我們可根據需求，決定每個元素該從 raw data 讀取多少與如何讀取 bytes。

目前 ES6 定義以下幾種 typed array types：

| 型別                 | bytes／元素 | 對應 C 語言 |
| ------------------- | :------: | :--------: |
| `Int8Array`         | 1    	   | int8_t     |
| `Uint8Array`        | 1    	   | uint8_t    |
| `Uint8ClampedArray` | 1    	   | uint8_t    |
| `Int16Array`        | 2    	   | int16_t    |
| `Uint16Array`       | 2    	   | uint16_t   |
| `Int32Array`        | 4    	   | int32_t    |
| `Uint32Array`       | 4    	   | uint32_t   |
| `Float32Array`      | 4    	   | float      |
| `Float64Array`      | 8    	   | double     |

實際上，TypedArray 本身並並不儲存任何 buffer 資料，只保存該 buffer 的 reference，我們可以透過 `TypedArray#buffer` 獲取原始的 ArrayBuffer。也因此，同一個 ArrayBuffer 可以建構出多個不同的 TypedArray。可視為「**從不同視角解讀 ArrayBuffer 中的 binary data**」。

例如下圖是一個 16 bytes 的 ArrayBuffer，我們可以透過它，建立多個不同型別的 TypedArray。

![](https://mdn.mozillademos.org/files/8629/typed_arrays.png)

不同型別的 TypedArray 的元素對應到不同的 byte 數量，這項資訊會記錄在 `TypedArray#BYTES_PER_ELEMENT` property 上。例如：Uint8Array 一個元素對應到一個 byte，Float64Array 則對應到 8 bytes。

如果還是無法理解，其實可以將 TypedArray 想像為 C 語言的 `void *ptr`，在存取、遍歷不同 data type 的 Array 時，轉型（cast）成不同的型別，讓指標根據不同 data type 的 size 做對應 offset。

### Array-like Methods

而 TypedArray 中的 Array，其實就是我們熟悉的 JavaScript Array，可視為「**在 ArrayBuffer 的資料之上，架一層可存取資料的 Array API**」。你想得到的 method `map`、`filter`、`reduce` 幾乎應有盡有，而 `push`、`shift`、`unshift`、`splice` 這類會改變 Array 長度的 **Mutator methods** 沒有實作，畢竟 TypedArray 就只是 buffer 的 reference，`pop` 後原始資料依然存在。

比較好玩的是，`TypedArray#subarray` 和 `TypedArray#slice` 同樣是回傳陣列切片，`slice` 是回傳一個淺拷貝（shallow-copy）的**新 Array**，新 array 的 `buffer` property 指向新切出來的 buffer，`byteOffset` 也是依據新的 buffer，所以會是 **0**。

而 `subarray` 則是在同個 buffer 繼續切片，調用 subarray 的 `buffer` 會取得相同的原始 buffer，`byteOffset` 也是根據原始 buffer 計算 offset。

### Consturct a TypedArray

建構 TypedArray 非常簡單，選擇好 data type 之後，`new` 一個就完成了！

**直接初始化**

建立一個 4 * 2 bytes 初始值為 0 的 Uint16Array。

```javascript
const u16 = new Uint16Array(4)
```

**從 TypedArray 建立**

我們也可以從其他 TypedArray 建立相同長度的 TypedArray，會指向同一個 buffer。

```javascript
const u8 = new Uint8Array(u16) // length of u8 is 4
```

這樣會建立一個新的型別 array，但記憶體區塊不變。我們的例子中，u8 因為溢位的緣故（overflow），自動過濾偶數 bytes（或奇數，視 [endianness][wiki-endianness] 而定），僅顯示餘下 4 個 bytes 的資料，記憶體位址變得不連續。

**從 ArrayBuffer 建立**

不過，也可以透過 `TypedArray#buffer` 取得並共享當前的 buffer，該 array 的記憶體區間就會是連續的。

```javascript
// 因為 buffer 總共有 4 * 2 = 8 bytes，所以 u8_continuous 長度為 8
const u8_continuous = new Uint8Array(u16.buffer)
```

當然，ArrayBuffer 可直接配置一塊記憶體區塊，並使用它建構 TypedArray，甚至透過 `length` 和 `byteOffset` 指定該 buffer 不同的區間來建構。

```javascript
const buffer = new ArrayBuffer(16)
const i32 = new Int32Array(buffer) // 32 * 2 bytes

// 從 4 bytes offset 的位址開始，切一個長度為 7 bytes 的 array。
const i8 = new Iint8Array(buffer, 4, 7)
```

### Overflow

如同 C 語言，不同類型的 TypedArray 可以容納的 bytes 範圍是固定的，超過此一範圍，就會出現[「溢位（Overflow）」][wiki-integer-overflow]，例如 Uint8Array 中僅能放入 1 byte = 8 bits 的資料，如果放入 `0x100`（256，9 bits），就會溢位。

那溢位後，資料會怎麼呈現呢？

每個語言實作不盡相同，TypedArray 的溢位處理規則和多數語言相同：**捨棄溢出的 high bits。** 我們來看簡單的例子。

```javascript
Uint8Array.of(0xff, 0x100)
// Unit8Array [255, 0]
```

第一個例子中，我們選用 Uint8Array，一個元素最多儲存 8 bits 的資料，第二個元素是 256，需要第 9 bit 來儲存，因此溢位。

```javascript
// 255，至少需要 8 bits 儲存
0b11111111

// 256，至少需要 9 bits 儲存
0b100000000
//└── 這個 1 溢位，將被捨棄，僅保留最低有效的 8 bits，計算結果為 `0`
// 捨棄的方式同於 bitwise or `& 0xFF`
0x100 & 0xFF
// 0
```

> Note：underflow 的處理方式與 overflow 相同。

### What is Uint8ClampedArray

Clamp 的本意是鉗子，在計算機科學中，通常意味將資料值限制在特定範圍間。而 `Uint8ClampedArray` 中，就是將元素值限制在 0 - 255。換句話說，就是處理溢位的規則與 `Uint8Array` 不同。**當 overflow 時，該值會等於最大值 255；當 underflow 時，該值會等於 0**。

```javascript
Uint8Array.of(0xff, 0x100, -100)
// Unit8Array [255, 0, 156]
Uint8ClampedArray.of(0xff, 0x100, -100)
// Uint8ClampedArray [255, 255, 0]
```

這有什麼好處呢？在影像處理上非常方便。有個很常舉的例子，有 3 bytes 的 Uint8Array 存放 RGB 色碼，我們想要增加他的 gamma factor，如果使用 Uint8Array 儲存：

```javascript
// 必須自行限制大小，防止 ooverflow／underflow。
u8[i] = Math.max(0, Math.min(255, u8[i] * gamma)) // u8 是一個 Uint8Array
```

如果是 `Uint8ClampedArray`，只需要直接乘上 gammer factor，非常方便。

```javascript
pixels[i] *= gamma // pixels 是一個 Uint8ClampedArray
```

### Composite Data Structure

當需處理類似 C struct 的複合資料結構，如下所示

```c
struct employee {
  unsigned int id;               // 4 * 1 bytes
  unsigned char department[4];   // 1 * 4 bytes
  float salary;         // 4 * 1 bytes
};
```

我們可以宣告對應的 TypedArray 來處理。模擬出如果 C struct 的資料結構。

```javascript
const buffer = new ArrayBuffer(12)
const idView = new Uint32Array(buffer, 0, 1)
const deptView = new Uint8Array(buffer, 4, 4)
const salaryView = new Float32Array(buffer, 8)
idView[0] = 123
deptView.forEach((_, i) => { deptView[i] = i * i })
salaryView[0] = 10000
```

## DataView

顧名思義，**DataView** 是一種建構在 buffer 之上的 view。與一般 TypedArray 不同的是，建構 DataView 時並不會固定的資料型別，取而代之的是存取 data 時，必須明確的指定從哪個 byte offset 取哪一種 data type 出來。

借用前例的複合資料來示範，`DataView` 如何針對每個 bytes 處理自定義的資料。

```javascript
const dv = new DataView(buffer)
// 從 byte offset 0 的位址開始取 Uint32 的資料
// 取得 ID -> 123
dv.getUint32(0, true)

// 從 byte offset 8 的位址開始寫入 Float32 的資料
dv.setFloat32(8, 200000, true)
dv.getFloat32(8, true) // 加薪囉！！
```

各位有沒有注意到，DataView 的 bytes getter／setter 最後面都多帶了一個 boolean 參數？這個參數是指定使用 Little-endian 讀取資料，預設為 `false` 也就是以 Big-endian 讀取。可控制 endian 是 DataView 蠻重要但也頗惱人的特性，在下一節會介紹 Endianness。

DataView 另一個重要特性就是不會 [buffer overflow][wiki-buffer-overflow]，所謂的 buffer overflow 是「**當寫入一筆資料到指定 buffer 中，若寫入的資料大小超過該 buffer 的 boundary，溢出值就會覆寫下個 byte**」。這種不安全的性質，也讓 buffer overflow 成為許多駭客的攻擊手法，有潛在的安全性問題。而透過 DataView setter 賦予一個超過型別最大值的數字，並不會覆蓋臨近記憶體位址的資料，而是內部先檢查邊界，處理 overflow 之後，再寫入該記憶體區間，彌補了 buffer overflow 的漏洞。

## Precautions

Typed Arrays 幾乎可以做到如同 C 語言般細膩的記憶體操作。不過越是自由的 API，就代表要學習更多知識，注意更多細節，以下是操作 TypedArray 該銘記在心的事情：

- [Endianness (Byte order)][wiki-endianness]
- [Data Structure Alignment][wiki-alignment]

### Endianness (Byte order)

在計算機科學領域下，Data 是一個物理概念，指儲存在電腦記憶體上的一個 bits／bytes 序列。Data 本身並沒有任何意義，想使用它，必須自行解讀出抽象的意義，例如將 Data 讀取為字串或數字。

我們知道記憶體是基於位址（address）依序儲存 Data，每個位址可以存上 1 byte data。如果使用 `Uint8Array` 這種一次存取一個 byte 的格式，那麼完全不會有任何問題，怎麼存取，都是依連續的記憶體位址順序：

```
--- 資料讀取順序 -->
| Offset | 0    | 1    | 2    | 3    |
| ------ | ---- | ---- | ---- | ---- |
| Data   | 0x11 | 0x22 | 0x33 | 0x44 |
```

我們會得到 `[17, 34, 51, 68]` 的 Array。

當我們需要一次存取多 bytes，例如這個範例的記憶體區塊其實是一個 32 bit 的整數，那實際代表的數字會是多大？是 `0x11223344`（十進位：287454020) 嗎？

這其實牽扯到 CPU 的設計，目前市面上的多數 CPU 處理 multi-bytes 的資料時，大多從「最低有效位（LSB，least significant byte）」，也就是從權重最小的位數開始寫入。所以讀取這筆資料時，最前面的記憶體 offset 就是最權重最小的 bytes，所以最後會得到 `0x44332211`（十進位：1144201745）。這種存取排序，我們稱之為 **Little-endian**。

有最低，當然就有最高，**Big-endian** 則是從「最高有效位（MSB，most significant byte）」開始存取。因此照著記憶體位址依序讀取，會得到跟記憶體 offset 順序相同的 `0x11223344`。Big-endian 雖然在個人電腦中不常見，但許多網路協議和設備都是採用 Big-endian 存取資料，佔有一定的重要性。

Little-endian 和 Big-endian 可以視為不同的電腦（CPU）講不同的語言，一個從右到左，另一個從左到右。其實在人類日常生活中也可以看到相同的現象，例如歐洲常用的日期格式為 day-month-year，ISO 國際標準則反之 year-month-day，所以，當你看到一個 17-09-07 的日期時，必須先判斷是否為 ISO 的標準，才能知道這場約會是在下禮拜的九月七日，抑或你需要一台時光機回到過去。

**Q：那在 JavaScipt 要如何處理 endian？**

如果不碰底層的記憶體操作，寫 JavaScript 是不用理會 endianness 的，但當你要操作 TypedArray 時，了解 data 的 endian 就至關重要了。`TypedArray` 預設是使用系統的 endianness，所以如果你接收一筆資料，與你的系統的 endianness 不一致，TypedArray 便使不上力。而前面介紹到 DataView 的 byte getter／setter 最後一個參數就是用來決定以哪種 byte order 存取資料，預設是 Big-endian（`false`），透過切換這個 flag，任何 binary data 都橫看成嶺側成峰了。

**Q：那我們要如何得知資料的 byte order？**

如果資料是自己家內部系統使用，其實溝通好就 OK，用 Mixed-endian 也不會有人管你。但如果是外界得來的任意資料，我們可以透過「[BOM（byte order mark）][wiki-bom]」來判斷資料屬於哪種 endianness。BOM 是一個 Unicode  magic number，通常放置在 text stream 的最前端。不過，並不是每個資料都會加上這個 header，而且有時候我們不需要 BOM 資訊，使用資料前還必須先 [strip bom][npm-strip-bom]，說實話挺麻煩的。

### Data Structure Alignment

要接觸底層的記憶體，免不了瞭解 CPU 如何從記憶體中讀取資料，記憶體底層到底如何配置。

一般來說，現代的 CPU 通常設計以 word 為單位（例如 4 bytes）讀寫記憶體裡的資料，而資料對齊（data aligment）則是將資料放置在 word-size * n 倍的記憶體位址上，使 CPU 以最為有效率的方式讀寫。那為什麼對齊 word-size 會最有效率呢？假設有個 C struct 如下：

```c
struct AlignDemo {
  char c;     // 1 byte
  int i;      // 4 bytes
  short s;      // 2 bytes
};
```

理論上的記憶體配置如下，總共需要 7 bytes 的記憶體空間。

```
c = char 所佔的 byte
s = short 所佔的 byte
i = int 所佔的 byte

| 0x000           | 0x020           |
| [c] [i] [i] [i] | [i] [s] [s] [ ] |
```

前面提到 CPU 是以 word-size 存取記憶體上的資料，當嘗試讀取 char 和 short 時並沒有什麼問題，CPU 只需取一次 word chunk 再 offset 就可取得正確的值。然而，當欲讀取 int 時，CPU 需先取第一個 data chunk 以獲取 int 前三個 bytes，再取第二個 word chunk 並 shift 資料，以取得 int 最後一個 byte。如此多餘的記憶體存取會造成 CPU 額外的負擔。

解決方法是 **Data Structure Padding**，也就是在資料無法對齊 word-size 時，加上一些填充用的成員。

在我們的例子中，可以這樣做：

```c
struct AlignDemo {
  char c;
  char padding_0[3]; // 填充用成員
  int i;
  short s;
  char padding_1[2];
};

```

記憶體配置則如下：

```
p = padding 所佔的 byte

| 0x000           | 0x020           | 0x040           |
| [c] [p] [p] [p] | [i] [i] [i] [i] | [s] [s] [p] [p] |
```

本來只需要 7 bytes，對個齊後，反而用掉這麼多額外的 bytes，你玩我嗎？

我們可以試著改變一下 struct member 的順序：

```c
struct AlignDemo {
  int i;
  char c;
  short s;
  char padding[0]
};
```

記憶體配置對應改變，只佔用 8 bytes。Brilliant！

```
| 0x000           | 0x020           |
| [i] [i] [i] [i] | [c] [s] [s] [p] |
```

結構對齊（struct alignment）在 C 語言中是一門不小學問，除了結構內的成員本身要對齊，結構本身也要對齊。

回到 JavaScript，當你在創建不同的 view 時，JavaScript engine 其實會進行簡單的 [natural alignment][stackoverflow-natural-align] 檢查。

```javascript
const buffer = new ArrayBuffer(6)
new Uint16Array(buffer, 1)
// RangeError: start offset of Uint16Array should be a multiple of 2
new Uint32Array(buffer, 0)
// RangeError: byte length of Uint32Array should be a multiple of 4
```

所以囉，當我們在設計複合資料時，想想對應的 C struct alignment，多考量記憶體底層，才不會讓操作 binary data 產生效能低落的反效果。

## Others

最後，讓我們來認識除了 Typed Array 家族以外，JavaScript 的生態圈其他與記憶體息息相關的成員吧！

### Node.js `Buffer`

早在 ES6 引入 TypedArray 之前，Node.js 為了處理 binary data，就實作 [Buffer class][nodejs-buffer]，也針對 V8 引擎做最佳化。Buffer 在 Node.js 的環境中是 Global object，其功能可視為 ArrayBuffer + TypedArray + DataView 的複合體，甚至可以配置 non-zero-filled 的 unsafe buffer，好危險啊。

在使用上，Buffer 可從 ArrayBuffer 建構，也可從自身建構 TypedArray。事實上，Node.js v3+ 之後，Buffer 就繼承自 `Uint8Array` 了，不過有些 memory share／copy 的實作與 spec 有出入，在與 TypedArray
 ArrayBuffer 轉換時，[需注意這些小細節][nodejs-buffer-caveats]。

### Web API `Blob`

[Blob][w3c-blob] 是一個不可變（immutable）的 raw binary sequence，只有兩個 attribute 和一個 method。

- `size`：blob 實例的 byte 大小。
- `type`：blob 實例的 [MIME type][wiki-mime]。
- `slice`：切割一部分的 blob 實例，返回新的 blob。

Blob 的 spec 寫在 [W3C File API draft][w3c-blob] 中，為 `File` class 的父類別。主要目的是提供可代表與儲存 JavaScript native 以外的格式，例如以 blob 儲存 **死肥宅.jpg**。Blob 除了可以從 object 建構，也可傳入 TypedArray 或 DOMString 建構。此外，File API，Fetch API、XMLHttpRequest v2 也都可以將 Request／Response 的 body 轉換成 Blob，非常泛用途呢！

而 Blob 最強大的地方就是配合 `URL.createObjectURL` 生成一個 Blob URL。如同你我認知中的 URL，任何運用 URL 之處，都可以傳入 Blob URL，比起 `Image`、`ImageData`、`MediaSource`，URL 接受與使用度肯定更為廣闊，這讓資料處理，物件傳遞的耦合性變得更低。

當我們建立 Blob URL 後，若可預期的未來內不需要用到該 URL，就使用 `URL.revokeObjectURL` 取消註冊，否則該 URL 指向的 Blob 會持續留存，佔用你的儲存空間，直到瀏覽器執行 unload document cleanup 的步驟（如關閉分頁），才會將所有 Blob URL 清除。所以說，如需管理 Blob URL，還是老老實實把這些 URL 記錄起來吧！

## Wrap-up

藉由這些直接操作 binary data 的 API，現代的 JavaScript 環境的效能提升到另一個層次，若再配合 Web worker  Service worker 等多線程技術，加上線程共享的 `ShareArrayBuffer` 與 `Atomic` API，高效能的 web app 指日可待。如果再加上逐漸普及，[即將成為 Webpack 一等公民][medium-webpack-awarded] 的 [WebAssembly][wasm]，JavaScript／Web 的世界更是不可限量啊！或許，使用 Rust 寫網頁的世代即將來臨 XD。

前端工程師們，活到老，學到掛吧！

## Reference

- [MDN: ArrayBuffer][mdn-arraybuffer]
- [MDN: TypedArray][mdn-typedarrays]
- [W3C: File API - Blob][w3c-blob]
- [Wiki: Data structure alignment][wiki-alignment]
- [Wiki: Endianness][wiki-endianness]
- [Wiki: Integer overflow][wiki-integer-overflow]
- [HTML5 Rocks: Typed Arrays][html5rocks-typedarray]
- [Node.js: Buffer][nodejs-buffer]
- [OPass：關於記憶體對齊(Alignment) ][opass-memory-alignment]
- [阮一峰：ECMAScript 6 入门][rungyifeng-es6]

關於作者：  
[@weihanglo](https://weihanglo.github.io) 掛著 iOS 工程師之名，行開發 Web App 之實。

[html5rocks-typedarray]: https://www.html5rocks.com/en/tutorials/webgl/typed_arrays/
[mdn-arraybuffer]: https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/ArrayBuffer
[mdn-typedarrays]: https://developer.mozilla.org/en-US/docs/Web/JavaScript/Typed_arrays
[medium-webpack-awarded]: https://medium.com/webpack/webpack-awarded-125-000-from-moss-program-f63eeaaf4e15
[nodejs-buffer-caveats]: https://nodejs.org/dist/latest-v8.x/docs/api/buffer.html#buffer_buffers_and_typedarray
[nodejs-buffer]: https://nodejs.org/dist/latest-v8.x/docs/api/buffer.html
[npm-strip-bom]: https://www.npmjs.com/package/strip-bom
[opass-memory-alignment]: http://opass.logdown.com/posts/743054-about-memory-alignment
[rungyifeng-es6]: http://es6.ruanyifeng.com/#docs/arraybuffer
[stackoverflow-natural-align]: https://stackoverflow.com/a/25658188
[w3c-blob]: https://w3c.github.io/FileAPI/#blob
[wasm]: http://webassembly.org/
[wiki-alignment]: https://en.wikipedia.org/wiki/Data_structure_alignment
[wiki-bom]: https://en.wikipedia.org/wiki/Byte_order_mark#UTF-16
[wiki-buffer-overflow]: https://en.wikipedia.org/wiki/Buffer_overflow
[wiki-endianness]: https://en.wikipedia.org/wiki/Endianness
[wiki-integer-overflow]: https://en.wikipedia.org/wiki/Integer_overflow
[wiki-mime]: https://en.wikipedia.org/wiki/MIME
