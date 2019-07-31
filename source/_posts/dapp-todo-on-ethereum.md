---
title: 在 Ethereum 上開發簡單的 Todo App
date: 2018-04-01 10:00:13
tags:
  - ethereum
  - smart contract
  - dapp
---
## 前言

近一兩年區塊鏈的技術造成一股熱潮，由於加密貨幣在投資上的炒作，常看到的區塊鏈範例都是和虛擬貨幣相關連的服務，另外，技術的應用焦點也常放在金融和會計業上，這是因為這兩個行業長久以來在市場上都有球員兼裁判的嫌疑（例如會計有資產信用背書和被雇用人這兩種矛盾的角色），因此需要一個「備受信任」的機制來重拾市場對它們的信任。而區塊鏈，或是廣義上來說的分散式賬本技術，正是一個有潛力的解法。不過就像《區塊鏈革命》這本書所說的，這個技術實現了「價值上的轉移」，理當會對更多的行業帶來影響。

Ethereum 提出 DApp(Decentralized App) 的想法，也就是藉著區塊鏈上佈署的智慧合約（Smart Contract）對區塊鏈資料進行操作，使得不但資料無法被竄改，連合約（程式碼）本身也無法被竄改，因此這些合約可以在沒有第三方（例如律師）的見證下具有信用。以下簡單的將傳統 web app 和 DApp 作類比：

| -- | -- | -- | -- |
| -- | -- | -- | -- |
| Web app | front-end | back-end API | Database |
| DApp | front-end | smart contract | Blockchain(Ethereum) |

由於這篇只會提到開發 Todo 程式的必要部分，如果需要更完整對於智慧合約的介紹和 Ethereum 的相關知識可以參考 Taipei Ethereum Meetup 的[部落格](https://medium.com/taipei-ethereum-meetup)或是 gasolin 網友編寫的 [gitbook](https://www.gitbook.com/book/gasolin/learn-ethereum-dapp/details)。

另外，雖然這裡只是實作一個簡單的 Todo 程式，但是可以想像如果擴展成接案平台的核心，發包人和接案人商量一個完成任務的評估方式和報酬，例如完成幾個測試後會收到多少款項，接著將剛剛的合約寫入 DAapp，未來只要接案人的專案通過測試後就能自動完成收款。

以下對 DApp 的實作利用 Truffle 這個開發框架，以 Solidity（Ethereum 官方開發的編程語言）開發智慧合約，並利用 javascript 的 web3.js 套件和合約溝通，以下分別就這兩個工具介紹。另外如果使用 VSCode 開發，可以安裝 Solidity 的[開發工具](https://github.com/juanfranblanco/vscode-solidity)，方便檢查語法上的問題。

這是這篇所使用到的[程式碼](https://github.com/pomelyu/EthereumTodo.git)

## Truffle - Ethereum Development Framework
[Truffle](http://truffleframework.com) 是 Ethereum（以太坊）的開發框架，可以建立測試用的區塊鏈，並將寫好的智慧合約編譯佈署，由於以太坊的開發環境變動的相當快，因此務必注意到不同版本的支援問題。這裡使用最新的 truffle 版本(v4.1.3)，因為這個版本建立的測試用區塊鏈支援 websocket 連線，配合 web3.js v1.0 之後的版本可以利用 socket 監聽事件的觸發，這是比較有效率的作法。

### Step1. 安裝 truffle
```bash
npm install -g truffle@4.1.3
```

### Step2. 建立專案資料並初始化
```bash
mkdir EthereumTodo && cd EthereumTodo
truffle init
# 資料夾結構如下
# build/       合約編譯完才會產生，這裡會生成描述合約的 json 檔，包含合約的 ABI(Application Binary Interface)
# contracts/   合約的檔案，以 .sol 為結尾
# migrations/  描述如何將合約佈署到區塊鏈
# test/        用來測試合約
# truffle.js   設定 truffle
```

### Step3. 查看版本資訊 
```bash
truffle version
# Truffle v4.1.3 (core: 4.1.3)
# Solidity v0.4.19 (solc-js)
```
請務必先檢查支援的 Solidity 版本，由於目前這個語言變動很快，需要更加注意版本間的語法差異。

### Step4. 建立測試用區塊鏈
```bash
# ganache-cli 是 truffle 內附的指令，用來取代原先的 testrpc
# 用 --seed apple banana cherry，指定隨機生成的種子，這樣可以確保每次建立的區塊鏈都是相同的，
# 如此一來合約的佈署位址也會相同，這在測試環境上非常好用
ganache-cli --seed apple banana cherry

## 執行結果會順便產生測試用的帳號和 key，帳號會在之後執行合約時用到
# Available Accounts
# ==================
# (0) 0x1d489c3f8ed5ee71325a847888b2157c9ac29c05
# ...
# Private Keys
# ==================
# (0) bea70301d065cf7946f25251c73dbfff93d4320715e43bdc0d5087553074cb64
# ...
# Listening on localhost:8545
```

### Step5. 設定 truffle 環境
```javascript
// truffle.js
module.exports = {
  networks: {
    development: {
      host: "localhost",
      port: 8545,     // default port of ganache-cli
      network_id: "*" // Match any network id
    }
  }
};
```

## 建立 Todo 合約
### Step1. 設定資料結構
```java
// contracts/TodoFactory.sol
// 指定編譯的版本
pragma solidity ^0.4.17;

// 指定合約的名稱，之後佈署或是測試時都是根據這個名稱（不是檔案名稱）
contract TodoFactory {
  struct Todo {
    string taskName;
    bool isCompleted;
    bool isValid;
  }

  Todo[] todos;
}
```
以上簡單的設定在合約中資料儲存的方式，利用 `struct` 包裹每個 todo 應該包含的資料，並利用陣列儲存。Solidty 常用的型別包含`int(uint, uint256), uint8, bool, address(8 bytes), byte` ，而 `string` 相當於是 `byte[]`。由於儲存資料在區塊鏈上相當耗費成本（以 POW 機制來說，就是需要有人挖礦），智慧合約的執行上也必須消耗 **gas**，因此會盡可能的選用適當的型別以減少寫入的資料量和運算量，另外在 `struct` 中盡可能的將相同的資料型別排列在一起，也可以節省儲存的資料量。

除了 `Array` 之外，Solidity 中常用的還有 `mapping`，代表 key-value 之間的對應。例如
```java
// 由 id(int) 對應到 todo(Todo)
mapping(int => Todo) idMapTodo;
```

### Step2. 加上操作資料的 Function(Method)
```java
contract TodoFactory {

  function addTodo(string _taskName) public {
    Todo memory todo = Todo(_taskName, false, true);
    uint todoId = todos.push(todo) - 1;
  }
}
```
在 `addTodo` 中，我們先以輸入的 `_taskName` 初始化一個型態是 `Todo` 的物件，接著加進合約中的 todos 陣列，並以陣列索引當作 id。注意到上面的函式即使加上回傳值也無法回傳預期的結果，之後會再解釋這部份。

Solidity 中 contract 和 function 的關係，可以類比成 class 和 function 的關係，contract 也是可以被繼承的，而 function 可以加上一些 **modifier**，例如：
 - `public`: 代表可以被外界調用
 - `private`: 代表只能被此合約中的其他 function 調用
 - `internal`: 代表可以被此合約和繼承的合約調用，像是 c++ 的 `protected`
 - `external`: 代表只能被外界調用
或是
 - `view`: 代表此 function 不會對區塊鏈上的資料作任何的更改，像是 get function
 - `pure`: 代表此 function 不會操作到區塊鏈上的任何資料，所以 pure function 不會消耗任何的 **gas**，可以想像這就是 util function

另外需要特別注意的是，Solidity 中**執行函式的方式**被分成 **Call** 和 **Transaction** 兩種（雖然程式碼都是 function）
 - **Call**: 代表執行函式但是不會對區塊鏈作任何的修改，可以使用「回傳值」，通常會被這樣使用的函式包含 `view` 或是 `pure` 這兩個關鍵字，如果對於一個有寫入的函式使用 call 的方式執行，結果**不會**寫入任何的資料
 - **Transaction**: 和 Call 相反，在執行上會寫入資料，因此需要等待礦工們將資料寫入，所以函式的「回傳值」僅代表 transaction hash，不會回傳預期的結果。

以上可以參考這個[討論串](https://ethereum.stackexchange.com/questions/765/what-is-the-difference-between-a-transaction-and-a-call)

除了預設的關鍵字，也可以利用關鍵字 `modifier` 宣告自訂的 **modifier**，類似 python 或是 js ES7 中的 decorator（裝飾字），例如
```java
contract TodoFactory {

  modifier isValidTodo(uint _todoId) {
    // require 要求傳入的參數要是 true，否則中止操作，並退還執行時消耗的 gas
    // 以下連結有更詳細的比較和說明
    // https://medium.com/taipei-ethereum-meetup/比較-require-assert-和-revert-及其運作方式-30c24d534ce4
    require(isTodoValid(_todoId));
    _; // 這是語法上必要的
  }

  function deleteTodo(uint _todoId) public isValidTodo(_todoId) {
    todos[_todoId].isValid = false;
  }
}
```
在上述的程式碼中，首先利用自訂的 **modifier** 檢查想要刪除的元素是否有效，有效才會刪除。回到前面新增物件的部分，把陣列索引當作 id 是很奇怪的作法，因為實作上如果把陣列的元素移除，可能會想用其他的元素填補這個空隙，以節省儲存空間，這樣索引就會被更改，例如 [a, b, c] 移除 b 後會變成 [a, c]，但是因為區塊鏈的寫入成本極高，因此當刪除陣列元素時，**不應該**搬移其他的元素，所以直接將這個元素設定成 invalid 是成本較低的作法，當然也可以用 `delete` 刪除，但是會造成空隙。

另外 Solidity 中的函式可以回傳複數的值，回傳時類似 tuple 以`()`包裹，不過回傳的資料型別只能是原始的資料型別或是陣列，也就是說不能回傳 `struct` 或者是 `string[]`。

### Step3. 加上 event 標示已完成的事件

如同前面提到的，執行 `addTodo`, `deleteTodo`, `completeTodo` 的時候都會以 **Transaction** 的方式執行（不然不會寫入資料），因此為了讓其他人知道執行完畢，並收到執行的結果，必須使用 event 觸發的方式。
```java
contract TodoFactory {

event OnTodoAdded(uint todoId);

  function addTodo(string _taskName) public {
    Todo memory todo = Todo(_taskName, false, true);
    uint todoId = todos.push(todo) - 1;
    
    // trigger event:
    // 這樣才能取得原來的回傳值 todoId
    // 在 v0.4.21 之後，必須寫成 emit OnTodoAdded(todoId)
    OnTodoAdded(todoId);
  }
}
```
`event` 的宣告就像是函式的 header，利用 `event` 可以讓多個 client 監聽智慧合約的變化，而且這些 `event` 一旦被觸發就會被紀錄在區塊鏈裡面，未來可以很輕易的查詢過去發生過的紀錄 

### Step4. 編譯及佈署合約
原則上就是照抄 `migrations/1_initial_migration.js`
```java
// migrations/4_deploy_todoFactory.js
const TodoFactory = artifacts.require('TodoFactory');

module.exports = function (deployer) {
  deployer.deploy(TodoFactory);
}
```

接著依序編譯和佈署合約到自建的測試區塊鏈上
```bash
truffle compile
```
執行結果會 在 `build/` 建立 `TodoFactory.json`

```bash
truffle migrate

## 執行結果
# Running migration: 4_deploy_todoFactory.js
#   Deploying TodoFactory...
#   ... 0xfecf0206d68c496cf067e320a4d4b5d294dfe89979552f7b6b8ab38696c51356
#   TodoFactory: 0x21e4624c5a0b3fda81d0833d412dded2bb3a7a7c
# Saving successful migration to network...
#   ... 0x6f592087ebfa7d5d77cce3f82c9d1222148c25499c348a59480ccfb3fe5884e1
# Saving artifacts...
```

其中 0x21e4624c5a0b3fda81d0833d412dded2bb3a7a7c 就是合約部署在區塊鏈上的位址

### Step5. 測試
由於已經佈署在區塊鏈上的合約無法再被修改，最多只能利用事先設定的函數調整參數，因此每一次的合約更新都會導致地址的改變，在實際的應用上代表著每次合約的更新都需要改變利用到的合約位址，這是非常麻煩的事情，因此佈署前的測試相當重要，另外測試對於 TDD（Test Drive Development） 或是 CI,CD 的流程也是不可或缺的。truffle 內建測試用的框架，利用 Mocha 和 Chai 這兩個在 javascript 中常用的套件（兩者所使用的版本可以從 [ganache-core](https://github.com/trufflesuite/ganache-core/blob/develop/package.json) 查看）

以下的測試可以驗證合約是否正常發佈
```javascript
// test/test_todoFactory.js
const TodoFactory = artifacts.require('TodoFactory');

contract('TodoFactory', function(accounts) {
  before(async () => {
    // 在所有測試開始前佈署合約
    contract = await TodoFactory.deployed();
  });

  it('Should contract deployed properly', () => {
    // 驗證合約是否已經被佈署
    assert.isDefined(contract);
  });
}
```
對於非同步的情形而言，可以使用 Promise 或者是 Callback 的語法，這裡用 Promise 配合 async-await 比較簡潔。不過因為目前 ganache-core 所使用的 chai 版本為 3.5，無法抓到非同步的錯誤，因此如果需要這方面的測試可以用以下的寫法
```java
contract('TodoFactory', function(accounts) {
  it('Should not complete invalid task', async () => {
    contract.completeTodo(9527, (err) => {
      assert.isDefined(err);
    })
  });
}
```

另外在測試的部分 truffle 對於 **Call** 和 **Transaction** 兩種執行方式並沒有區分，都是用一般函式的呼叫方式，差別在後者回傳的是 transaction hash，從 hash 可以取得觸發的 event 的資訊
```javascript
// test/utils.js
function getEvents (tx, filter) {
  const logs = tx.logs;
  const events = _.filter(logs, filter);
  return events;
}

// test/test_todoFactory.js
contract('TodoFactory', function(accounts) {
  it('Should add new todo properly', async () => {
    // addTodo 的呼叫是 Transaction 所以即使 addTodo 中有回傳值，也無法收到
    const tx1 = await contract.addTodo(Todo1.taskName);
    const events1 = utils.getEvents(tx1, { event: 'OnTodoAdded', logIndex: 0 });
    todoId1 = events1[0].args.todoId;
  });
}
```

另外如果要檢查執行時觸發的 event 可以參考 stackoverflow 上的[討論](https://ethereum.stackexchange.com/questions/15353/how-to-listen-for-contract-events-in-javascript-tests)來驗證
```javascript
// test/utils.js
function assertEvent(contract, filter) {
  return new Promise((resolve, reject) => {
    const event = contract[filter.event]();
    // event.watch, event.get, event.stopWatching
    // 在 web3 中也有對應的 function 來監聽區塊鏈上的事件
    event.watch();
    event.get((error, logs) => {
      const log = _.filter(logs, filter);
      if (!_.isEmpty(log)) {
        resolve(log);
      } else {
        reject(new Error("Failed to find filtered event for " + filter.event));
      }
    });
    event.stopWatching();
  });
},

// test/test_todoFactory.js
contract('TodoFactory', function(accounts) {
  it('Should delete todo properly', async () => {
    await contract.deleteTodo(todoId1);

    await utils.assertEvent(contract, { event: 'OnTodoDeleted', args: { todoId: todoId1 } });
  });
}
```
接著執行以下指令後可以得到測試的結果
```bash
truffle test
```

講完 DApp「後端」的部分後，接下來是利用前端來和智慧合約互動

## Web3.js
[Web3.js](https://github.com/ethereum/web3.js/) 提供 javascript 用來和以太坊互動的 API，這邊使用的版本是 v1.0，v1.0 與之前的版本有相當大的差別，除了額外提供 Socket 接口監聽事件，API 的呼叫方式也完全不同，甚至有些連運作的邏輯也不同，所以在查詢資料上需要特別注意這點。前端的功能除了傳統 Todo App 的新增、刪除、標記完成任務的功能之外，還可以列出這個 DAPP 過去的操作紀錄。但這邊只會強調與 web3 相關的接口部分，其餘的部分請看完整的[程式碼](https://github.com/pomelyu/EthereumTodo.git)，運作流程如下：

<img src="/img/cychien/dapp-todo-data-flow.png" alt="data-flow" style="width: 600px;"/>

### Step1. 安裝 web3
```bash
npm install web3
```

### Step2. 初始化 web3
```javascript
// ethereum-todo/src/config/config-web3.js

// 使用 websocket
// localhost:8545 是利用 truffle 建立的測試用區塊鏈
// const web3 = new Web3(new Web3.providers.WebsocketProvider('ws://localhost:8545')) 
const web3 = new Web3('ws://localhost:8545');

// or 使用 http
// const web3 = new Web3(new Web3.providers.HttpProvider('ws://localhost:8545')) 
// const web3 = new Web3('http://localhost:8545');
```
除了自己架設測試的區塊鏈外，也可以使用[公開的測試區塊鏈](https://medium.com/taipei-ethereum-meetup/ethereum-智能合約開發筆記-不用自己跑節點-使用-infura-和-web3-js-呼叫合約-2b8c852ed3d2)。

### Step3. 初始化合約
這邊需要用到合約的 ABI（Application Binary Interface） 以及在區塊鏈上佈署的位址，ABI 就是紀錄合約中使用到的函數和變數的文件，[這裡](https://medium.com/taipei-ethereum-meetup/ethereum-智能合約開發筆記-深入智能合約-abi-268ececb70ae)有更詳細的說明，首先先將前一個部分編譯後的檔案（`build/contracts/TodoFactory.json`）複製到專案資料夾
```javascript
// ethereum-todo/src/contracts/todoContract
import web3 from 'config/config-web3';
import TodoFactoryJSON from './TodoFactory.json';

const CONTRACT_ADDRESS = '0x21e4624c5a0b3fda81d0833d412dded2bb3a7a7c';
const todoContract = new web3.eth.Contract(TodoFactoryJSON.abi, CONTRACT_ADDRESS);

export default todoContract;
```
這樣就建立一個合約的實體可供操作

### Step4. 執行合約
重複前面提到的，**Transaction** 合約的執行需要消耗 **gas**，所以我們需要有一個帳號來花費 **gas** 執行這些合約，可以從 truffle 建立的區塊鏈中找到測試用的帳號（也就是執行時建立的那十個），並藉著 web3 提供的 api 查看帳號擁有的 **gas**：
```javascript
// ethereum-todo/src/helpers/accountsHelper/balance.js
import web3 from 'config/config-web3';

export async function getBalanceAsync(address) {
  const balance = await web3.eth.getBalance(address);
  return balance;
}

// 接者可以在任何地方使用 getBalanceAsync 來取得特定使用者目前的 gas 量，例如
// const DEFAULT_USER = '0x1d489c3f8ed5ee71325a847888b2157c9ac29c05';
//
// void async function() {
//   const balance = await getBalanceAsync(DEFAULT_USER);
//   console.log('Balance of account0', balance);
// }()
```

在 web3 中 **Call** 和 **Transaction** 分別對應 `contract.methods.myMethod.call` 和 `contract.methods.myMethod.send` 兩種呼叫方式，後者在之前的版本是 `sendTransaction`。兩者的使用如下
```javascript
// ethereum-todo/src/helpers/todoHelpers/todoAction.js
import todoContract from 'contracts/todoContract';

// 第一個測試帳號的 gas 數目相當多，很適合用來測試合約的執行
const DEFAULT_USER = '0x1d489c3f8ed5ee71325a847888b2157c9ac29c05';

export async function getTodoAsync(todoId) {
  // 估計需要消耗的 gas
  const gas = await todoContract.methods.getTodo(todoId).estimateGas();
  console.log('Get Todo: Estimated gas', gas);
  // 因為 getTodo 不會修改到資料，所以用 call 
  const result = await todoContract.methods.getTodo(todoId).call({
    from: DEFAULT_USER,
    gas:200000,
  });
  return result;
}

export async function addTodoAsync(taskName) {
  // 估計需要消耗的 gas，因為必須寫入字串，所以很可能會消耗超過預設值 90000 的 gas，
  // 故調高 gas limit 到 200000
  const gas = await todoContract.methods.addTodo(taskName).estimateGas();
  console.log('AddTodo: Estimated gas', gas);
  // 因為 addTodo 會修改資料，所以必須用 send
  // 如果這邊改成 call，依然可以執行，但不會有資料的寫入
  await todoContract.methods.addTodo(taskName).send({
    from: DEFAULT_USER,
    gas: 200000,
  });
}
```

無論用 `call` 或是 `send` 都可以指定消耗的 **gas** 最大值（稱為 gas limit 這裡是 200000），gas limit 的設計是為了防止智慧合約在執行時產生無窮迴圈的情形，因為所有的運算都需要消耗 gas，一旦消耗 gas 的總量到達 gas limit，就會終止執行。

### Step5. 監聽執行的結果
還是一樣 `Transaction` 的執行需要等到礦工們寫入資料才算真的完成，因此只能利用監聽事件的方式來確定。在這個專案中將這些監聽的處理放在 `src/events` 資料夾下，未來或許放在 middleware 是比較漂亮的方式。
```javascript
// ethereum-todo/src/events/todoEvents.js
import todoContract from 'contracts/todoContract';
import * as todoHelper from 'helpers/todoHelpers';
import { addTodo, deleteTodo, completeTodo } from 'containers/App/duck/todo';

import store from '../store';

// Event
// OnTodoAdded 是 event 的名稱
todoContract.events.OnTodoAdded({
}, async (error, result) => {
  if (error) {
    console.log(error);
    return;
  }
  // result
  // {
  //   raw: {
  //     data: "0x0000000000000000000000000000000000000000000000000000000000000001",
  //     topics: ["0x6edbfebf4adc3e180444860a21cd838446f00049410a44c6ec4a178a2ebe529b"]
  //   },
  //   returnValues: {
  //     todoId: 1
  //   },
  //   address: "0x21e4624c5A0B3fdA81D0833d412DDED2bb3A7a7C", // 合約的 address
  //   blockHash: "0x9186b52740bff34239c92137ae1ecb7205a028b540e30b79256b13b829354252",
  //   blockNumber: 61,
  //   event: "OnTodoAdded",
  //   signature: "0x6edbfebf4adc3e180444860a21cd838446f00049410a44c6ec4a178a2ebe529b",
  //   transactionHash: "0x997195a40d9ad102b0c18b730711fec0623596643c5820253296415b816563a3",
  //   transactionIndex: 0,
  //   type: "mined"
  // }
  const { returnValues: { todoId } } = result;

  // 因為事件的回傳值只有 todoId，因此還需要取得完整的 todo 資料。
  const todo = await todoHelper.getTodoAsync(todoId);
  store.dispatch(addTodo(todoId, todo[0], todo[1]));
  console.log('Add', todoId);
});
```

### Step6. 獲取過去的事件
區塊鏈可以視為一個保存操作紀錄並且確保這些紀錄無法被竄改的資料庫，因此上述的操作事件都會被紀錄在區塊鏈上，web3 提供 `getPastEvents` 這個 api 來取得過去的事件（web3 在 v1.0 版本前要取得過去的事件需要持續監聽，而非直接傳回結果）
```javascript
// ethereum-todo/src/helpers/todoHelpers/eventLogs.js
import todoContract from 'contracts/todoContract';

export async function getAllEventsAsync() {
  const events = await todoContract.getPastEvents('allEvents', {
    // 也就是取得從第一個區塊到最新區塊的所有事件
    fromBlock: 0,
    toBlock: 'latest'
  });
  // events 是一個陣列，陣列元素與前述監聽事件的回傳值相同
  return events;
}
```

透過以上的兩個部分已經可以利用智慧合約在區塊鏈上寫一個 Todo DApp，不過使用的方式仍然相當侷限，也忽略不少實際上可能會碰到的問題，事實上光是如何適當的儲存資料在區塊鏈上就是一大挑戰，這個專案可以當作簡單基底，繼續深入研究。

## 參考資料
- [Ethereum區塊鏈！智能合約(Smart Contract)與分散式網頁應用(DApp)入門](https://www.gitbook.com/book/gasolin/learn-ethereum-dapp/details)
- [What is different between a transaction and a call](https://ethereum.stackexchange.com/questions/765/what-is-the-difference-between-a-transaction-and-a-call)
- [比較 requre, assert, 和 revert 及其運作的方式](https://medium.com/taipei-ethereum-meetup/比較-require-assert-和-revert-及其運作方式-30c24d534ce4)
- [How to listen for contract events in javascript tests](https://ethereum.stackexchange.com/questions/15353/how-to-listen-for-contract-events-in-javascript-tests)
- [在公開測試鏈上部署合約](https://medium.com/taipei-ethereum-meetup/ethereum-智能合約開發筆記-不用自己跑節點-使用-infura-和-web3-js-呼叫合約-2b8c852ed3d2)。
- [深入智能合約 ABI](https://medium.com/taipei-ethereum-meetup/ethereum-智能合約開發筆記-深入智能合約-abi-268ececb70ae)

關於作者： 
@cychien 喜愛閱讀，熱愛動手作，相信未來建立在對歷史的理解上
