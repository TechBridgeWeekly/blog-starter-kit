---
title: 深入 Session 與 Cookie：Express、PHP 與 Rails 的實作
date: 2019-09-07 02:05:40
tags:
    - session
    - cookie
    - web
author: huli
---

## 前言

這是一系列共三篇的文章，我稱之為 Session 與 Cookie 三部曲。系列文的目標是想要由淺入深來談談這個經典議題，從理解概念一直到理解實作方式。這是系列文的最後一篇，三篇的完整連結如下：

1. [白話 Session 與 Cookie：從經營雜貨店開始](https://medium.com/@hulitw/session-and-cookie-15e47ed838bc)
2. [淺談 Session 與 Cookie：一起來讀 RFC](https://github.com/aszx87410/blog/issues/45)
3. [深入 Session 與 Cookie：Express、PHP 與 Rails 的實作
](https://github.com/aszx87410/blog/issues/46)

第一篇以白話的方式來談 Session 與 Cookie，全篇沒有談到太多技術名詞；第二篇直接去看 Cookie 的三份 RFC 來理解到底什麼是 Session，也補齊了一些 Cookie 相關的知識。而這一篇則是要深入 Session，一起帶大家看看三種不同的 Session 實作方式。

這三樣分別是 Node.js 的 Web 框架 Express、PHP 以及 Ruby on Rails。會挑選這三個是因為他們對於 Session 機制的實作都不同，是我覺得很適合拿來參考的對象。

好，接著就開始吧！

## Express

[Express](https://expressjs.com/) 本身是個極度輕量的框架，有許多其他框架底下的基本功能，在這邊都要額外安裝 middleware 才能使用。

先來簡單介紹一下 middleware 的概念。在 Express 裡面，當收到一個 Request  之後就會轉交給相對應的 middleware 來做處理，處理完以後變成 Response 回傳回去。所以 Express 的本質其實就是一大堆 middleware。

用圖解釋的話會長這樣：

![螢幕快照 2019-08-08 下午11 22 26](https://user-images.githubusercontent.com/2755720/62776748-10456480-bade-11e9-8000-6604aca08c8c.png)

舉個例子好了，一段基本的程式碼會長這樣：

``` js
const express = require('express')
const app = express()
const port = 5001
  
// global 的 middleware
app.use((req, res, next) => {
  req.greeting = 'hello'
  next()
})
  
// 特定 route 的 middleware
app.get('/', (req, res) => {
  res.end(req.greeting)
})
  
app.listen(port, () => {
  console.log(`Example app listening on port ${port}!`)
})
```

第一個 middleware 是 global 的，所以任何 request 都會先到達這個 middleware，而這邊可以對 req 或是 res 這兩個參數設置一些東西，最後呼叫 `next` 把控制權轉給下一個 middleware。

而下一個 middleware 就可以拿到前面的 middleware 處理過後的資訊，並且輸出內容。如果沒有呼叫 next，代表不想把控制權轉移給下個 middleware。

在 Express 裡面，管理 Session 的 middleware 是 [express-session](https://github.com/expressjs/session)，範例程式碼長這樣（改寫自官網範例）：

``` js
const express = require('express')
const session = require('express-session')
  
const app = express()
const port = 5001
  
// 使用 session middleware
app.use(session({
  secret: 'keyboard cat'
}))
   
app.get('/', function(req, res, next) {
  
  // 可以用 req.session 拿取存在 session 的值
  // 這邊判斷有沒有 req.session.views
  // 如果有的話就 +1，反之初始化成 1
  // 所以 req.session 可讀也可寫
  if (req.session.views) {
    req.session.views++
    res.write('views: ' + req.session.views)
    res.end()
  } else {
    req.session.views = 1
    res.end('welcome to the session demo. refresh!')
  }
})
  
app.listen(port, () => {
  console.log(`Example app listening on port ${port}!`)
})
```

使用了 session middleware 以後，可以直接用 `req.session.key` 來存取你要的資訊，同一個變數可以寫入也可以讀取，跟 PHP 的 $_SESSION 有異曲同工之妙。

接著我們來看看 express-session 的程式碼吧！主要的程式碼都在 [index.js](https://github.com/expressjs/session/blob/master/index.js) 這個檔案，大概有快七百行，不太可能一行一行講解。

而且寫得好的 library，會花很多精力在向後相容以及資料合法性的檢查，這些都是一些比較瑣碎而且對於想要理解機制比較沒幫助的東西。

所以我會直接把程式碼稍微整理一下，去除掉比較不重要的部分並且重新組織程式碼，只挑出相關的段落。

我們會關注三個重點：

1. sessionID 如何產生
2. sessionID 儲存方式
2. session 資訊儲存方式

可以先來看產生 sessionID 的地方：

``` js
// get the session id generate function
var generateId = opts.genid || generateSessionId
  
// generates the new session
store.generate = function(req){
  req.sessionID = generateId(req);
  req.session = new Session(req);
  req.session.cookie = new Cookie(cookieOptions);
  
  if (cookieOptions.secure === 'auto') {
    req.session.cookie.secure = issecure(req, trustProxy);
  }
};
  
function generateSessionId(sess) {
  return uid(24);
}
```

express-session 的客製化程度很高，可以自己傳進去產生 sessionID 的函式。若是沒有傳，預設會使用 `uid(24)`，這邊的 uid 指的是 [uid-safe](https://github.com/crypto-utils/uid-safe) 這個 library，會產生一個長度為 24 bytes 的隨機 ID。

文件上有特別說明這個長度：

> Asynchronously create a UID with a specific byte length. Because base64 encoding is used underneath, this is not the string length. For example, to create a UID of length 24, you want a byte length of 18.

所以填入 24，最後產生出來的會是長度為 32 個字元的字串。

那這個 sessionID 是以什麼樣的形式存進 Cookie 的呢？

``` js
var cookie = require('cookie')
var signature = require('cookie-signature')
  
// get the session cookie name
var name = opts.name || opts.key || 'connect.sid'
  
// get the cookie signing secret
var secret = opts.secret
  
if (secret && !Array.isArray(secret)) {
  secret = [secret];
}
  
// set-cookie
onHeaders(res, function(){
  
  // set cookie
  setcookie(res, name, req.sessionID, secrets[0], req.session.cookie.data);
});
  
function setcookie(res, name, val, secret, options) {
  var signed = 's:' + signature.sign(val, secret);
  var data = cookie.serialize(name, signed, options);
  
  debug('set-cookie %s', data);
  
  var prev = res.getHeader('Set-Cookie') || []
  var header = Array.isArray(prev) ? prev.concat(data) : [prev, data];
  
  res.setHeader('Set-Cookie', header)
}
```

存在 cookie 裡面的 sessionID 的 key 一樣可以自己指定，但預設會是 `connect.sid`，所以以後一看到這個 key 就知道這是 express-session 預設的 sessionID 名稱。

內容的部分比較特別一點，會以 `s:` 開頭，後面接上 `signature.sign(sessionID, secret)` 的結果。

這邊要再看到 [cookie-signature](https://github.com/tj/node-cookie-signature) 這個 library，底下是一個簡單範例：

``` js
var cookie = require('cookie-signature');
  
var val = cookie.sign('hello', 'tobiiscool');
val.should.equal('hello.DGDUkGlIkCzPz+C0B064FNgHdEjox7ch8tOBGslZ5QI');
```

這邊的 sign 到底做了什麼呢？原始碼很簡單，可以稍微看一下：

``` js
var crypto = require('crypto');
  
/**
 * Sign the given `val` with `secret`.
 *
 * @param {String} val
 * @param {String} secret
 * @return {String}
 * @api private
 */
  
exports.sign = function(val, secret){
  if ('string' != typeof val) throw new TypeError("Cookie value must be provided as a string.");
  if ('string' != typeof secret) throw new TypeError("Secret string must be provided.");
  return val + '.' + crypto
    .createHmac('sha256', secret)
    .update(val)
    .digest('base64')
    .replace(/\=+$/, '');
};
```

就只是把你要 sign 的內容用 hmac-sha256 產生一個[鑑別碼](https://zh.wikipedia.org/wiki/%E9%87%91%E9%91%B0%E9%9B%9C%E6%B9%8A%E8%A8%8A%E6%81%AF%E9%91%91%E5%88%A5%E7%A2%BC)，並且加在字串後面而已，中間會用`.`來分割資料。

若是你不知道什麼是 hmac 的話我稍微提一下，簡單來說就是可以對一串訊息產生鑑別碼，目的是為了保持資料的完整性讓它不被竄改。你可以想成它就是訊息對應到的一組獨一無二的代碼，如果訊息被改掉了，代碼也會不一樣。

以上面的範例來說，`hello` 利用 `tobiiscool` 這個 secret，得到的結果為：`DGDUkGlIkCzPz+C0B064FNgHdEjox7ch8tOBGslZ5QI`，於是完整字串就變為：`hello.DGDUkGlIkCzPz+C0B064FNgHdEjox7ch8tOBGslZ5QI`，前面是我的資料，後面是資料的鑑別碼。

如果有人想竄改資料，例如說把前面改成 hello2，那這個資料的鑑別碼就不會是後面那一串，我就知道有人篡改資料了。所以藉由這樣的方式來保持資料完整性，其實原理跟 [JWT](https://jwt.io/) 是差不多的，你看得到資料但沒辦法改它，因為改了會被發現。

你可能會疑惑說：那我幹嘛不把整個 sessionID 加密就好？為什麼要多此一舉用這種方式？我自己猜測是因為原始資料其實不怕別人看，只是怕人改而已；若是原始資料是敏感資訊，會用加密的方式。但因為原始資料只是 sessionID 而已，被別人看到也沒什麼關係，只要保障資料完整性即可。而且加密需要的系統資源應該比這種訊息驗證還多，因此才採用這種方式。

好，我們再講回來前面，所以 express-session 會把 sessionID 存在 cookie 裡面，key 是 `connect.sid`，value 則是 `s:{sessionID}.{hmac-sha256(sessionID, secret)}`。

好奇的話你可以去任何使用 Express 的網站然後看一下 cookie 內容，就可以找到實際的資料（或是自己隨便執行一個也行），這邊我用我的當作範例，我的 connect.sid 是： s%3AfZZVCDHefchle2LDK4PzghaR3Ao9NruG.J%2BsOPkTubkeMJ4EMBcnunPXW0Y7TWTucRSKIPNVgnRM，把特殊字元 decode 之後變成： `s:fZZVCDHefchle2LDK4PzghaR3Ao9NruG.J+sOPkTubkeMJ4EMBcnunPXW0Y7TWTucRSKIPNVgnRM`。

也就是說我的 sessionID 是`fZZVCDHefchle2LDK4PzghaR3Ao9NruG`，鑑別碼是`J+sOPkTubkeMJ4EMBcnunPXW0Y7TWTucRSKIPNVgnRM`。

知道儲存 sessionID 的方式以後，從 cookie 裡面取得 sessionID 的方式應該也能看懂，就是把事情反過來做而已：

``` js
// get the session ID from the cookie
var cookieId = req.sessionID = getcookie(req, name, secrets);
  
function getcookie(req, name, secrets) {
  var header = req.headers.cookie;
  var raw;
  var val;
  
  // read from cookie header
  if (header) {
    var cookies = cookie.parse(header);
  
    raw = cookies[name];
  
    if (raw) {
      if (raw.substr(0, 2) === 's:') {
        val = unsigncookie(raw.slice(2), secrets);

        if (val === false) {
          debug('cookie signature invalid');
          val = undefined;
        }
      } else {
        debug('cookie unsigned')
      }
    }
  }
  
  return val;
}
  
/**
 * Verify and decode the given `val` with `secrets`.
 *
 * @param {String} val
 * @param {Array} secrets
 * @returns {String|Boolean}
 * @private
 */
function unsigncookie(val, secrets) {
  for (var i = 0; i < secrets.length; i++) {
    var result = signature.unsign(val, secrets[i]);
  
    if (result !== false) {
      return result;
    }
  }
  
  return false;
}
```

接下來就剩最後一個了，session 資訊到底存在哪裡？是存在記憶體、檔案，還是其他地方？

其實這個在程式碼裡面寫得很清楚了，預設是存在記憶體裡面的：

``` js
var warning = 'Warning: connect.session() MemoryStore is not\n'
  + 'designed for a production environment, as it will leak\n'
  + 'memory, and will not scale past a single process.';
  
// get the session store
var store = opts.store || new MemoryStore()
  
// notify user that this store is not
// meant for a production environment
/* istanbul ignore next: not tested */
if (env === 'production' && store instanceof MemoryStore) {
  console.warn(warning);
}
```

那到底是怎麼存呢？可以參考 [session/memory.js](https://github.com/expressjs/session/blob/master/session/memory.js)：

``` js
function MemoryStore() {
  Store.call(this)
  this.sessions = Object.create(null)
}
  
MemoryStore.prototype.get = function get(sessionId, callback) {
  defer(callback, null, getSession.call(this, sessionId))
}
  
MemoryStore.prototype.set = function set(sessionId, session, callback) {
  this.sessions[sessionId] = JSON.stringify(session)
  callback && defer(callback)
}
  
function getSession(sessionId) {
  var sess = this.sessions[sessionId]
  
  if (!sess) {
    return
  }
  
  // parse
  sess = JSON.parse(sess)
  
  return sess
}
```

首先用 `Object.create(null)` 創造出一個乾淨的 Object（這是很常用的一個方法，沒看過的可以參考：[詳解 Object.create(null)](https://juejin.im/post/5acd8ced6fb9a028d444ee4e)），然後以 sessionID 作為 key，`JSON.stringigy(session)`作為 value，存到這個 object 裡面。

所以說穿了其實 express-session 的 session information 預設就是存在一個變數裡面而已啦，因此你只要一把 process 結束掉重開，session 的資料就都全部不見了。而且會有 memory leak 的問題，所以官方也不推薦用在 production 上面。

如果要用在 production 上面，必須額外再找`store`來用，例如說 [connect-redis](https://github.com/tj/connect-redis#readme) 就可以跟 express-session 搭配，把 session information 存在 redis 裡。

以上就是 Express 常用的 middleware：express-session 的原始碼分析。從上面的段落我們清楚知道了 sessionID 的產生方式以及如何存在 cookie，還有 session information 所儲存的地方。

## PHP（7.2 版本）

PHP 內建就有 session 機制，不必使用任何的 framework，而使用的方法也很簡單：

``` php
<?php
session_start();
  
if (empty($_SESSION['views'])) {
    $_SESSION['views'] = 1;
} else {
    $_SESSION['views']++;
}
  
echo $_SESSION['views'];
?>
```

其實跟 express-session 的用法有點像，只是一個是 `req.session`，一個是`$_SESSION`。

我原本也想跟剛剛看 express-session 一樣，直接去看 PHP 的原始碼，然後從中發現如何實作。但因為 PHP 的原始碼全部都是 C，對我這種幾乎沒寫過 C 的人來說很難看懂，因此我也只能反過來。先跟大家介紹 PHP 的 Session 機制是如何實作的，再從原始碼裡面去找證據支援。

首先呢，PHP 的 Session 機制與 express-session 差不多，都會在 Cookie 裡存放一個 sessionID，並且把 session information 存在伺服器。express-session 預設是存在記憶體，PHP 預設則是存在檔案裡面。

以上這些都可以在 PHP 的設定檔調整，都寫在 `php.ini` 裡面，底下以我的為例，列出一些相關的設定：

``` ini
[Session]
; Handler used to store/retrieve data.
; http://php.net/session.save-handler
session.save_handler=files
  
; Argument passed to save_handler.  In the case of files, this is the path
; where data files are stored. Note: Windows users have to change this
; variable in order to use PHP's session functions.
;
; The path can be defined as:
;
;     session.save_path = "N;/path"
  
session.save_path="/opt/lampp/temp/"
  
; Name of the session (used as cookie name).
; http://php.net/session.name
session.name=PHPSESSID
  
; Handler used to serialize data.  php is the standard serializer of PHP.
; http://php.net/session.serialize-handler
session.serialize_handler=php
```

在 Cookie 裡面你就能看見一個 `PHPSESSID`，值大概長得像這樣：`fc46356f83dcf5712205d78c51b47c4d`，這就是 PHP 所使用的 sessionID。

接著你去 `session.save_path` 看，就會看到儲存你 session 資訊的檔案，檔名很好認，就是 `sess_` 加上 sessionID：

```
root@debian:/opt/lampp/temp# ls
  
adminer.invalid
adminer.version
sess_04719a35fb67786d574ec6eca969f7cb
sess_fc46356f83dcf5712205d78c51b47c4d
```

若是打開 session 檔案，內容會是被序列化（serialize）之後的結果：

```
views|i:5;
```

這就是 PHP session 的真面目了。把 session information 全都存在檔案裡面。

若是想要研究 PHP session 的相關原始碼，最重要的檔案就是這兩個：[ext/session/session.c](https://github.com/php/php-src/blob/PHP-7.2/ext/session/session.c) 跟 [ext/session/mod_files.c](https://github.com/php/php-src/blob/PHP-7.2/ext/session/mod_files.c)，前者管理 session 生命週期，後者負責把 session 實際存到檔案裡面或者是讀出來。後者其實就很像我們在 express-session 裡面看到的 Store，只要遵守一樣的 interface，就可以自己寫一個其他的 mod 出來，例如說 mod_redis.c 之類的。

接著我們一樣先來找找看 sessionID 是如何產生的，可以直接在 mod_files.c 搜尋相關字眼，就會找到底下這段：

``` c
/*
 * Create session ID.
 * PARAMETERS: PS_CREATE_SID_ARGS in php_session.h
 * RETURN VALUE: Valid session ID(zend_string *) or NULL for FAILURE.
 *
 * PS_CREATE_SID_FUNC() must check collision. i.e. Check session data if
 * new sid exists already.
 * *mod_data is guaranteed to have non-NULL value.
 * NOTE: Default php_session_create_id() does not check collision. If
 * NULL is returned, session module create new ID by using php_session_create_id().
 * If php_session_create_id() fails due to invalid configuration, it raises E_ERROR.
 * NULL return value checks from php_session_create_id() is not required generally.
 */
PS_CREATE_SID_FUNC(files)
{
    zend_string *sid;
    int maxfail = 3;
    PS_FILES_DATA;
  
    do {
        sid = php_session_create_id((void**)&data);
        if (!sid) {
            if (--maxfail < 0) {
                return NULL;
            } else {
                continue;
            }
        }
        /* Check collision */
        /* FIXME: mod_data(data) should not be NULL (User handler could be NULL) */
        if (data && ps_files_key_exists(data, ZSTR_VAL(sid)) == SUCCESS) {
            if (sid) {
                zend_string_release(sid);
                sid = NULL;
            }
            if (--maxfail < 0) {
                return NULL;
            }
        }
    } while(!sid);
  
    return sid;
}
```

這邊呼叫了 `php_session_create_id` 來產生 sessionID，然後會檢查有沒有產生重複的 id，有的話就重試最多三次。而 `php_session_create_id` 則是存在於 session.c 那個檔案：

``` c
#define PS_EXTRA_RAND_BYTES 60
  
PHPAPI zend_string *php_session_create_id(PS_CREATE_SID_ARGS) /* {{{ */
{
    unsigned char rbuf[PS_MAX_SID_LENGTH + PS_EXTRA_RAND_BYTES];
    zend_string *outid;
  
    /* Read additional PS_EXTRA_RAND_BYTES just in case CSPRNG is not safe enough */
    if (php_random_bytes_throw(rbuf, PS(sid_length) + PS_EXTRA_RAND_BYTES) == FAILURE) {
        return NULL;
    }
  
    outid = zend_string_alloc(PS(sid_length), 0);
    ZSTR_LEN(outid) = bin_to_readable(rbuf, PS(sid_length), ZSTR_VAL(outid), (char)PS(sid_bits_per_character));
  
    return outid;
}
```

重點其實只有這一個：`php_random_bytes_throw`，這個 function 如果繼續追下去會找到 [ext/standard/php_random.h](https://github.com/php/php-src/blob/623911f993f39ebbe75abe2771fc89faf6b15b9b/ext/standard/php_random.h#L32)，然後找到 [ext/standard/random.c](https://github.com/php/php-src/blob/8fc58a1a1d32dd288bf4b9e09f9302a99d7b35fe/ext/standard/random.c#L89)，才是真正產生隨機數的地方。

但最後找到的那個 function 想要看懂必須花一大段時間，因此我就沒有細看了。總之在不同作業系統上會有不同的產生方式，其中一種還會使用到 [/dev/urandom](https://zh.wikipedia.org/wiki//dev/random)。

知道了 sessionID 的產生方式以後，我們來看看 PHP 的 session information 是怎麼做 serialize 的。可以在[官方文件](https://www.php.net/manual/en/function.session-encode.php)上看到一個 function 叫做：`session_encode`，輸出的結果跟我們在 session 檔案裡面看到的資料一模一樣，而這個 function 的敘述寫著：

> session_encode() returns a serialized string of the contents of the current session data stored in the $_SESSION superglobal.

> By default, the serialization method used is internal to PHP, and is not the same as serialize(). The serialization method can be set using session.serialize_handler.

接著我們直接在 session.c 裡面搜尋`session_encode`，會找到這一段：

``` c
/* {{{ proto string session_encode(void)
   Serializes the current setup and returns the serialized representation */
static PHP_FUNCTION(session_encode)
{
    zend_string *enc;
  
    if (zend_parse_parameters_none() == FAILURE) {
        return;
    }
  
    enc = php_session_encode();
    if (enc == NULL) {
        RETURN_FALSE;
    }
  
    RETURN_STR(enc);
}
```

只是一個 `php_session_encode` 的 wrapper 而已，而且 `php_session_encode` 也只是再呼叫別的東西：

``` c
static zend_string *php_session_encode(void) /* {{{ */
{
    IF_SESSION_VARS() {
        if (!PS(serializer)) {
            php_error_docref(NULL, E_WARNING, "Unknown session.serialize_handler. Failed to encode session object");
            return NULL;
        }
        return PS(serializer)->encode();
    } else {
        php_error_docref(NULL, E_WARNING, "Cannot encode non-existent session");
    }
    return NULL;
}
/* }}} */
```

`return PS(serializer)->encode();` 這一句才是重點。其實追到這邊的時候就有點卡住，因為不清楚這邊的 `serializer` 是從哪邊來的。但往下稍微看一下程式碼，找到一段應該是相關的：

``` c
#define PS_DELIMITER '|'
  
PS_SERIALIZER_ENCODE_FUNC(php) /* {{{ */
{
    smart_str buf = {0};
    php_serialize_data_t var_hash;
    PS_ENCODE_VARS;
  
    PHP_VAR_SERIALIZE_INIT(var_hash);
  
    PS_ENCODE_LOOP(
        smart_str_appendl(&buf, ZSTR_VAL(key), ZSTR_LEN(key));
        if (memchr(ZSTR_VAL(key), PS_DELIMITER, ZSTR_LEN(key))) {
            PHP_VAR_SERIALIZE_DESTROY(var_hash);
            smart_str_free(&buf);
            return NULL;
        }
        smart_str_appendc(&buf, PS_DELIMITER);
        php_var_serialize(&buf, struc, &var_hash);
    );
  
    smart_str_0(&buf);
  
    PHP_VAR_SERIALIZE_DESTROY(var_hash);
    return buf.s;
}
/* }}} */
```

會知道相關是因為 `#define PS_DELIMITER '|'` 這一行，這個符號在 session 檔案裡有出現，可以猜測應該是拿來分隔什麼東西的。而實際的值則是交給`php_var_serialize`處理。

`php_var_serialize`若是繼續往下追，可以找到 [ext/standard/var.c](https://github.com/php/php-src/blob/7686b0b88906e2522300b9e631ddde2051de839f/ext/standard/var.c#L1112)（直接用 GitHub 搜尋功能就可以找到這個檔案，搜尋功能超方便的），最後就會找到真正在處理的地方：[php_var_serialize_intern](https://github.com/php/php-src/blob/7686b0b88906e2522300b9e631ddde2051de839f/ext/standard/var.c#L883)，裡面會針對不同的形態去呼叫不同的 function。

以我們之前存在 session 裡面的 views 來說，是一個數字，所以會跑到這個 function：

``` c
static inline void php_var_serialize_long(smart_str *buf, zend_long val) /* {{{ */
{
    smart_str_appendl(buf, "i:", 2);
    smart_str_append_long(buf, val);
    smart_str_appendc(buf, ';');
}
/* }}} */
```

追到這邊，就知道為什麼當初 session 序列化之後的結果是`views|i:5;`了。`|`拿來分隔 key 跟 value，i 代表著型態，5 代表實際的數字，; 則是結束符號。

以上就是 PHP Session 機制的相關原始碼分析，我們稍微看了如何產生 sessionID 以及 session information 如何做序列化。也知道了以預設的狀態來說，cookie 名稱會叫做 PHPSESSID，而且會以檔案的方式來儲存 session 的內容。

最後來分享兩個跟 PHP Session 有關的文章，都十分有趣：

1. [HITCON CTF 2018 - One Line PHP Challenge](https://blog.orange.tw/2018/10/hitcon-ctf-2018-one-line-php-challenge.html)
2. [[Web Security] 透過 LFI 引入 PHP session 檔案觸發 RCE](https://cyku.tw/lfi-leads-to-rce-via-session-file/)


## Rails（5.2 版本）

Rails 是一個 Ruby 的 Web 框架，俗稱 Ruby on Rails。會挑這一套是因為我本來就知道它儲存 session 的方法不太一樣。我當初只是好奇 Rails 怎麼生成 sessionID 的，於是就去 GitHub 的 repo 搜尋：session，然後找到這個檔案：[rails/actionpack/test/dispatch/session/cookie_store_test.rb](https://github.com/rails/rails/blob/5-2-stable/actionpack/test/dispatch/session/cookie_store_test.rb)，是個測試，但有時候測試其實對找程式碼幫助也很大，因為裡面會出現一堆相關的 function 跟參數。

我那時觀察了一陣子，發現裡面出現了很多次的 session_id，於是就改用這個關鍵字搜尋，找到了 [rails/actionpack/lib/action_dispatch/middleware/session/cookie_store.rb](https://github.com/rails/rails/blob/5-2-stable/actionpack/lib/action_dispatch/middleware/session/cookie_store.rb)，發現裡面的註解把 Rails 的 Session 實作方式寫得一清二楚：

``` ruby
# This cookie-based session store is the Rails default. It is
# dramatically faster than the alternatives.
#
# Sessions typically contain at most a user_id and flash message; both fit
# within the 4K cookie size limit. A CookieOverflow exception is raised if
# you attempt to store more than 4K of data.
#
# The cookie jar used for storage is automatically configured to be the
# best possible option given your application's configuration.
#
# If you only have secret_token set, your cookies will be signed, but
# not encrypted. This means a user cannot alter their +user_id+ without
# knowing your app's secret key, but can easily read their +user_id+. This
# was the default for Rails 3 apps.
#
# Your cookies will be encrypted using your apps secret_key_base. This
# goes a step further than signed cookies in that encrypted cookies cannot
# be altered or read by users. This is the default starting in Rails 4.
#
# Configure your session store in <tt>config/initializers/session_store.rb</tt>:
#
#   Rails.application.config.session_store :cookie_store, key: '_your_app_session'
#
# In the development and test environments your application's secret key base is
# generated by Rails and stored in a temporary file in <tt>tmp/development_secret.txt</tt>.
# In all other environments, it is stored encrypted in the
# <tt>config/credentials.yml.enc</tt> file.
#
# If your application was not updated to Rails 5.2 defaults, the secret_key_base
# will be found in the old <tt>config/secrets.yml</tt> file.
#
# Note that changing your secret_key_base will invalidate all existing session.
# Additionally, you should take care to make sure you are not relying on the
# ability to decode signed cookies generated by your app in external
# applications or JavaScript before changing it.
#
# Because CookieStore extends Rack::Session::Abstract::Persisted, many of the
# options described there can be used to customize the session cookie that
# is generated. For example:
#
#   Rails.application.config.session_store :cookie_store, expire_after: 14.days
#
# would set the session cookie to expire automatically 14 days after creation.
# Other useful options include <tt>:key</tt>, <tt>:secure</tt> and
# <tt>:httponly</tt>.
```

Rails 預設使用 cookie-based session，因為它比其他解決方案都來得快。雖然 cookie 有大小限制，但頂多只會存 flash message 跟 user_id，離 4k 的上限還有一大段距離。

在 Rails 3 裡面 cookie 只會被 signed 不會被加密，意思就是使用者看得到 user_id 但沒辦法改它（就像我們在 express-session 看到的 sessionID 一樣，看得到但不能改）。

而 Rails 4 以後預設就會把 cookie 的值整個加密，什麼都看不到。在測試環境時 Rails 會自動幫你產生一個 secret 來加密，也可以透過 Rails 的設定檔來設定。

在這份檔案中也可以看到有一個 function 叫做`generate_sid`，是拿來產生 sessionID 的。這個 function 存在於 [rails/actionpack/lib/action_dispatch/middleware/session/abstract_store.rb](https://github.com/rails/rails/blob/5-2-stable/actionpack/lib/action_dispatch/middleware/session/abstract_store.rb)：

``` ruby
def generate_sid
    sid = SecureRandom.hex(16)
    sid.encode!(Encoding::UTF_8)
    sid
end
```

直接呼叫了 Ruby 的函式庫 [SecureRandom](https://ruby-doc.org/stdlib-2.5.1/libdoc/securerandom/rdoc/SecureRandom.html) 來產生亂數並當作 sessionID。

至於在 Cookie 裡面的 key 是什麼，可以經由設定 `app.config.session_store` 來調整。根據[這邊](https://github.com/rails/rails/blob/5-2-stable/railties/lib/rails/application/finisher.rb#L39)的程式碼：

``` ruby
# Setup default session store if not already set in config/application.rb
initializer :setup_default_session_store, before: :build_middleware_stack do |app|
    unless app.config.session_store?
        app_name = app.class.name ? app.railtie_name.chomp("_application") : ""
        app.config.session_store :cookie_store, key: "_#{app_name}_session"
    end
end
```

預設值會是 `_#{app_name}_session`，例如說我的 app_name 叫做 huli，Cookie 名稱就會是 _huli_session。

然後把 session information 實際寫進去 cookie 的地方在 [rails/actionpack/lib/action_dispatch/middleware/session/cookie_store.rb](https://github.com/rails/rails/blob/5-2-stable/actionpack/lib/action_dispatch/middleware/session/cookie_store.rb)：

``` ruby
def set_cookie(request, session_id, cookie)
  cookie_jar(request)[@key] = cookie
end

def get_cookie(req)
  cookie_jar(req)[@key]
end

def cookie_jar(request)
  request.cookie_jar.signed_or_encrypted
end
```

會呼叫與 cookie 相關的 `signed_or_encrypted` 來做處理。

接著我去搜了一下文件，發現其實[官方文件](https://guides.rubyonrails.org/security.html#sessions)都寫得十分清楚了：

> The session ID is generated using SecureRandom.hex which generates a random hex string using platform specific methods (such as OpenSSL, /dev/urandom or Win32 CryptoAPI) for generating cryptographically secure random numbers. Currently it is not feasible to brute-force Rails' session IDs.

上面這段寫了 sessionID 的產生方式。

> The CookieStore uses the encrypted cookie jar to provide a secure, encrypted location to store session data. Cookie-based sessions thus provide both integrity as well as confidentiality to their contents. The encryption key, as well as the verification key used for signed cookies, is derived from the secret_key_base configuration value.
> 
> As of Rails 5.2 encrypted cookies and sessions are protected using AES GCM encryption. This form of encryption is a type of Authenticated Encryption and couples authentication and encryption in single step while also producing shorter ciphertexts as compared to other algorithms previously used. The key for cookies encrypted with AES GCM are derived using a salt value defined by the config.action_dispatch.authenticated_encrypted_cookie_salt configuration value.

這段則是寫說從 Rails 5.2 開始採用 AES GCM 來加密，底下還有一個段落我沒複製，主要是提到之前程式碼註解裡面寫的，Rails 4 前只用 HMAC 來做驗證，而不是加密。

而且我看一看之後發現這文件寫的好棒喔，除了把這些機制說明清楚以外，底下還介紹了我們上一篇提到的 Session Fixation Attack 以及 CSRF。

若是還想深入研究，可以參考 Rails 裡面 Cookie 相關的實作：[rails/actionpack/lib/action_dispatch/middleware/cookies.rb](https://github.com/rails/rails/blob/5-2-stable/actionpack/lib/action_dispatch/middleware/cookies.rb)，註解裡面有詳細的說明，例如說加密的部分：

``` ruby
# Returns a jar that'll automatically encrypt cookie values before sending them to the client and will decrypt them for read.
# If the cookie was tampered with by the user (or a 3rd party), +nil+ will be returned.
#  
# If +secret_key_base+ and +secrets.secret_token+ (deprecated) are both set,
# legacy cookies signed with the old key generator will be transparently upgraded.
#  
# If +config.action_dispatch.encrypted_cookie_salt+ and +config.action_dispatch.encrypted_signed_cookie_salt+
# are both set, legacy cookies encrypted with HMAC AES-256-CBC will be transparently upgraded.
#  
# This jar requires that you set a suitable secret for the verification on your app's +secret_key_base+.
#  
# Example:
#  
#   cookies.encrypted[:discount] = 45
#   # => Set-Cookie: discount=DIQ7fw==--K3n//8vvnSbGq9dA--7Xh91HfLpwzbj1czhBiwOg==; path=/
#  
#   cookies.encrypted[:discount] # => 45
def encrypted
  @encrypted ||= EncryptedKeyRotatingCookieJar.new(self)
end
```

往底下追的話就可以看到 `EncryptedKeyRotatingCookieJar` 的完整程式碼，或你也可以再往下，看看 [rails/activesupport/lib/active_support/message_encryptor.rb](https://github.com/rails/rails/blob/5-2-stable/activesupport/lib/active_support/message_encryptor.rb)，負責加密的程式碼長這樣：

``` ruby
def _encrypt(value, **metadata_options)
    cipher = new_cipher
    cipher.encrypt
    cipher.key = @secret
  
    # Rely on OpenSSL for the initialization vector
    iv = cipher.random_iv
    cipher.auth_data = "" if aead_mode?
  
    encrypted_data = cipher.update(Messages::Metadata.wrap(@serializer.dump(value), metadata_options))
    encrypted_data << cipher.final
  
    blob = "#{::Base64.strict_encode64 encrypted_data}--#{::Base64.strict_encode64 iv}"
    blob = "#{blob}--#{::Base64.strict_encode64 cipher.auth_tag}" if aead_mode?
    blob
end
```

這裡的 cipher 是從 openssl 來的，所以最底層是使用了 openssl。

整理到這邊應該就差不多了，就不再繼續深入了。

## 總結

在這篇裡面我們看了三個不同的 Session 儲存方式。第一種是 express-session，把 session information 存在記憶體裡面；第二種是 PHP，存在檔案裡面；最後一種則是 Rails，採用了之前提過的 cookie-based session，將資訊直接加密並且存在 cookie 裡。

在這系列當中，第一篇文章我們理解了概念，第二篇利用讀 RFC 加深印象並重新理解了一次 Session，最後一篇則是直接參考一些主流框架的實作，看看我們之前所提到的 sessionID 應該如何產生，session information 應該存在哪裡，cookie-bases session 又應該如何實作。

寫這系列的初衷就是想讓大家把這些概念一次理解清楚，就不用以後每次碰到都重新查一遍。

最後，希望這系列對大家有幫助，有任何錯誤都可以在底下留言反映。

底下是系列文的完整清單：

1. [白話 Session 與 Cookie：從經營雜貨店開始](https://medium.com/@hulitw/session-and-cookie-15e47ed838bc)
2. [淺談 Session 與 Cookie：一起來讀 RFC](https://github.com/aszx87410/blog/issues/45)
3. [深入 Session 與 Cookie：Express、PHP 與 Rails 的實作](https://github.com/aszx87410/blog/issues/46)

關於作者： 
[@huli](https://medium.com/@hulitw) 野生工程師，相信分享與交流能讓世界變得更美好
