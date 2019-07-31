---
title: 後端軟體工程工具箱：資料庫/SQL/ORM篇
date: 2017-05-06 23:00:00
tags: 
	- Python
	- PHP
	- Back End
	- Back End Development
	- Back End Engineer
	- Web
	- TCP
	- HTTP
	- UDP
author: kdchang
---
# 前言
事實上，要成為一個好的後端軟體工程師除了必須對於後端工程的程式語言的部份有所了解外，也需要了解系統設計的原理和伺服器規劃（Server 規劃、Load Balance、Memory Cache、DB Scaling、Cloud Server、後端框架、TCP/IP/UDP 網路通訊協定、HTTP 超文字傳輸協定、設計模式、資料庫操作、SQL/ORM、資訊安全、性能優化等）。接下來我們將來探討資料庫/SQL/ORM 相關議題。

# 資料庫基礎概念

一般持久性儲存資料是資訊系統或是應用程式蠻重要的一個環節，除了使用檔案系統外使用資料庫也是另外一個非常重要的儲存工具。比起檔案系統，資料庫雖然相對複雜（一般來說需要正規化）但在資料查詢使用和管理上比起檔案會方便許多。

在資料庫系統中，我們會稱資料庫操作的一個完整的邏輯過程為事務（transaction）。例如：銀行 ATM 轉帳，從原帳戶扣除金額，以及向目標帳戶新增金額，這兩個資料庫操作的總和，就會構成一個完整的邏輯過程，不可拆分。這個過程被稱為一個事務，具有 ACID 特性。

1. 什麼是 ACID？
	ACID 係指資料庫管理系統（DBMS）在寫入/更新資料的過程中，為保證事務（transaction）是正確可靠的，所必須具備的四個特性：原子性（atomicity，或稱不可分割性）、一致性（consistency）、隔離性（isolation，又稱獨立性）、持久性（durability）

	- 原子性：一個事務（transaction）中的所有操作，要麼全部完成，要麼全部不完成，不會結束在中間某個環節。事務在執行過程中發生錯誤，會被回滾（rollback）到事務開始前的狀態，就像這個事務從來沒有執行過一樣

	- 一致性：在事務開始之前和事務結束以後，資料庫的完整性沒有被破壞。這表示寫入的資料必須完全符合所有的預設規則，這包含資料的精確度、串聯性以及後續資料庫可以自發性地完成預定的工作

	- 隔離性：資料庫允許多個並發事務同時對齊數據進行讀寫和修改的能力，隔離性可以防止多個事務並發執行時由於交叉執行而導致數據的不一致。事務隔離分為不同級別，包括讀未提交（read uncommitted）、讀提交（read committed）、可重複讀（repeatable read）和序列化（serializable）

	- 持久性：事務處理結束後，對資料數據的修改就是永久的，即便系統故障也不會遺失

2. 資料庫並發控制（Concurrency control）
	資料庫管理系統（DBMS）中的並發控制的任務是確保在多個事務同時存取資料庫中同一數據時不破壞事務的隔離性（isolation）以及資料庫的統一性。下面舉例說明並發操作帶來的數據不一致性問題：

	現有兩處火車票售票點，同時讀取某一趟列車車票資料庫中車票餘額為 X。兩處售票點同時賣出一張車票，同時修改餘額為 X -1 寫回資料庫，這樣就造成了實際賣出兩張火車票而資料庫中的記錄卻只少了一張。

	產生這種情況的原因是因為兩個事務讀入同一數據並同時修改，其中一個事務提交的結果破壞了另一個事務提交的結果，導致其數據的修改遺失，破壞了事務的隔離性。並發控制要解決的就是這類問題。

	一般來說我們會使用以下幾種方式來解決：
	- 封鎖
		封鎖（lock）是一項用於多用戶同時存取資料庫的技術，是實作並行控制的一項重要手段，能夠防止當多用戶覆寫資料庫時造成資料遺失和損壞。當有一個用戶對資料庫內的資料進行操作時，在讀取資料前先鎖住資料，這樣其他用戶就無法存取和修改該資料，直到這一資料修改並寫回資料庫解除封鎖為止。

	- 時間戳
		時間戳（Timestamp）用於辨識記錄下來的時間日期的字串。國際標準為 ISO 8601

		```
		2005-10-30 T 10:45 UTC
		2007-11-09 T 11:20 UTC 
		Sat Jul 23 02:16:57 2005
		```

	- 樂觀並發控制
		樂觀並行控制（又稱「樂觀鎖」，Optimistic Concurrency Control，簡稱 OCC ）是一種並行控制的方法。它假設多用戶並行的交易在處理時不會彼此互相影響，各交易能夠在不產生鎖的情況下處理各自影響的那部分資料。在提交資料更新之前，每個交易會先檢查在該交易讀取資料後，有沒有其他交易又修改了該資料。如果其他交易有更新的話，正在提交的交易會進行回復

		讀取：交易將資料讀入快取，這時系統會給交易分派一個時間戳
		校驗：交易執行完畢後，進行提交。這時同步校驗所有交易，如果交易所讀取的資料在讀取之後又被其他交易修改，則產生衝突，交易被中斷回復
		寫入：通過校驗階段後，將更新的資料寫入資料庫
	
	- 悲觀並發控制
		悲觀並行控制（又稱「悲觀鎖」，Pessimistic Concurrency Control，簡稱 PCC ）是一種並行控制的方法。它可以阻止一個交易以影響其他用戶的方式來修改資料。如果一個交易執行的操作讀某行資料應用了鎖，那只有當這個交易把鎖釋放，其他交易才能夠執行與該鎖衝突的操作。悲觀並行控制主要用於資料爭用激烈的環境，以及發生並發衝突時使用鎖保護資料的成本要低於回復交易的成本的環境中。

3. 什麼是正規化？
	正規化是在資料庫中組織資料一系列原理和技術。其中包括建立資料表，以及在這些資料表之間根據規則建立關聯性，這些規則的設計目的是：透過刪除重複性和不一致的相依性，保護資料並讓資料庫更有彈性。 
	
	- 第一正規化
	第一正規化是資料庫正規化中所使用的一種正規形式。第一正規化是為了要排除重複群 的出現，所採用的方法是要求資料庫的每個列的值域都是由原子值組成；每個欄位的值都只能是單一值。

	- 第二正規化
	符合第一正規化。所有非鍵的欄位都一定是候選鍵全體欄位的函式。

	- 第三正規化
	是資料庫正規化中所使用的一種正規形式，要求所有非鍵屬性都只和候選鍵有相關性，也就是說非鍵屬性之間應該是獨立無關的。

	現在資料庫設計最多滿足 3NF，普遍認為正規化過高，雖然具有對資料關係更好的約束性，但也導致資料關係表增加而令資料庫 IO 更易繁忙，原來交由資料庫處理的關係約束現更多在資料庫使用程式中完成。

# RDB vs. NoSQL

![後端軟體工程工具箱： 資料庫/SQL/ORM 篇 ](/img/kdchang/nosql-vs-sql-overview.png) 

一般而言，資料庫主要有分為關聯式資料庫（RDB）和 NoSQL（Not Only SQL / nonrelational）資料庫，關聯式資料庫是由一堆資料表組成，每個資料表會對照一個固定的資料表結構（schema）。一般會使用 SQL（Structured Query Language）這種宣告式語言來操作數據資料，例如：MySQL、PostgreSQL、Oracle、MSSQL、SQLite。相對於關聯式資料庫，NoSQL 並不是使用資料表來保存資料，例如：MongoDB 就是一種很流行的文件型態 NoSQL 資料庫，其元素並非一列列數據，而是使用類 JSON 文件（document）格式呈現，更容易延展擴充和更有效率。此外還有針對不同需求設計的資料庫，例如：欄資料庫、圖形（Graph）資料庫等。本篇文章會主要討論關聯式資料庫並補充一些 NoSQL 知識點。

# SQL

首先，我們先來介紹如何使用 SQL 來操作關聯式資料庫！（注意 SQL 語法可以大寫或小寫，但實務上通常寫大寫。）

RDB 資料庫相關命名方式（主要依團隊共識）：
1. 資料庫：底線連接 
2. 資料表：複數、底線連接 
3. 資料欄位：單數、底線連接

NoSQL 資料庫相關命名方式（主要依團隊共識）：
1. 資料庫：Pascal casing (a.k.a. upper camel case)
2. 資料表：單數，Schema name for tables prefix (E.g.: SchemeName.TableName) 
3. 資料欄位：單數，Schema name for tables prefix (E.g.: SchemeName.TableName)

按：Microsoft 的命名方式喜歡單字第一個字母大寫，如 OrderDetail。而 MySQL 比較常見全部小寫，單字中間加底線的命名方式，如 birth_date。這和資料庫的字元大小寫敏感度預設值有關，MS SQL Server 預設是大小寫不分，MySQL 則是大小寫視為不同欄位，所以統一小寫比較不容易出錯。

![後端軟體工程工具箱： 資料庫/SQL/ORM 篇 ](/img/kdchang/rdb.jpg) 

1. 資料庫/資料表創建（CREATE）
	表格被分為欄位 (column) 及列位 (row)。每一列代表一筆資料，而每一欄代表一筆資料的一部份。當我們對表格下定義時，我們需要註明欄位的標題，以及那個欄位的資料種類。

	使用 `CREATE DATABASE` 指令可以建立資料庫，命名使用小寫英文以及下底線組：

	```sql
	CREATE DATABASE database_name;
	```

	實際使用，建立一個名為 `dr_course` 的資料庫：

	```sql
	CREATE DATABASE dr_course;
	```

	使用 `CREATE TABLE` 指令可以建立資料表，命名使用大寫英文以及：

	```sql
	CREATE TABLE "表格名"
	("欄位 1" "欄位 1 資料種類",
	"欄位 2" "欄位 2 資料種類",
	... );
	```

	實際使用，建立一個名為 `users` 的資料表：

	```sql
	CREATE TABLE users
	(id BIGINT(7) NOT NULL AUTO_INCREMENT,
	name CHAR(50),
	email CHAR(50),
	age INT(50),
	course_id INT(50),
	salary INT(50));
	```

	實際使用，建立一個名為 `courses` 資料表：

	```sql
	CREATE TABLE courses
	(id BIGINT(7) NOT NULL AUTO_INCREMENT,
	name CHAR(50),
	point INT(50));
	```

2. 新增（INSERT）

	使用 `INSERT INTO` 指令可以新增資料：

	```sql
	INSERT INTO "表格名" ("欄位1", "欄位2", ...) VALUES ("值1", "值2", ...);
	```

	實際使用：

	```
	INSERT INTO users ("name", "email", "age", "salary") VALUES ("Mark", "mark@gmail.com", 20, 70000);
	```

3. 選擇（SELECT）

	使用 `SELECT` 指令可以選取資料：

	```sql
	SELECT "欄位" FROM "表格名";
	```

	實際使用：

	```sql
	SELECT name, email FROM users;
	```

4. 刪除（DELETE）

	使用 `DELETE` 指令可以新增資料：

	```sql
	DELETE FROM "表格名" WHERE "條件";
	```

	實際使用：

	```sql
	DELETE FROM users WHERE id=1;
	```
5. 修改（UPDATE）
	
	使用 `UPDATE SET` 指令可以新增資料：
	
	```sql
	UPDATE "表格名" SET "欄位1"=[新值] WHERE "條件";
	```

	實際使用：
	
	```sql
	UPDATE users SET name=Mark WHERE id=1;
	```

6. 分組根據（GROUP BY）

	使用 `INSERT INTO` 指令可以新增資料：

	```sql
	SELECT "欄位1", SUM("欄位2") 
	FROM "表格名" 
	GROUP BY "欄位1";
	```

	實際使用：

	```sql
	SELECT name, SUM(salary) FROM users GROUP BY users;
	```

7. 排序根據（ORDER BY）

	通常我們會需要將資料做排序，此時可以使用 `ORDER BY` 指令可以進行資料排序（[] 代表可選，ASC 和 DESC 是 ascending 和 descending 的縮寫，default 使用 ASC）：

	```sql
	SELECT "欄位名" 
	FROM "表格名" 
	[WHERE "條件"]
	ORDER BY "欄位名" [ASC, DESC];
	```

	實際使用：

	```sql
	SELECT * FROM users ORDER BY ASC;
	```

8. 聯結（JOIN）

	我們現在有兩個資料表單一個是 users 一個是 courses ，一個存放使用者的資料表，一個存放課程資料表。我們希望計算使用者修了多少學分的課，所以透過 course_id 來連結兩個表單：

	```sql
	SELECT A1.name, SUM(A2.point)  
	FROM users A1, courses A2 
	WHERE A1.course_id = A2.id 
	GROUP BY A1.course_id;
	```

	之前我們看到的左連接 (left join)，又稱內部連接 (inner join)。在這個情況下，要兩個表格內都有同樣的值，那一筆資料才會被選出。那如果我們想要列出一個表格中每一筆的資料，無論它的值在另一個表格中有沒有出現，我們就需要用到 SQL OUTER JOIN (外部連接) 的指令：

	```sql
	SELECT A1.name, SUM(A2.point) SALES 
	FROM users A1, courses A2 
	WHERE A1.course_id = A2.id (+) // 在 Oracle 資料庫要使用 + 
	GROUP BY A1.course_id;
	```

9. 不重複資料（DISTINCT）

	使用 `DISTINCT` 指令可以選擇不重複資料：

	```sql
	SELECT DISTINCT "欄位" 
	FROM "表格名";
	```

	實際使用：

	```sql
	SELECT DISTINCT name FROM users;
	```

# 子查詢
在 SQL 中可以將查詢結果當做一個資料表，再次進行 SELECT 查詢：

```
SELECT MIN(id) AS min_user_id FROM (SELECT id FROM users WHERE age = 20); 
```

# 索引
建議資料表中常用來查詢的欄位使用 index 索引可以在使用 JOIN 時提高檢索效率，資料表中可以有一個或多個索引，也可以把某個索引欄位設為 unique。

# 查詢最佳化

一般而言，先篩選再 JOIN 效能會比較好。但一般資料庫引擎會幫你最佳化，所以只要下 SQL 就好：

```sql
SELECT name FROM users JOIN courses ON users.course_id = courses.id WHERE courses.point > 2;
```

# ORM
物件關聯對映（英語：Object Relational Mapping，簡稱ORM，或O/RM，或O/R mapping），是一種程式設計技術，用於實現物件導向編程語言裡不同類型系統的資料之間的轉換。

# MongoDB
MongoDB 是一個開源且跨平台的 NoSQL 資料庫，主要使用類 JSON 格式的文件進行資料儲存和 schema 定義。

![後端軟體工程工具箱： 資料庫/SQL/ORM 篇 ](/img/kdchang/nosql-databases.gif) 

# Redis
Redis 是一個非常流行的開源、支援網路、基於記憶體、鍵值對儲存資料庫，使用 ANSI C 編寫而成。

```python
#coding:utf-8
import redis
# lredis-server 保持開啟，若是有使用密碼，則要在 ConnectionPool 使用 password=密碼
pool = redis.ConnectionPool(host='127.0.0.1', port=6379, db=0)
r=redis.StrictRedis(connection_pool=pool)
# 字符串 string
r.set('test','aaa')
print r.get('test');
# 列表 list
# 注意 python、lrange range 範圍
x=0
for x in range(0,11):
	r.lpush('list',x)
	x=x+1
print r.lrange('list','0','10')
# hash/dict 
dict_hash={'name':'tang','password':'tang_passwd'}
r.hmset ('hash_test',dict_hash)
print r.hgetall('hash_test')
# 集合 set
r.sadd('set_test','aaa','bbb')
r.sadd('set_test','ccc')
r.sadd('set_test','ddd')
print r.smembers('set_test')
# 有序集
r.zadd('zset_test','aaa',1,'bbb',1)
r.zadd('zset_test','ccc',1)
r.zadd('zset_test','ddd',1)
print r.zrange('zset_test',0,10)
```

# Memcache

Memcache 是一套開源的分散式的快取系統，是由 LiveJournal 的 Brad Fitzpatrick 所開發。由於一般認為 Memcache 缺乏認證以及安全管制，因此應該將 Memcache 伺服器放置在防火牆後。

此外 memcached 的 API 使用三十二位元的循環冗餘校驗（CRC-32）計算鍵值後，將資料分散在不同的機器上。當表格滿了以後，接下來新增的資料會以 LRU 機制替換掉。由於 memcached 通常只是當作快取系統使用，所以使用 memcached 的應用程式在寫回較慢的系統時（像是後端的資料庫）通常需要額外的程式碼更新 memcached 內的資料。

```
function get_foo (int userid) {
   result = db_select("SELECT * FROM users WHERE userid = ?", userid);
   return result;
}
```
 
下面的程式會先到 Memcache 檢查是否有 `userrow:userid` 的資料，如果有則直接傳回結果，如果不存在時再去資料庫查詢，並將結果放到 Memcache 內，記得要同步資料庫和 Memcache，避免 Cache coherency 問題：

```
function get_foo (int userid) {
    result = memcached_fetch("userrow:" + userid);
    if (!result) {
        result = db_select("SELECT * FROM users WHERE userid = ?", userid);
        memcached_add("userrow:" + userid,  result);
    }
    return result;
}
```

# AWS DynamoDB 
Amazon DynamoDB 是一種快速靈活的 NoSQL 雲端資料庫服務，適合所有需要一致性且延遲低於 10 毫秒規模應用程式。它是全受管的雲端資料庫，支援文件和鍵值存放模型。

# 總結
以上介紹了後端軟體工程工具箱：資料庫/SQL/ORM 相關議題，在接下來的章節中我們將為大家打開後端工程的工具箱，介紹那些必須掌握的後端軟體工程知識。

# 延伸閱讀
1. [並發控制（英語：Concurrency control）](https://zh.wikipedia.org/wiki/%E5%B9%B6%E5%8F%91%E6%8E%A7%E5%88%B6)
2. [SQL 語法教學](http://www.1keydata.com/tw/sql/sql.html)
3. [資料庫概念](http://www.wun-ching.com.tw/img/Books_files/D049e4-9789862368602-trial.pdf)
4. [MySQL与PostgreSQL：该选择哪个开源数据库？哪一个更好？](http://www.infoq.com/cn/news/2013/12/mysql-vs-postgresql)
5. [說明資料庫正規化基本概念](https://support.microsoft.com/zh-tw/kb/283878)
6. [SQL Tutorial](http://www.w3schools.com/SQl/default.asp)
7. [MySQL 超新手入門（8）儲存引擎與資料型態](http://www.codedata.com.tw/database/mysql-tutorial-8-storage-engine-datatype/)
8. [標題[SQL ] 欄位命名規則](https://www.ptt.cc/bbs/Database/M.1332051208.A.F4E.html)
9. [資料庫物件命名原則](http://renjin.blogspot.tw/2008/02/database-naming-conventions.html)
10. [資料庫表單及欄位命名規則實例](http://www.neo.com.tw/archives/275)

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 