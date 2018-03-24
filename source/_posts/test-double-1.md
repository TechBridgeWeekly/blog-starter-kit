---
title: 測試替身(上篇)
date: 2018-03-02 07:12:25
tags: unitTest testDouble
author: jyt0532
---

如果你在工作時寫過一些測試程式，也許你聽過什麼是 Mock，但其實 Mock 只是測試替身的其中一種，測試替身包含了 Dummy、Mock、Fake、Stub、Spy。而不同語言或不同 framework 有時候會把類似的概念合在一起。

本系列的目的是讓你寫單元測試的時候，對應不同情況，知道應該用哪一種替身。像筆者常用的 Mockito 基本上把大部份的測試替身都用 Mock 取代，雖然用起來容易，但我以前從來不知道自己用的是哪一種測試替身 。

希望看完本系列的文章後，讀者能夠在寫完一段程式碼之後，就立即能分析出自己這段程式碼需要用的是什麼替身來進行測試。

![Alt text](/img/jyt0532/the_prestige.jpg)
## 名詞解釋

SUT: System Under Test。就是需要被測試的東西。

DOC: Depended On Component。就是SUT需要依賴的東西。

DOC 非常常見，幾乎無可避免。

比如說 SUT 是 web server。那 DOC 就是 database。

比如說 SUT 是 web fronted。那 DOC 就是 web server。

你不太可能每個函數都自己玩自己的，你通常都會需要呼叫別人的函式，但這其實對測試帶來了負擔。比如說你每次想測試你的 webserver 可不可以新增使用者的時候，你都需要真的去 database 叫他加一個給你。

這實在開銷太大，也非常不實際。

## 測試替身的目的

1.第一個也是最重要的一個，是隔離你的 SUT，不被任何 DOC 干擾。

我不想要我測試新增使用者的時候，還要保證 database 是正常的。我任何時候都想跑測試，不依賴任何人。

2.加速執行時間，避免不必要開銷。

不依靠他人之後，所有你需要的 DOC 的回傳值都先定義好，當然加快了執行速度。

3.讓你的測試 Deterministic。

我不想要在不同時間或不同空間裡，會得到不一樣的測試結果。
比如說，尖峰時刻 database load 太大，回傳了不預期的 Http status 429。
這是我不想在我預期要是 happy case 的情況時看到的結果。

4.模擬特殊狀況(special case)

比無法測試 happy case 的情況還要慘的事情，是無法測試 bad case。

這點也很重要，如果真的遇到了不預期的狀況(比如剛剛說的 429)，最慘就是等一陣子，database 正常後，就可以過了。

但有時候我們就是想要知道，當真的回傳 429 的時候，我們處理的方法是不是正確。如果沒有 test doubles，根本無法保證這種狀況一定發生。也不可能去 DDOS 自己的 database 製造這種情況。

5.可以讓你測試到你不想公開的資訊

這點就厲害了。來個例子:

SUT 是 WebServer。 DOC 是 database

```java
public class WebServer {
  private Database database;
  public void create(){
    database.insert()
  }
}
```

比如說這樣，我想知道我 `webser.create` 的時候，`database.insert`的確被呼叫，要怎麼測試？
我不想要開放一個 `public function getDatabase` 供大家存取 database 的**狀態**，僅僅只是為了測試用途。

該怎麼辦呢？這時候來一個漂亮的替身。

```java
public class WebServerTest{
  @Test
  public void databaseInsertedWhenServerCreate(){
    TestDatabase testDatabase = new TestDatabase();
    WebServer webserver = WebServer(testDatabase);
    webserver.create();
    assertTrue(testDatabase.isInserted)
  }
}
public class TestDatabase extends Database{
  private boolean isInserted
  public void insert(){
    isInserted = true;
  }
  public void isInsert(){
    return isInserted;
  }
}
```
注意:

1. TestDatabase 需要 extends Database，不然丟不進 Webserver 的 constructor。

2. 原本的 Database 這個 Class 沒有 `isInsert` 這個函式，是我自己加的。

搞定。prod 上的 Database 完全不用動，我就可以知道當 `webserver.create` 被呼叫的時候，我的的確確呼叫了 `database.insert`。


看完了為什麼需要測試替身之後，之後會一一介紹每個測試替身的使用時機跟用法。

## Dummy

首先登場的，就是最簡單的替身，也就是 Dummy。

當我們需要傳一個變數給某個 method 的時候，需要一個跟 signature 一樣型態的變數，**可是這個變數以後又不會用到**。為了加速跟省記憶體空間，我們可以丟一個 Dummy 替身給這個 method。

比如說，我們想要測試 People 的 `getNumberOfPerson` 這個函式。

```java
public class People {
  private List<Person> persons;
  public void addPerson(Person p){
    persons.add(p);
  }
  public int getNumberOfPerson(){
    return persons.size();
  }
}
```
那你的測試可能原本長成這樣:
```java
public class PeopleTest{
  @Test
  public void testGetNumberOfPerson(){
    People people = new People();
    Person person1 = new Person("John Doe", 25, address1, phoneNumber1);
    Person person2 = new Person("Jane Doe", 23, address2, phoneNumber2);
    people.addPerson(person1);
    people.addPerson(person2);
    assertEquals(2, people.getNumberOfPerson())
  }
}
```
建一個 Person 可能開銷過大，而且很麻煩。況且 Person 並不是這個 test 的重點。

這時候就要來個簡單的 Dummy。
```java
public class DummyPerson extends Person {
   public DummyPerson() {}
}

```
這個 class 的目的就是要通過 `addPerson` 的型別限制，所以只要 extends Person 就可以。
```java
public class PeopleTestWithDummy{
  @Test
  public void testGetNumberOfPerson(){
    People people = new People();
    people.addPerson(new DummyPerson());
    people.addPerson(new DummyPerson());
    assertEquals(2, people.getNumberOfPerson())
  }
}
```

就是這麼簡單，Person 的其他 method 我們都全部不管，那如果 getNumberOfPerson 呼叫了 Person 的其他 method 我們無法知道，但這也不是這個 unit test 在乎的重點。

但如果你真的想確保其他 method 不會被 call，那就在 DummyPerson 裡面覆寫 Person 的其他 method。然後都 throw Exception 就可以。

其實很多人在測試的時候，直接傳 null 進去，如果你要傳進去的 function 沒有 nullCheck，這也是個可行的方式。但如果有 nullCheck 那還是只能用 Dummy。

### Mockito - Dummy

如果你寫的是 java，你會很常看到 Mickito。
在 Mockito 裡面，如何生一個 Dummy object 呢?
```java
public class PeopleTestWithDummy{
  @Test
  public void testGetNumberOfPerson(){
    People people = new People();
    people.addPerson(mock(Person.class));
    people.addPerson(mock(Person.class));
    assertEquals(2, people.getNumberOfPerson())
  }
}
```

小心不要被這裡的 mock 搞混，他只是 syntax 是 mock，但在 mockito 裡面，如果你只 mock 一個 class 但沒有給他任何預期的行為，他就是個dummy。

**我們是以用法來區分 TestDoubles，不是 syntax**，因為很多 framework 不會為每一個 TestDoubles 都給一個專屬 syntax。


## Stub

Stub 是我們介紹的替身中第一個可以讓我們獨立測試 SUT 的測試替身。

### Stub 使用時機

當我們需要測試一個 SUT，但我們卻不想要依賴真實的 DOC，我們可以用 Stub 去取代我們的 DOC。

Stub 並不需要真的表現的跟 DOC 一樣。他只要 api 長得一樣(也就是輸入輸出長得一樣)，讓 SUT 以為是真正的 DOC 就可以。

來個例子，今天我們想測試 `getSecretNumber` 是不是能正確回傳。

```java
public class WebServer {
  private Database database;
  int secretNumber;
  public WebServer(Database database){
    this.database = database;
    secretNumber = 42;
  }
  public int getSecretNumber(String username,String password){
    return database.authorize(username, password) ?
        secretNumber : -1;
  }
}
public class Database {
  public Database(){
  }
  public boolean authorize(String username, String password){
    //Connect to database
    //Query database
    //etc
  }
}
```
程式碼裡面，我們需要去 authorize，這一步會花費很多時間，這也不是現在這個測試的重點。這時候就來個 Stub。


```java
public class DatabaseStub extends Database {
  public boolean authorize(String username, String password) {
    return true;
  }
}
public class TestWithStub{
  @Test
  public void testGetSecretNumber(){
    WebServer webserver = new WebServer(new DatabaseStub());
    assertEquals(42, webserver.getSecretNumber("BoYu", "jyt"));
  }
}
```

搞定，這樣就不用真的去 query database。

### Mockito - Stub

有 Mockito 的話，並不需要真的寫一個新的 DatabaseStub。
```java
Database databaseStub = mock(Database.class);
when(databaseStub.authorize(anyString(), anyString()))
    .thenReturn(true);
WebServer webserver = new WebServer(databaseStub);
assertEquals(42, webserver.getSecretNumber("BoYu", "jyt"));
```
你把一個 class Mock 了之後，他的每一個 function 都只會回傳 null。
**你需要去指定你會用到的 method 的行為** 輸入值是什麼回傳什麼。

### 測試特殊情況

如介紹時所說，stub 還能模擬測試的特殊狀況。

在這個例子裡，你想知道如果 authorize 不過，是不是回傳 -1。就把剛剛 stub 的回傳值改成 false 就可以。


```java
Database databaseStub = mock(Database.class);
when(databaseStub.authorize(anyString(), anyString()))
    .thenReturn(false);
WebServer webserver = new WebServer(databaseStub);
assertEquals(-1, webserver.getSecretNumber("BoYu", "jyt"));
```
還可以讓你的 Stub throw exception。

```java
when(databaseStub.authorize(anyString(), anyString()))
  .thenThrow(new NullPointerException());
```

完全不用依賴真正的 DOC，隨便你愛怎麼玩就怎麼玩。

當你的 SUT 有一些 indirect input(並不是在你測試的程式提供的 input，而是 DOC 提供的 input)，需要事先定義好 DOC 的回傳值，就是用 Stub。

## 總結

本文介紹了何謂測試替身以及測試替身的目的，還介紹了 Dummy 和 Stub 兩種替身，下一篇會介紹另外三種測試替身，儘請期待。

## 延伸閱讀

1. [我聊的不是金庸 是測試替身](https://www.jyt0532.com/2018/01/24/jinyong-test-double/)

關於作者：
[@jyt0532](https://www.jyt0532.com/) 後端工程師，喜歡學習新知挑戰新事物，最近在寫一本關於[JVM的教學書](https://www.jyt0532.com/toc/jvm/)。
