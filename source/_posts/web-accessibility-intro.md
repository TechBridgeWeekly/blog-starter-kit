---
title: Web Accessibility 的重要性
date: 2019-10-13 17:07:29
tags:
  - web
  - web accessibility
  - a11y
  - javascript
---

# 前言

你是否曾因為網路上被歧視、被霸凌的新聞或影片而憤憤不平？你是否曾因為各種身殘心不殘的勵志故事而感到激勵？
如果你有過上述經驗，那當你在製作網站時，是否曾經考量過 Web Accessibility（a11y） 呢？

老實說，我真的很少在實作時認真的驗證自己的網站的可訪性，花在考量是否能支援 IE 9 的時間還比較多一些。（想當年在政府機關服務時，看到局處長很自豪官網拿到無障礙網站評等第一時，還覺得莫名其妙，想說網站這麼醜，到底哪裡無障礙 XD）

在實作的時候，PM 會拿數據告訴你，有多少用戶是使用舊版的瀏覽器，所以我們需要支援到何種程度，但應該很少 PM 會拿出數據告訴你，有多少 screen reader 來存取我們的網站·。

然而，光台灣，2018 年的身心障礙人口有一百多萬人，其中視覺障礙的有將近六萬（[衛生福利處的資料](https://dep.mohw.gov.tw/DOS/lp-2976-113.html)），美國比例更高，有將近 25% 的身心障礙人口（[資料來源](https://youtu.be/dvtfNpt75aA?t=26101)）。

依照你產品的用戶比例，這些數據大概無法說服你的 PM，讓他們給你更多時間思考如何增強網站的可訪性，但仔細想想，如果你在乎那些身心健全卻不願意升級自己瀏覽器的人，而不在乎這些走出障礙，連接到網路吸取知識的殘障人士，你是不是無意中在他們的人生道路中增加了更多阻礙，不自覺地成為歧視他人的一方呢。

今天想透過這篇文章，整理一些資源與簡短的實作要點，喚醒大家對 Web Accessibility 重要性的認知，也是提醒自己在未來實作上必定要多加注意。

# 使用 screen reader 的感覺是什麼

相信會看到這篇文章的人，多少對於 Web Accessiblity 都有了解，知道是為了輔助障礙人士閱讀網頁內容，平常實作時至少在 `<img>` 元素上會加上 `alt` 等文字來描述圖片。但你有真的使用過 screen reader 嗎？你知道使用這些輔助工具上網的人，看到的世界跟你有多大的不同嗎？

我原本也不知道，直到我試用了 Mac 內建的 VoiceOver，用它來閱讀平常看的網站，像是 wikipedia：
<iframe width="560" height="315" src="https://www.youtube.com/embed/IHm6me_VfyM" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

體驗實在很糟，除了機器人的聲調外，外加極快的語速，在你從瀏覽器的分頁標籤移動到真正的網頁內容前，還需經過許多瀏覽器本身按鈕選項的介紹，接著到了網頁內容，文章因為連結的關係，變得破碎（為了讓你知道是連結或是按鈕，在文字內容前都會先朗讀出該段內容的功能性，像是：*連結*、*按鈕*），閱讀順序也與我們憑眼睛觀看時的不同。

然而 Wikipedia 已經算是 a11y 處理得不錯的了，畢竟內容大多也只有文字與圖片。想想看現在這麼多炫麗介面的網站，各種需要使用者與之互動的功能，光用想像的就是悲劇，看一下 Google 大肆宣傳的 AMP Story：

<iframe width="560" height="315" src="https://www.youtube.com/embed/cdPVMDJB37k" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

在強調 Web performance 的同時，web accessibility 也該考量進去。這邊就有[文章](https://ethanmarcotte.com/wrote/amphora/)在討論關於 amp-story 在 a11y 的 [issue](https://github.com/ampproject/meta-ac/blob/master/meetings/2019-03-f2f.md#accessibility)，不過 ampproject 內是有 accessibility working group 在想辦法解局的。

上面的 Screen reader 範例比較是針對視障朋友的使用為主，現實中能造成生活不便的可不只有視覺上的困難，還有聽覺、肢體等等，像是滑鼠這類需要高度手眼協調能力的操作，就不是那麼有親和力，因此光是透過鍵盤操作網站的順暢度，就必須好好思考，也是實作 Web Accessibility 時的一大重點。

如果想快速體驗一下使用 screen reader 的效果，又不想學習 Voiceover 等系統內建 reader 的，可以到 udacity 提供的[網站](http://udacity.github.io/ud891/lesson3-semantics-built-in/02-chromevox-lite/) 試試，使用的是 Chrome 的 chromevox-lite 閱讀器，且有刻意將內容模糊化，感受會更真實點。

# Web Accessibility 的資源與實作要點

說了許多緣由以及範例，現在來談談該如何學習 Web Accessibility，並實作在我們的網站中。

## 網絡內容無障礙功能指南 - Web Content Accessibility Guidelines

先從 W3C 指南了解 "無障礙" 的定義。[網絡內容無障礙功能指南 (WCAG) 2.1](https://www.w3.org/TR/WCAG21/)，是 W3C 擬定的無障礙功能指南和最佳做法，旨在有系統地闡述“無障礙功能”的含義。很多國家政府機關都會要求根據此指南來實作網站。

WCAG 有四大原則：

* 可感知（Perceivable）：所有資訊與使用者介面元件都必須要能以各種使用者能感知的方式，呈現給使用者。像是能讓 Screen Reader 閱讀出內容。不能讓使用者透過各種感官都無法感知到內容。

* 可操作（Operable）：使用者元件與網頁內容間的導覽都要可操作，不能出現使用者無法操作的介面。

* 可理解（Understandable）：呈現給使用者的資訊與操作方式都要能被使用者理解。

* 強健（Robust）：不論是被哪種輔助科技工具或是各種 user agents 所存取，網站內容都要能不受影響，呈現給使用者完整的內容，若使用者使用的工具變得更加進步，網站也得跟上腳步，維持內容的存取、閱讀性。

這是無障礙內容的一個概覽，還可以透過 [WebAIM](https://webaim.org/) 檢查清單，來檢視我們該如何依照 WCAG 的指南實作。

## 其他資源

除了 W3C 的定義指南外，還有許多更親和的資源可以參考，畢竟 Web accessibility 不是很新的議題了，像是 [MDN](https://developer.mozilla.org/zh-CN/docs/Learn/Accessibility) 或是 [Google](https://developers.google.com/web/fundamentals/accessibility) 都有非常詳細的資料與教學，Udacity 上也有相關[課程](https://www.udacity.com/course/web-accessibility--ud891)。

看完大概都需要花上幾個小時的時間，如果想快速了解的話，其實也已經有不少關於 a11y 的繁中文章，像這篇 - [回歸初心，一探 Web Accessibility](https://medium.com/frochu/%E5%9B%9E%E6%AD%B8%E5%88%9D%E5%BF%83-%E4%B8%80%E6%8E%A2web-accessibility-baaa4d22f4a7)，簡短精要，整理了上面 Google 與 Udacity 課程的內容，適合快速了解何謂 a11y，而最近[鐵人賽的這系列文](https://ithelp.ithome.com.tw/users/20108045/ironman/2454)也很不錯，蒐集整理非常多資訊，細節很多，可以當作參考工具，在實作時隨時翻閱，推推。

此外，上面的資源著重在一般的網站上，而身為資料視覺化的愛好者，我很好奇該如何處理 Data visualization 上的 Web Accessibility 問題。

從 [Lindsey](https://www.a11ywithlindsey.com/about) 的這篇文章 - [Accessibility in d3 Bar Charts](https://www.a11ywithlindsey.com/blog/accessibility-d3-bar-charts) 中可以窺知一二。

主要是要在圖表中加入足夠的描述文字，然後在設計顏色時，使用一些輔助工具如 [coblis](http://www.color-blindness.com/coblis-color-blindness-simulator/) 來建立 color blind friendly 的圖表。

若是較常使用 SVG 來製作動態圖表的人，推薦研讀一下這篇專門介紹如何為你的 SVG 增強 accessibility 的文章 -  [accessible-svgs](https://css-tricks.com/accessible-svgs/)，可以為你的資訊圖表在 Web Accessibility 上帶來非常大的改善。

## 實作要點

知道了為何需要了解 Web Accessibility，也知道了有哪些資源可以閱讀，接下來根據上面閱讀的資料，簡單總結幾個實作上要關注的重點：

### 思考網站能以什麼方式瀏覽

這其實是 [回歸初心，一探 Web Accessibility](https://medium.com/frochu/%E5%9B%9E%E6%AD%B8%E5%88%9D%E5%BF%83-%E4%B8%80%E6%8E%A2web-accessibility-baaa4d22f4a7) 這篇文章中所提到的結論，我覺得蠻有道理的。

與其思考使用者失去什麼感官能力，不如專注在你的網站能提供哪些方式瀏覽。可能是 screen reader、鍵盤（keyboard）、聲控或眼動儀等其他人機介面。從這些操作介面去思考該怎麼加強自己網站的可訪性。

### 網頁元件的可聚焦性

所謂的可聚焦，指的就是元件能夠成為焦點，而所謂焦點，是指當前螢幕上瀏覽器當下能接收來自鍵盤輸入、剪貼板輸出的元件（字段、複選框、按鈕或連結等輸入項目），更簡單來說，就是使用者此刻正在操作的元件。

為什麼可聚焦性很重要呢？除了讓使用者知道自己目前所使用的元件外，就是讓 screen reader 知道目前該 read 哪個元件。而瀏覽器一次只能聚焦在一個元件上，這個特性讓 screen reader 不會同時讀到兩個不同元件的描述。

在一般瀏覽器上頭，被聚焦的元件通常預設會以一個藍色外框包裹住：

![defaul outline](/img/arvinh/outline-blue.png)

實務上很多人都會用 `outline: none` 把這個外框拿掉，但實際上是非常不友善的行為，若是不喜歡瀏覽器預設的樣式，可以跟設計師溝通，看要如何修改被聚焦的元件樣式，但記得要能讓使用者注意到樣式的變化，才能幫助他們判斷元件的聚焦與否。如 Youtube 上的回復查看按鈕：

![custom outline](/img/arvinh/outline-custom.png)

另外要注意的是，並非所有元件都要設成可聚焦，像是文章內文本身，或是其他即便讓 screen reader 朗讀出來也無法很好傳達意思的元件，其實跳過反而比較不會造成混淆與困擾。

### DOM 的順序

Screen reader 在移動時，會依照 DOM 的順序進行，所以應當盡量將你想要呈現給使用者的內容順序，完整的對應到 DOM 的順序上頭。除此之外，還得注意 CSS 的影響，例如 `float`，就可能會造成視覺上的順序與 DOM 順序有所差異，導致 screen reader 朗讀的次序受到影響與畫面不符：

```html
<button style="float: right">1</button>
<button>2</button>
<button>2</button>
```

<button style="width:100px;background:transparent;color:#5050c5;font-size: 1.5rem;float: right">1</button><button style="width:100px;background:transparent;color:#5050c5;font-size: 1.5rem;">2</button><button style="width:100px;background:transparent;color:#5050c5;font-size: 1.5rem;">2</button>
<br/>

另外，善用 `tabindex`，能夠更好的幫助使用者操作鍵盤（tab）瀏覽網站時的移動順序，無論是跳過隱藏在 Responsive 表單選項中的元件，或是在開啟的 Modal 中製造出 Key trap，讓使用者不會移動到 Modal 覆蓋下的元件上。

### 語義化標籤

眾所皆知，Semantic HTML 對於提高 SEO 很有幫助，然而，它也能大大增加網站的可訪性（大概也是因此才會讓 SEO 效果提升），一個好的 Semantic HTML Element 應該包含：

1. Role： 元件的類型（按鈕、input 元件、超連結等等）
2. Name(Label)：通常與 input 元件並用，像是 radio input、dropdown list 等等
3. State：元件當下的狀態（點擊、展開、收合）
4. Value：元件內的值（Input 元件內的值、Button 上的文字等等）

瀏覽器會根據你的 HTML 建造出 DOM Tree，同時也會依照你在 DOM 元件上的資訊建造出 Accessibility Tree（[source](https://developers.google.com/web/fundamentals/accessibility/semantics-builtin/the-accessibility-tree)）：

![accessibility-tree](/img/arvinh/treestructure.jpg)

而 Screen reader 就會根據這棵 Accessibility Tree 進行朗讀，因此你提供的 Semantic HTML 越清楚，資訊越詳細，就能夠建構出越強健的 Accessibility Tree 供輔助工具參考。

再者，如同文章前頭提到的，網頁上的媒體元件，像是 `img`、`video` 或是視覺圖表，都應該提供對應的 `alt` 描述，讓 Screen reader 至少能根據 `alt` 來說明該媒體元件的內容。不過，若是單純拿來裝飾用的 `img`，可以給予 `alt` 空值，代表其裝飾性。

想檢測網站是否有足夠完整的 Accessibility Tree 的話，可以用 Chrome 打開你想看的網站，並且開啟開發者工具，在 Element 標籤內，右側可以找到 Accessibility 的 tab，在那邊就能看到當前頁面元件的 Accessibility Tree：

![Chrome DevTool Accessibility Tree](/img/arvinh/chrome-dev-AT.png)

### WAI-ARIA

WAI-ARIA，代表 Web Accessibility Initiative — Accessibility Rich Internet Applications （網頁可訪性倡議 — 無障礙網頁應用）。

上面章節我們提到瀏覽器會依照 Semantic HTML Element 來建構 Accessibility Tree，然而有時你需要使用像是 `div`、`span` 等元素來做一些客製化的元件時，該怎麼辦呢？不就無法保持語義了嗎？

這時 WAI-ARIA 就派得上用場了。使用 ARIA 屬性，我們就可以補足元件缺少的訊息，讓它進入 Accessibility Tree 當中。（[source](https://developers.google.com/web/fundamentals/accessibility/semantics-builtin/the-accessibility-tree)）

![DOM+ARIA](/img/arvinh/dom-aria-tree.png)

舉個例子：

一個用 `<li>` 元件所客製化的 checkbox，我們可以依靠 CSS 讓他 "表現" 得像 checkbox，對於視力正常的人來說是沒問題的，但 screen reader 可不認得他，因此我們至少必須加上 ARIA 屬性中的：`role` 與 `aria-*`，讓其加入到 Accessibility Tree 中：

```diff
- <li tabindex="0" class="checkbox" checked>
+ <li tabindex="0" class="checkbox" role="checkbox" checked aria-checked="true">
  Receive promotional offers
</li>
```

`role` ，等於宣告該 DOM 元件該扮演什麼角色，像是 `checkbox`、`button` 或 `dialog` 等，還可再分類出 `Widget roles`、`Composite roles` 和 `Landmark roles` 等等。詳細 spec 可以看 [W3C 的定義](https://www.w3.org/TR/wai-aria-1.1/#role_definitions)

而搭配 `role` 使用的 `aria-*` 則為元件定義了*屬性*與*狀態*，像是上面例子中的 `aria-checked="true"` 即為元素狀態的一種，告知 Accessibility Tree 這元件目前屬於 `checked` 狀態。

也能透過 `aria-lable` 設定專門給輔助工具使用的 API：

```js
<button aria-label="screen reader only label"></button>
```

或是設定父項/子項聯繫，例如控制特定區域的客製化捲軸：

```js
<div role="scrollbar" aria-controls="main"></div>
<div id="main">
// . . .
</div>
```

關於 `role` 與 `aria-*` 狀態和屬性，可以先從 MDN 的[這份文件](https://developer.mozilla.org/en-US/docs/Web/Accessibility/ARIA/ARIA_Techniques)概觀所有列表，然後從 [W3C 的定義](https://www.w3.org/WAI/PF/aria-1.1/states_and_properties)中去翻找詳細資訊。

### 色彩、樣式

文章前面有提到，設計顏色時，應當使用一些輔助工具如 [coblis](http://www.color-blindness.com/coblis-color-blindness-simulator/) 來模擬色盲的使用者是如何 "看" 你的網站，可以根據這些測試與資訊，與設計師討論，製作出 color blind friendly 的介面。

除了顏色之外，從文字的大小、表格的間隙，到長條圖中每條 Bar 的距離，都可能影響到視覺障礙的使用者，也都可以在設計無障礙網站時考慮進去。

## 結論

不知道大家投身 Web 領域的動機是什麼，我想一定有部分人的理由與我相同，相信 Web 是最平易近人的媒介，能輕易把任何資訊帶給全世界，你不用存錢半天才能買到一台 iPhone 進入 App Store 下載應用程式，只需要到圖書館把電腦打開，透過瀏覽器即可連接世界。如果你也有同樣信念，除了加強 Web performance，讓存取網站的門檻降低外，提升 Web Accessibility 讓網站能真正服務到 "所有人"，絕對也是必修的課題之一，弭平資訊落差，落實公平正義，從你我做起！

P.S. Web Accessibility 的內容很多，這篇文章旨在引起大家對其的重視，詳細的規格與實作細節可以從底下的參考資料中去閱讀。

### 資料來源

1. [React rally 2019](https://youtu.be/dvtfNpt75aA?t=26101)
2. [Amphora](https://ethanmarcotte.com/wrote/amphora/)
3. [MDN - 可訪問性](https://developer.mozilla.org/zh-CN/docs/Learn/Accessibility)
4. [Google - 無障礙功能](https://developers.google.com/web/fundamentals/accessibility)
5. [回歸初心，一探 Web Accessibility](https://medium.com/frochu/%E5%9B%9E%E6%AD%B8%E5%88%9D%E5%BF%83-%E4%B8%80%E6%8E%A2web-accessibility-baaa4d22f4a7)
6. [實踐無障礙網頁設計（Web Accessibility）系列](https://ithelp.ithome.com.tw/users/20108045/ironman/2454)
7. [Accessibility in d3 Bar Charts](https://www.a11ywithlindsey.com/blog/accessibility-d3-bar-charts)

關於作者：
[@arvinh](http://blog.arvinh.info/about/) 前端攻城獅，熱愛數據分析和資訊視覺化
