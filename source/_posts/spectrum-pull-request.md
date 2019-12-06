---
title: 記一次幫開源專案 spectrum 修 bug 的經歷
date: 2019-04-20 08:27:21
tags:
  - open source
  - spectrum
  - github
author: huli
---

## 前言

最近又開始了自己的教學計畫，第一期的時候寫了這篇：[利用 Github Classroom 加 Travis CI 打造改作業系統](https://github.com/aszx87410/blog/issues/27)，第二期寫了這篇：[AWS Lambda + GitHub API + Google Sheet = 自動化簽到系統](https://github.com/aszx87410/blog/issues/32)，都是利用現成工具來快速湊出符合自己需求的系統。

而第三期開始前我希望課程能有個討論區，讓學生們可以在上面很方便地問問題。一直以來都是用 Slack，但 Slack 最大的缺點就是免費版會吃訊息，很多不錯的資訊被洗掉就覺得滿可惜的，希望能有個論壇或是討論區之類的地方會比較好。

在兩年前我也寫過一篇：[自架論壇的解決方案：flarum, github issue, nodeBB, discourse](http://huli.logdown.com/posts/1989995-the-forums-solution-flarum-discourse-github-issue)，研究了幾套方案，最後選擇 GitHub Issue。因為最簡單、最方便，但最大的缺點是學生好像不太習慣，因為左看右看上看下看其實都不像是個論壇。

前陣子因緣際會之下知道了這一個平台：[spectrum](https://spectrum.chat/)，首頁的 slogan 寫得很清楚：

> The community platform for the future.

去年被 GitHub 買下來之後變成完全免費的，以前付費版的功能也變免費了。在我看來，其實就是「更像討論區」的 Slack。先給大家看一張截圖：

![](/img/huli/spectrum/sc.png)

最左邊是不同的 workspace，這跟 Slack 一樣。再來你可以看到各個 Channles，這跟 Slack 也一樣，唯一不同的是右邊，原本 Slack 的訊息變成了一個個討論串，有標題跟內文。

這樣你大概可以懂我在說什麼了，這套跟 Slack 很像，但是更適合當作是討論區。

免費、背後是 GitHub、可以有 private 的論壇、開源，這簡直是完美的解決方案。除了沒有手機 App 以外，其他沒什麼可以挑剔的，就決定是這一套了！

## 事情沒有那麼順利...

開始試用幾天之後，發現有一個超級大問題，雖然說在功能上沒什麼問題，但是在體驗上我覺得極差，這一個小缺點就足夠讓我放棄這個平台。

是什麼問題呢？排版。

spectrum 原生支援 Markdown，使用起來十分順手，可是換行卻出了問題。在有些地方只有空行是沒有用的，結尾要加兩個空格才會換行，雖然我覺得這很不方便，但勉強可以接受。

可是！在 spectrum 上面，要兩個換行才會真的換行。

底下是範例，最下面的 line1 跟 line2 應該要換行：

![](/img/huli/spectrum/layout1.png)

可是 po 文之後卻會變成這樣：

![](/img/huli/spectrum/layout2.png)

換行變成了空格，如果是英文還好，但如果是中文的話排版就變得超級無敵奇怪，完全不能接受。

心灰意冷的我去了官方討論區[發文](https://spectrum.chat/spectrum/general/how-to-input-new-line-when-creating-a-new-post~2e53fc58-990a-433c-8f86-d2e28cdeaf87)，想說會不會有什麼其他換行的方法只是我不知道。

結果官方給我的回覆是：「對，現在你只能換行兩次才會真的換行」。

原本槁木死灰的我想說那就乾脆放棄吧，研究看看有沒有其他解法，甚至還一度想說要不要自己寫一套出來，但一想到要支援一大堆功能就覺得很麻煩，遲遲無法下定決心。

經過幾天的深思之後，覺得 spectrum 這個平台真的很好，但唯一的缺點就是排版問題，如果這個缺點解決了，沒道理不用它。

馬特拉不拉，我們自己拉。官方有 bug 沒空修，沒關係，我們自己修！這就是開源的好處。

## 修 bug 之旅

要幫開源專案修 bug 的第一步很簡單，就是想辦法把整個環境跑起來。你要有辦法在本機跑起來才有辦法驗證自己到底有沒有修成功，所以官方文件是很重要的。

[spectrum](https://github.com/withspectrum/spectrum) 的文件很齊全，有一連串的指示告訴你應該要怎麼做。照著做之後，就能把前後端都在自己的 local 跑起來了。

在等待安裝這些套件的途中，可以自己稍微猜一下問題出在哪裡。那時的我猜說應該是 markdown 的編輯器出了什麼問題，可能在把 markdown 轉成 HTML 的時候出了問題，沒有處理好，所以少了換行。

光猜是沒有用的，第一步要縮小問題範圍並且定位問題，先找出最重要的發文這一段到底發生了什麼事情。

在 Chrome 我們可以用 React Devtool，看到發文的介面是一個叫做 composer 的 component。接著在 [composer/index.js](https://github.com/withspectrum/spectrum/blob/0cef471b45779adcdfbb22dcc57884712c015e91/src/components/composer/index.js#L500) 裡面可以看到是由一個叫做 Inputs 的元件負責。

在 [Inputs.js](https://github.com/withspectrum/spectrum/blob/0cef471b45779adcdfbb22dcc57884712c015e91/src/components/composer/inputs.js#L54) 裡面發現了一件驚人的事情，原來按下 Preview 的時候，會直接送 request 到一個寫死的路徑並且把結果顯示出來：

``` js
const onClick = (show: boolean) => {
  setShowPreview(show);
  
  if (show) {
    setPreviewBody(null);
    fetch('https://convert.spectrum.chat/from', {
      method: 'POST',
      body,
    })
      .then(res => {
        if (res.status < 200 || res.status >= 300)
          throw new Error('Oops, something went wrong');
        return res.json();
      })
      .then(json => {
        setPreviewBody(json);
      });
  }
};
```

既然轉換是 Server 做的，那接下來就要來找找 Server 到底做了什麼。

可是我又不知道`https://convert.spectrum.chat/from`是對應到 Server 的哪裡，要怎麼找到 Server 是怎麼處理的呢？

這邊可以換一個想法，雖然說預覽的時候的確是送到這邊沒錯，但是發文的時候 Server 一定也會處理這個格式轉換，所以可以先找出發文的時候 Server 到底做了什麼，應該會有一些線索。

接著在前端發文之後查看 Network tab，因為後端是 GraphQL 所以滿好看的，是一個叫做`publushThread`的操作。

立刻往 Server 的部分找，循線找到了這個檔案：[publishThread.js](https://github.com/withspectrum/spectrum/blob/0cef471b45779adcdfbb22dcc57884712c015e91/api/mutations/thread/publishThread.js#L76)，並且發現裡面呼叫了一個 `processThreadContent` 來做轉換。

往下追這個 function，[看程式碼](https://github.com/withspectrum/spectrum/blob/0cef471b45779adcdfbb22dcc57884712c015e91/shared/draft-utils/process-thread-content.js#L12)之後發現這應該是最底層了：

``` js
// @flow
import { stateFromMarkdown } from 'draft-js-import-markdown';
import { convertFromRaw, convertToRaw, EditorState } from 'draft-js';
import { addEmbedsToEditorState } from './add-embeds-to-draft-js';
  
export default (type: 'TEXT' | 'DRAFTJS', body: ?string): string => {
  let newBody = body;
  if (type === 'TEXT') {
    // workaround react-mentions bug by replacing @[username] with @username
    // @see withspectrum/spectrum#4587
    newBody = newBody ? newBody.replace(/@\[([a-z0-9_-]+)\]/g, '@$1') : '';
    newBody = JSON.stringify(
      convertToRaw(
        stateFromMarkdown(newBody, {
          customBlockFn: elem => {
            if (elem.nodeName !== 'PRE') return;
  
            const code = elem.childNodes.find(node => node.nodeName === 'CODE');
            if (!code) return;
  
            const className = code.attributes.find(
              ({ name }) => name === 'class'
            );
            if (!className) return;
  
            const lang = className.value.replace('lang-', '');
  
            return {
              type: null,
              data: {
                language: lang,
              },
            };
          },
          parserOptions: {
            atomicImages: true,
            breaks: true,
          },
        })
      )
    );
  }
  
  // Add automatic embeds to body
  try {
    return JSON.stringify(addEmbedsToEditorState(JSON.parse(newBody || '')));
    // Ignore errors during automatic embed detection
  } catch (err) {
    console.error(err);
    return newBody || '';
  }
};
```

而且沒有看出任何跡象，看起來一切正常。此時的我想說：該不會要往下追到 draft-js 或是其他的 library 吧？

但是既然都找到這了，應該先來看一下它轉出來會是什麼東西，再決定下一步該怎麼辦，於是我在這個 function 加了 log，把它最後轉換的東西印出來。

我的輸入是：

```
oneline
newline  
thirdline
  
fourline
  
fiveline
```

輸出是：

``` js
{
  "blocks":[
    {
      "key":"bq56i",
      "text":"oneline\nnewline\nthirdline",
      "type":"unstyled",
      "depth":0,
      "inlineStyleRanges":[],
      "entityRanges":[],
      "data":{}
    },
    {
      "key":"9h38b",
      "text":"fourline",
      "type":"unstyled",
      "depth":0,
      "inlineStyleRanges":[],
      "entityRanges":[],
      "data":{}
    },
    {
      "key":"fuprm",
      "text":"fiveline",
      "type":"unstyled",
      "depth":0,
      "inlineStyleRanges":[],
      "entityRanges":[],
      "data":{}
    }
  ],
  "entityMap":{}
}
```

不印則已，一印驚人！

沒想到上面的測資轉換出來是：`"text":"oneline\nnewline\nthirdline"`，看來 Server 的轉換完全正常，換行被轉為`\n`，兩個換行被轉為一個新的 block，看來問題是出在前端沒有把這個換行好好輸出。

接著再用差不多的方法一樣用 React Devtool 來看，發現前端顯示是 [threadDetail.js](https://github.com/withspectrum/spectrum/blob/be3d7cc2b2bafec6715c7623db59c897902073ff/src/views/thread/components/threadDetail.js#L359) 在處理，而裡面呼叫了 threadRenderer.js，看來這就是真的 render 的地方了。

找到 [threadRenderer.js](https://github.com/withspectrum/spectrum/blob/c34bb1fa4b9957bcfcc6ff0165582e2f635bf5e7/src/components/threadRenderer/index.js#L4) 之後，發現裡面只是單純地呼叫了 [redraft](https://github.com/lokiuz/redraft) 這個 library。

好，雖然又有新的東西要研究，但離答案愈來愈近了。

仔細看了一下 redraft 的文件，看起來是可以自定義每一個型態最後的輸出要長什麼樣子。往下把官方文件看完，發現有一區是 [Common issues
](https://github.com/lokiuz/redraft#common-issues)，裡面寫著：

> Can the multiple spaces between text be persisted?
>    
> Add white-space: pre-wrap to a parent div, this way it will preserve spaces and wrap to new lines (as editor js does)

看到這邊，答案已經很明顯了，就是前端顯示忘記加 `white-space: pre-wrap`，所以預設的行為會把換行當作是空格。

真相大白的時候我在心裡暗罵了一聲髒話，但是是罵自己。因為這問題在前端其實滿常見的，我也用過這屬性很多次。可是在我看到這問題的時候我第一點居然是往後端去懷疑，完全沒想到有可能是前端的問題，更沒有想到原來是加一行 CSS 就可以搞定的事。

接著就先發了一個 [Issue](https://github.com/withspectrum/spectrum/issues/4885) 記錄了一下調查的過程跟成因，然後發了個 [PR](https://github.com/withspectrum/spectrum/issues/4885)，雖然只是改這麼一行而已，但是對我意義重大。因為只要這個 bug 修好，這套就立刻海放其他現成的論壇系統。

他們的速度很快，發 PR 之後隔天就被 merge 了，再隔個一週就被 deploy 到 production 了，真的很有效率。

## 意猶未盡，再修一個！

雖然只有一行，但探索的過程獲益良多，而且 PR 能被 merge 就很開心。既然都修了一個，那來找找看有沒有其他容易修的好了，可以一起順手修掉。

在官方的 Issues 翻一翻，找到一個看起來不難的：[Weird image failed rendering in thread body](https://github.com/withspectrum/spectrum/issues/4812)，這個 Issue 很簡單，就是不知道為什麼會出現下面的 bug：

![](/img/huli/spectrum/alt.png)

文字覆蓋住了後面的圖片。

Issue 裡面有附上原文網址，點進去以後用 devtool 看了一下，發現問題出在當瀏覽器無法載入 img 標籤的圖片時，就會變成這樣。

我之前完全沒碰過這問題，但自己試了一下，發現 img 原本有 margin，可是在圖片沒辦法載入的時候會失效。直覺告訴我這可能是 [margin collapsing](https://developer.mozilla.org/en-US/docs/Web/CSS/CSS_Box_Model/Mastering_margin_collapsing) 有關的問題。

後來我自己試了一下，問題出在當圖片無法載入，img 的高度就會變成 0，然後 margin 就會失效。因為一些排版跟 CSS 的元素，下面的文字就會蓋上來，變成下面的圖片那樣。

![](/img/huli/spectrum/height.png)

那有什麼好解法嗎？

我發現一個最簡單的解法就是加上 `alt` 屬性，當圖片無法載入時就會就會顯示這個文字，img 就能保有高度，margin 也能作用。

![](/img/huli/spectrum/height2.png)

查到解法之後一樣先回在 [Issue](https://github.com/withspectrum/spectrum/issues/4812) 下面跟他們討論，看他們覺得如何。

後來我發現原本上傳圖片時其實就有設定 alt，但可能在一些邊界條件下會是空的，或者是使用者手動把 alt 移除掉。

所以最後的解法也很簡單，就是幫 alt 加一個預設值，[PR: Add default alt text to img](https://github.com/withspectrum/spectrum/pull/4887)：

```
-  <img key={key} src={data.src} alt={data.alt} />,
+  <img key={key} src={data.src} alt={data.alt || 'Image'} />
```

## 總結

雖然只有貢獻了兩行，但能看到自己的帳號出現在 release log 上面還是滿開心的：

![](/img/huli/spectrum/re.png)

如果是以前的自己，我絕對不會幹這種事。絕對是發現 bug 之後就停住了，然後等著官方團隊來修 bug。

但這幾年漸漸開始熟悉起看其他人的 code，工作的時候偶爾沒事就可以看一下 redux-form 或是 redux 的 source code 等等，看著看著覺得也沒那麼可怕。而且 GitHub 還有個超好用功能叫做「搜尋」，很多時候直接搜關鍵字就能找到相關的原始碼，節省超級多時間。

在看其他人的專案時，我覺得最難的是定位問題，一但你定位問題之後其他都沒那麼難了，因為你已經知道是哪個檔案、哪段程式碼有問題，接著只要朝那邊去研究就好。至於該怎麼定位問題，有以下幾個建議：

1. 直接搜尋程式碼，看能不能找到相關段落
2. 利用 devtool 找出相關的元件
3. 看文件，看上面有沒有附一些架構

當你要修 bug 的時候，方向是很明確的，沒有必要整個專案都看過，只要找到你要修的地方就好。這一篇希望能分享我的經驗給大家。

最後，當個工程師真好，有開源專案真好，有 bug 都可以自己修掉。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好