---
title: 從 React 原始碼看 keyPress 與 keyDown 事件
date: 2019-03-23 09:26:04
tags: react
author: huli
---

## 前言

前陣子有個學生跑來問我一個問題，說他在寫 React 的時候分不清楚 keyPress 與 keyDown 這兩個事件，還有 keyCode 跟 charCode 這兩個東西，有時候拿得到值，有時候卻拿不到，覺得十分困惑。

我原本以為是 React 做了一些處理，所以去看了一下原始碼。後來發現 React 的確有做一些處理，但實際上這個問題跟 React 沒什麼關係，而是 keyPress 跟 keyDown 這兩個原生的 JavaScript 事件本來就有差異。

所以他碰到的問題跟 React 一點關係都沒有，只是對這部分的事件機制不熟而已。

儘管如此，能夠藉由一個實際的問題來參考一下 React 的實作還是件很不錯的事，而且 React 的註解寫得很好。

因此，這篇會先帶大家來看這兩個事件的不同，最後再來看 React 裡面怎麼做一些處理。

## keyPress 跟 keyDown 的差異

首先，我們要來看看 keyPress 與 keyDown 這兩個原生事件的差異到底在哪裡，這部分我們直接請出 MDN 來為我們做解釋：

> The keypress event is fired when a key that produces a character value is pressed down. Examples of keys that produce a character value are alphabetic, numeric, and punctuation keys. Examples of keys that don't produce a character value are modifier keys such as Alt, Shift, Ctrl, or Meta.

來源：https://developer.mozilla.org/en-US/docs/Web/Events/keypress

> The keydown event is fired when a key is pressed down.
> 
> Unlike the keypress event, the keydown event is fired for all keys, regardless of whether they produce a character value.

來源：https://developer.mozilla.org/en-US/docs/Web/Events/keydown

簡單來說呢，keyDown 會在你按下任何按鍵時觸發，但是 keyPress 只會在你按下的按鍵可以產生出一個字元的時候觸發，白話一點就是你按下這按鍵是在打字。

例如說你按`a`，畫面上會出現一個字元 a，所以 keyDown 跟 keyPress 都會觸發。但如果你按`shift`，畫面上什麼都不會出現，所以只有 keyDown 會觸發。

w3c 提供了一個很不錯的網頁：[Key and Character Codes vs. Event Types
](https://www.w3.org/2002/09/tests/keys.html)，讓你可以自己實驗看看。

下圖中我輸入 a，兩者都會觸發，接著我按 shift，只會觸發 keyDown，再來按 backspace 把文字刪掉，也只會觸發 keyDown：

![](/img/huli/keyboard/keyboard.gif)

所以這兩者的差異相信大家應該可以很清楚的知道了，keyDown 可以當作是「按下按鍵」，keyPress 則當作「輸入東西」時會觸發的事件。

接著我們來談談 keyCode 跟 charCode。

## keyCode 與 charCode 的差異

先來談談 charCode 好了，或許你有看過 JavaScript 裡面有個函式是這樣的：

``` js
console.log(String.fromCharCode(65)) // A
```

charCode 其實就是某一個字元所代表的一個號碼，或更精確一點地說，就是它的 Unicode 編碼。

這邊如果不太熟的話可以參考這篇文章：[[Guide] 瞭解網頁中看不懂的編碼：Unicode 在 JavaScript 中的使用](https://pjchender.blogspot.com/2018/06/guide-unicode-javascript.html)。

在 JavaScript 裡面也可以用另一個函式拿到字元所對應的編碼：

```
console.log('嗨'.charCodeAt(0)) // 21992
```

若是你把這 21992 轉成 16 進位，會變成 0x55E8，這個其實就是「嗨」的 Unicode：

![](/img/huli/keyboard/hi.png)  
（來源：https://www.cns11643.gov.tw/wordView.jsp?ID=90944）

那什麼是 keyCode 呢？既然 charCode 代表著是一個 char（字元）的 code，那 keyCode 顯然就是代表一個 key（按鍵）的 code。

每一個「按鍵」也都有一個它自己的代碼，而且有時候會讓你混淆，因為它跟 charCode 可能是一樣的。

舉例來說：「A」這個按鍵的 keyCode 是 65，而「A」這個字元的 charCode 也是 65。這應該是為了某種方便性所以這樣設計，但你要注意到一點：

> 當我按下「A」這個按鍵的時候，我可能要打的是 a 或是 A，有兩種可能

或是舉另外一個例子，當你要打數字 1 時，如果你是用 Q 上方的那顆按鍵而不是用純數字鍵盤，你要打的字可能是「1」或是「!」或甚至是「ㄅ」，因為它們都是同一顆按鍵。

一顆按鍵對應了不只一個字元，所以單單從 keyCode，你是沒辦法判斷使用者想打什麼字的。

講到這裡，我們可以來想一下這兩個跟 keyPress 與 keyDown 的關聯了。

剛剛說到 keyPress 是你要輸入文字的時候才會觸發，所以這個事件會拿到 charCode，因為你要知道使用者打了什麼字。那為什麼不是 keyCode 呢？因為你從 keyCode 根本不知道他打了什麼字，所以拿 keyCode 也沒用。

keyDown 則是在你按下任何按鍵時都會觸發，這時候一定要拿 keyCode，因為你要知道使用者按了什麼按鍵。若是拿 charCode 的話，你按 shift 或是 ctrl 就沒有值了，因為這不是一個字元，就沒辦法知道使用者按了什麼。

總結一下，當你要偵測使用者輸入文字的時候，就用 keyPress，並且搭配 charCode 來看使用者剛剛輸入了什麼；當你想偵測使用者「按下按鍵」的時候，就用 keyDown，搭配 keyCode 獲得使用者所按下的按鍵。

這就是 keyPress、keyDown 以及 keyCode 跟 charCode 的差別。

順帶一提，在輸入中文的時候 keyPress 不會有值，keyDown 則會回傳一個神秘的代碼 229：

![](/img/huli/keyboard/chinese.gif)

## key 與 which

在 keyPress 與 keyDown 這兩個 event 裡面，其實還有兩個屬性：key 與 which。

我們先來看一下 which 是什麼：

> The which read-only property of the KeyboardEvent interface returns the numeric keyCode of the key pressed, or the character code (charCode) for an alphanumeric key pressed.

來源：https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent/which

根據我自己的理解，當你在 keyPress 裡面用 which 的時候，拿到的應該就是 charCode；在 keyDown 裡面用的時候就是 keyCode，所以你在寫程式的時候可以統一用 event.which 來拿這個資訊，不必再區分 keyCode 或是 charCode。

不過 MDN 附的參考資料寫的滿模糊的，所以這部分我也不是很確定：

> which holds a system- and implementation-dependent numerical code signifying the unmodified identifier associated with the key pressed. In most cases, the value is identical to keyCode.

來源：https://www.w3.org/TR/2014/WD-DOM-Level-3-Events-20140925/#widl-KeyboardEvent-which

接著來看一下 key：

> The KeyboardEvent.key read-only property returns the value of the key pressed by the user while taking into considerations the state of modifier keys such as the shiftKey as well as the keyboard locale/layou

來源：https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent/key

簡單來說 key 會是一個字串，你剛剛按了什麼按鍵或是打了什麼字，key 就會是什麼。上面 MDN 的網頁下方有附一個簡單的範例讓你來測試 key 的值。

例如說我輸入 A，key 就是 A，按下 Shift，key 就是 Shift。

還有一點要注意的是，這個屬性在 keyPress 或是 keyDown 事件裡面都拿得到。所以儘管是 keyDown 事件，你也能知道使用者剛剛輸入了什麼或是按了什麼按鍵。

但儘管如此，關於「偵測輸入」的事件應該還是用 keyPress 最合適，除非你想要偵測其他不會產生字元的按鍵（Ctrl, Delete, Shift...）才用 keyDown 事件。

在這邊做個中場總結，其實這些 which、keyCode 跟 charCode，在不同瀏覽器上面都可能有不同的表現，所以是跨瀏覽器支援一個很麻煩的部分，從這個方向去找，你可以找到一大堆在講瀏覽器相容性的文章。

但近幾年來舊的瀏覽器漸漸被淘汰，大部分的使用者在用的瀏覽器應該都比較符合標準了，因此相容性並不是本篇文章的重點，所以就沒有多提了。

接下來終於要到可能是最吸引你的部分：React 原始碼。

## 初探 React 原始碼

React 原始碼這麼大，該從何找起呢？

這邊推薦一個超級好用的方法：GitHub 的搜尋。通常只要拿你想找的 function 名稱或是相關的關鍵字下去搜尋，就能夠把範圍限縮的很小，只要用肉眼再翻一下資料就能夠找到相對應的原始碼，是方便又好用的一個方法。

這邊我們用`keyPress`來當關鍵字，出現了 12 筆結果：

![](/img/huli/keyboard/search.png)

用肉眼稍微篩選一下，發現很多都是測試，那些都可以直接跳過。你應該很快就能定位到幾個相關的檔案，像是這兩個：

1. [packages/react-dom/src/events/SyntheticKeyboardEvent.js](https://github.com/facebook/react/blob/b87aabdfe1b7461e7331abb3601d9e6bb27544bc/packages/react-dom/src/events/SyntheticKeyboardEvent.js)
2. [packages/react-dom/src/events/getEventKey.js](https://github.com/facebook/react/blob/b87aabdfe1b7461e7331abb3601d9e6bb27544bc/packages/react-dom/src/events/getEventKey.js)

沒錯，這兩個就是今天的主角。

我們先來看`SyntheticKeyboardEvent.js`，如果你對 React 還算熟悉的話，應該知道你在裡面拿到的事件都不是原生的事件，而是 React 會包裝過之後再丟給你，而現在這個`SyntheticKeyboardEvent`就是經過 React 包裝後的事件，就是你在 onKeyPress 或是 onKeyDown 的時候會拿到的 `e`。

為了方便起見，我們切成幾個 function，一個一個來看。

``` js
charCode: function(event) {
  // `charCode` is the result of a KeyPress event and represents the value of
  // the actual printable character.
  
  // KeyPress is deprecated, but its replacement is not yet final and not
  // implemented in any major browser. Only KeyPress has charCode.
  if (event.type === 'keypress') {
    return getEventCharCode(event);
  }
  return 0;
}
```

這邊註解寫得很棒，説 keyPress 已經被 deprecated 了但是替代品還沒準備好。再者，也提到了只有 keyPress 有 charCode。

所以這邊就是判斷 event 的 type 是不是 keypress，是的話就回傳`getEventCharCode(event)`，否則回傳 0。

接著我們來看一下`getEventCharCode`在做什麼（小提醒，這個函式在另外一個檔案）：

``` js
/**
 * `charCode` represents the actual "character code" and is safe to use with
 * `String.fromCharCode`. As such, only keys that correspond to printable
 * characters produce a valid `charCode`, the only exception to this is Enter.
 * The Tab-key is considered non-printable and does not have a `charCode`,
 * presumably because it does not produce a tab-character in browsers.
 *
 * @param {object} nativeEvent Native browser event.
 * @return {number} Normalized `charCode` property.
 */
function getEventCharCode(nativeEvent) {
  let charCode;
  const keyCode = nativeEvent.keyCode;
  
  if ('charCode' in nativeEvent) {
    charCode = nativeEvent.charCode;
  
    // FF does not set `charCode` for the Enter-key, check against `keyCode`.
    if (charCode === 0 && keyCode === 13) {
      charCode = 13;
    }
  } else {
    // IE8 does not implement `charCode`, but `keyCode` has the correct value.
    charCode = keyCode;
  }
  
  // IE and Edge (on Windows) and Chrome / Safari (on Windows and Linux)
  // report Enter as charCode 10 when ctrl is pressed.
  if (charCode === 10) {
    charCode = 13;
  }
  
  // Some non-printable keys are reported in `charCode`/`keyCode`, discard them.
  // Must not discard the (non-)printable Enter-key.
  if (charCode >= 32 || charCode === 13) {
    return charCode;
  }
  
  return 0;
}
```

接著我們一樣分段來看比較方便：

``` js
/**
 * `charCode` represents the actual "character code" and is safe to use with
 * `String.fromCharCode`. As such, only keys that correspond to printable
 * characters produce a valid `charCode`, the only exception to this is Enter.
 * The Tab-key is considered non-printable and does not have a `charCode`,
 * presumably because it does not produce a tab-character in browsers.
 *
 * @param {object} nativeEvent Native browser event.
 * @return {number} Normalized `charCode` property.
 */
```

開頭的註解先跟你說 charCode 代表的就是 character code，所以可以用 String.fromCharCode 來找出搭配的字元。

因此，只有能被印出來（或者是說可以被顯示出來）的字元才有 charCode，而 Enter 是一個例外，因為 Enter 會產生空行。但 Tab 不是，因為你按 Tab 不會產生一個代表 Tab 的字元。

``` js
let charCode;
const keyCode = nativeEvent.keyCode;
  
if ('charCode' in nativeEvent) {
  charCode = nativeEvent.charCode;
  
  // FF does not set `charCode` for the Enter-key, check against `keyCode`.
  if (charCode === 0 && keyCode === 13) {
    charCode = 13;
  }
} else {
  // IE8 does not implement `charCode`, but `keyCode` has the correct value.
  charCode = keyCode;
}
```

這邊針對瀏覽器的相容性做處理，FireFox 沒有幫 Enter 設定 charCode，所以要額外判斷 keyCode 是不是 13。然後 IE8 沒有實作 charCode，所以用 keyCode 的值來取代。

``` js
// IE and Edge (on Windows) and Chrome / Safari (on Windows and Linux)
// report Enter as charCode 10 when ctrl is pressed.
if (charCode === 10) {
  charCode = 13;
}
  
// Some non-printable keys are reported in `charCode`/`keyCode`, discard them.
// Must not discard the (non-)printable Enter-key.
if (charCode >= 32 || charCode === 13) {
  return charCode;
}
```

這邊應該算是一個 special case，當使用者按下 Ctrl + Enter 時的 charCode 是 10，React 想把這個也當作按下 Enter 來處理。

另外，有些沒辦法被印出來的字元應該要被拿掉，所以最後做了一個範圍的判斷。

charCode 的處理就是這樣了，仔細看看其實還滿有趣的，針對瀏覽器的相容性跟一些特殊狀況做了處理。

接著我們回到`SyntheticKeyboardEvent.js`，來看看 keyCode 的處理：

``` js
keyCode: function(event) {
  // `keyCode` is the result of a KeyDown/Up event and represents the value of
  // physical keyboard key.
  
  // The actual meaning of the value depends on the users' keyboard layout
  // which cannot be detected. Assuming that it is a US keyboard layout
  // provides a surprisingly accurate mapping for US and European users.
  // Due to this, it is left to the user to implement at this time.
  if (event.type === 'keydown' || event.type === 'keyup') {
    return event.keyCode;
  }
  return 0;
}
```

這邊說 keyCode 的值其實是依賴於鍵盤的，意思是說有些鍵盤可能會產生不太一樣的 keyCode，但因為大多數美國跟歐洲的使用者都是 US keyboard，所以這邊就直接把 keyCode 丟回去而不做特殊處理。

其實這一段我沒有看得完全懂，只是大概猜一下意思而已。這邊指的「keyboard layout」可能是像 QWERTY 或是 Dvorak 這種的 layout，按鍵的排列方式完全不同。但如果這樣就會產生不同的 keyCode 的話，是不是代表有些網站可能會有 bug？

不過大多數人的鍵盤都是同樣的排列，所以好像不用太擔心這個問題。

``` js
which: function(event) {
  // `which` is an alias for either `keyCode` or `charCode` depending on the
  // type of the event.
  if (event.type === 'keypress') {
    return getEventCharCode(event);
  }
  if (event.type === 'keydown' || event.type === 'keyup') {
    return event.keyCode;
  }
  return 0;
}
```

最後是 which 的部分，如果是 keypress 就把 charCode 傳回去，keydown 或是 keyup 的話就把 keyCode 傳回去。

講到這裡，我們已經看到 React 對於 charCode、keyCode 以及 which 的處理了，charCode 針對特殊情形以及瀏覽器相容性做檢查，keyCode 直接回傳，which 則根據事件不同回傳相對應的值。

最後我們來看一下 key 的處理，這邊放在另外一個檔案叫做`getEventKey.js`：

``` js
/**
 * Normalization of deprecated HTML5 `key` values
 * @see https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent#Key_names
 */
const normalizeKey = {
  Esc: 'Escape',
  Spacebar: ' ',
  Left: 'ArrowLeft',
  Up: 'ArrowUp',
  Right: 'ArrowRight',
  Down: 'ArrowDown',
  Del: 'Delete',
  Win: 'OS',
  Menu: 'ContextMenu',
  Apps: 'ContextMenu',
  Scroll: 'ScrollLock',
  MozPrintableKey: 'Unidentified',
};
  
/**
 * Translation from legacy `keyCode` to HTML5 `key`
 * Only special keys supported, all others depend on keyboard layout or browser
 * @see https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent#Key_names
 */
const translateToKey = {
  '8': 'Backspace',
  '9': 'Tab',
  '12': 'Clear',
  '13': 'Enter',
  '16': 'Shift',
  '17': 'Control',
  '18': 'Alt',
  '19': 'Pause',
  '20': 'CapsLock',
  '27': 'Escape',
  '32': ' ',
  '33': 'PageUp',
  '34': 'PageDown',
  '35': 'End',
  '36': 'Home',
  '37': 'ArrowLeft',
  '38': 'ArrowUp',
  '39': 'ArrowRight',
  '40': 'ArrowDown',
  '45': 'Insert',
  '46': 'Delete',
  '112': 'F1',
  '113': 'F2',
  '114': 'F3',
  '115': 'F4',
  '116': 'F5',
  '117': 'F6',
  '118': 'F7',
  '119': 'F8',
  '120': 'F9',
  '121': 'F10',
  '122': 'F11',
  '123': 'F12',
  '144': 'NumLock',
  '145': 'ScrollLock',
  '224': 'Meta',
};
  
/**
 * @param {object} nativeEvent Native browser event.
 * @return {string} Normalized `key` property.
 */
function getEventKey(nativeEvent: KeyboardEvent): string {
  if (nativeEvent.key) {
    // Normalize inconsistent values reported by browsers due to
    // implementations of a working draft specification.
  
    // FireFox implements `key` but returns `MozPrintableKey` for all
    // printable characters (normalized to `Unidentified`), ignore it.
    const key = normalizeKey[nativeEvent.key] || nativeEvent.key;
    if (key !== 'Unidentified') {
      return key;
    }
  }
  
  // Browser does not implement `key`, polyfill as much of it as we can.
  if (nativeEvent.type === 'keypress') {
    const charCode = getEventCharCode(nativeEvent);
  
    // The enter-key is technically both printable and non-printable and can
    // thus be captured by `keypress`, no other non-printable key should.
    return charCode === 13 ? 'Enter' : String.fromCharCode(charCode);
  }
  if (nativeEvent.type === 'keydown' || nativeEvent.type === 'keyup') {
    // While user keyboard layout determines the actual meaning of each
    // `keyCode` value, almost all function keys have a universal value.
    return translateToKey[nativeEvent.keyCode] || 'Unidentified';
  }
  return '';
}
```

這邊一樣是針對瀏覽器的相容性做處理，如果 event 本身就有 key 的話，先做 normalize，把回傳的結果統一成相同的格式。而 FireFox 會把可以印出的字元都設定成 MozPrintableKey，這邊 normalize 成 `Unidentified`。

如果 normalize 完之後的 key 不是`Unidentified`的話就回傳，否則再做進一步處理。

而這個進一步處理指的就是 polyfill，如果沒有 key 可以用的話就自己針對 charCode 或是 keyCode 來做處理，回傳相對應的字元或是按鍵名稱。

React 對於這些按鍵相關事件的處理就到這邊差不多了。

原始碼註解寫的很好，可以獲得很多相關資訊，而程式碼很短又不複雜，看起來也很輕鬆，是個很適合入門的切入點。

## 總結

以前用了這麼多次這些按鍵相關事件，我自已卻從來沒想過這些的區別。要嘛就是隨意寫寫然後出 bug，要嘛就是直接從 stackoverflow 上面複製最佳解答，從來都不知道這些的差異。

這次剛好是因為要幫人解惑才去深入研究，沒想到一個簡單的按鍵事件其實也是水很深，可能要真的踩過雷才會更有感觸。最麻煩的其實是瀏覽器的相容性，各個瀏覽器可能都有自己不同的實作，要怎麼處理這些不同的情況才是麻煩的地方。

提到 React 原始碼，大家想到的可能都是 render 的相關機制或是 component 的處理，那些原始碼十分複雜，而且必須要對整體的架構有一定的理解才比較好看懂。

這篇選擇從 keyboard 的事件出發，來看 React 針對這部份的處理。相信程式碼大家都看得懂，也不會覺得特別難，就是想告訴大家若是你想研究其他人的原始碼，有時候不一定要整個專案都看懂，可以先從一些小地方開始下手。

從 utils 這種簡單的 function 開始也行，不一定要從最難的開始挑戰，你都能學到很多東西。

關於作者：  
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好