---
title: "React Form: Redux Form vs React Final Form vs Formik and Yup"
date: 2019-05-03 18:28:23
tags: react
author: cythilya
---
![React Form: Redux Form vs React Final Form vs Formik & Yup](https://cythilya.github.io/assets/react-form/hacking-4154618_1280.jpg)

本文會先從為何要做表單狀態管理說起，接著看目前市面上有哪些好的表單函式庫和條列挑選原則，並探討三個表單函式庫 Redux Form、React Final Form、Formik & Yup，最後做比較和總結。

## 為何需要做表單狀態管理？

為什麼需要做表單狀態管理呢？這就要從 controlled component 和 uncontrolled component 開始談起。

在一般 HTML 的世界裡面，表單的狀態是由元件本身來做儲存和更新的，稱之為「uncontrolled component」；而在 React 的世界裡，表單的狀態和值的更新是由開發者處理，如下圖所示，表單欄位的值可從 props 或 state 取得，在這裡是 state，並且當使用者打字等行為時觸發事件來做值的更新，在這裡是觸發 onChange 事件，這樣的元件稱為「controlled component」。這樣值的儲存與更新的管理方式，就是表單簡易的狀態管理的例子，表單還有其他狀態要管理，例如：表單是否合法（valid）、各欄位的錯誤訊息（validation、error）、欄位值是否被更改過（dirty）、初始值的設定（initial value）、各種 callback 的設定等。

![Controlled vs uncontrolled component](https://cythilya.github.io/assets/react-form/react-form-controlled-vs-uncontrolled-component-1.png)

也因為由開發者來管理表單的狀態，所以我們可能需要自己刻一套管理工具，或是使用市面上大師們已經實作好的函式庫就好了，不管是自已刻還是用別人刻好的，在專案的開發上，都可以達到兩個效果...

- 不用重造輪子，節省開發時間，若每次實作表單都要重做一次表單狀態管理機制就太辛苦了 XD
- 若是多人開發專案，就能統一彼此的實作方式，減少維護的難度。

## 當紅的表單函式庫

先不要談自己刻這件事，來看看大師們幫我們刻好的表單函式庫有哪些...

![Popular form libraries in React](https://cythilya.github.io/assets/react-form/popular-react-form-lib.png)

由上圖可知，可選擇的表單函式庫真的很多，大致上可分類為功能完整、輕量或內建驗證工具等共三種，另外還有一些是進入維護狀態、不再開發新功能的，就不納入評選。

- 功能完整是指 API 開得很充足的函式庫，可讓開發者實作細緻的表單元件，當然也能減少撰寫程式碼，例如：Redux Form（功能真是包山包海）、React Final Form、Formik、Informed。
- 強調輕量亦即打包後的檔案很小，適合對體積大小有顧慮的專案，例如：Final Form、React Final Form、Formsy React。
- 內建驗證工具的函式庫讓開發者能很方便的撰寫驗證規則，例如：Formik、Informed、React Forms。

接下來我選擇 Redux Form、React Final Form 和 Formik & Yup 來做討論。

## 如何選擇好的表單函示庫？

先綜觀來看怎麼選擇好的表單函示庫。一般來說，這些函式庫主要都是做狀態管理，而它們都做得很好，只是有些差異，因此歸納了一些挑選原則...

- 這個函式庫是怎麼做狀態管理的？會不會有效能問題？
- API 充足嗎？能為開發者減少撰寫多少程式碼？通常 API 充足的函式庫，打包後的檔案體機會比較大，只能取捨一下了。
- 如何撰寫驗證邏輯？
- 擴充性如何？是否能輕易新增和移除功能以符合未來的需求？
- 文件是否詳細？範例是否充足？之後若遇到問題是否能很快被解決？
- 打包後的檔案大小。
- Github 星星數 and NPM 下載數。這是比較不重要的考量點，畢竟過去火紅的專案，目前可能已不符合需求，但已長時間累積了很多的星星數和下載數；而新出來的專案可能因起步較晚而沒有很多的星星數和下載數，但做得卻非常好。
- 函式庫提供的特點是否能解決專案的需求？

## Redux Form

![Redux Form](https://cythilya.github.io/assets/react-form/redux-form.png)

Redux Form 利用 Redux 來儲存整個 app 的表單狀態，再根據當前所需提取特定表單資料。也就是說，先在 Redux store 建立表單的 reducer，經由 [HOC](https://reactjs.org/docs/higher-order-components.html) 的方式連接表單和 store，再由 props 帶入資料。

### Workflow

![Redux Form Workflow](https://cythilya.github.io/assets/react-form/redux-form-diagram.png)

圖片來源：[Redux Form - Getting Started](https://redux-form.com/8.2.0/docs/gettingstarted.md/)

Redux Form 的工作流程是這樣的...

當使用者對表單元件輸入資料時會發出 action 去更新 store，當狀態被更新，就會重新渲染元件，使用者就能看到剛才輸入的資料。

### Demo

#### 範例 1

這是一個簡易的表單範例，並且使用 redux-form-validators 作為驗證的工具。

![Redux Form 範例](https://cythilya.github.io/assets/react-form/redux-form-example-1.png)

在這個範例中有兩個欄位 Name 和 Email，初始值就不是合法的了，所以若在此時按下按鈕 Submit 提交表單，會看到 Name 下方紅色的錯誤訊息「Length must exceed 3 characters.」

![Redux Form 範例](https://cythilya.github.io/assets/react-form/redux-form-example-2.png)

在修正欄位值後，就可以順利提交了。

![Redux Form 範例](https://cythilya.github.io/assets/react-form/redux-form-example-3.png)

來看原始碼，如果想得到特定表單的資訊，必須從 reducer 中使用 selector 提取出來，然後再用 props 帶進表單，例如：formData、formValues 和 formErrors。formData 會取得目前表單所有欄位的狀態，例如這個欄位是否被 touched、dirty 或目前 focus 在哪個欄位等；formValues 取得目前表單各欄位的值；formErrors 取得目前表單各欄位的錯誤訊息。

```javascript
Form = connect((state) => ({
  formMeta: getFormMeta('register')(state), // get form data by using selectors
  formValues: getFormValues('register')(state),
  formErrors: getFormSyncErrors('register')(state),
}))(Form);
```

在 selector 這裡要輸入唯一的表單名稱「register」，我常常不是打錯字就是輸入重複的名稱，感到困擾和麻煩 XD

另外，在這裡搭配 [redux-form-validators](https://www.npmjs.com/package/redux-form-validators) 作為驗證工具，它提供一些簡易的 field-level validation 驗證規則，並且可自訂錯誤訊息，也可經由 addValidator 加入驗證規則。

簡易使用方式如下，將 name 這個欄位加上驗證規則必填（required）和字數限制（length，必須超過 4 個字），並自訂報錯訊息。

[Demo](https://726wprxw3q.codesandbox.io/redux-form/register)，[原始碼](https://codesandbox.io/s/726wprxw3q)。

```javascript
<Field
  name='name'
  validate={[
    required({ message: 'Required.' }),
    length({ min: 4, message: 'Length must exceed 3 characters.' }),
  ]}
/>
```

#### 範例 2

這是一個混合同步和非同步驗證的範例，同步驗證像是名稱是否必填、字數限制至少 4 個字，email 是否合法；非同步驗證像是詢問使用者的名稱是否已被使用，輸入「paul」表示已被別人使用了，這是模擬從伺服器端回傳驗證結果，再顯示錯誤訊息的範例。

![Redux Form 範例](https://cythilya.github.io/assets/react-form/redux-form-example-4.png)

[Demo](https://726wprxw3q.codesandbox.io/redux-form/feedback)，[原始碼](https://codesandbox.io/s/726wprxw3q)，其他範例還有[建立動態欄位](https://726wprxw3q.codesandbox.io/redux-form/contact)。

### 優點

- 提供充足的 API 能實作細緻的元件。
- 可混合同步與非同步驗證。
- 由於 Redux Form 本身並沒有提供驗證工具，但可自行實作驗證的部份或搭配他人寫好的工具，例如：[redux-form-validators](https://www.npmjs.com/package/redux-form-validators) 或 [redux-form-yup](https://www.npmjs.com/package/redux-form-yup)，相對是很有彈性的。

### 缺點

- 表單狀態不需要存在全域的 store 裡面，畢竟表單狀態與其他原件無關。
- 每當表單更新狀態就會更新 store，亦即使用者每打一個字都會做更新，整個表單就會重新渲染，造成不需要更新的欄位也被重新渲染，影響效能（備註），這在表單很大的時候，延遲狀況尤其明顯。之後我們會看到兩個效能較佳的表單函式庫-React Final Form 和 Formik。React Final Form 有訂閱表單和欄位狀態的機制；而 Formik 有 Fastfield 元件，利用實作 shouldComponentUpdate 的機制決定要不要重新渲染該欄位，都能有效限制渲染的次數。
- 必須使用 Redux。若 app 很小，不需使用 Redux 做狀態管理，卻必須因為表單而使用 Redux 才能用 Redux Form；又或者是 app 並非使用 Redux 做狀態管理就不能使用 Redux Form。
- 壓縮後的打包大小較大，超過 26.7KB。React Final Form + Final Form 是 7.7KB，而 Formik 是 12KB。
- 非同步驗證只支援 form-level 的驗證，在實作上來說，若能在欄位設定驗證規則是比較直覺和方便的。
- 文件和範例不夠充足、不夠詳細。
- 缺少擴充性。

關於以上的缺點，不管是表單狀態的儲存、打包大小、文件與範例、擴充性等，接下來我們會看到兩個做得更好的函式庫-React Final Form 和 Formik。

**備註**：減少不必要的渲染是指在 React 做 virtaul dom 的比對而決定是否要渲染前，可利用元件實作的機制而省下這複雜計算過程，因此能減少延遲效果。

## Final Form

![Final Form](https://cythilya.github.io/assets/react-form/final-form-cover.png)

在看 React Final Form 之前，先來看它的狀態管理引擎 Final Form。由於 Final Form 是表單狀態管理的引擎，因此與框架無關，並且可以獨立使用或實作 React 或 Vue 的 wrapper 包裝後來使用它。

Final Form 的特點如下

- 由於是狀態管理引擎，因此與框架無關，可以獨立使用或實作 React 或 Vue 的 wrapper 包裝後來使用它。
- Final Form 利用訂閱的機制來選擇要追蹤的狀態，若追蹤的狀態有更新再通知更新元件。
- 表單的狀態是存在 Final Form 的 form instance 的 state，不像 Redux Form 是存在全域的 store 裡面，因此與任何類似 flux 狀態管理工具完全無相依關係。
- 擴充性高，可利用 decorator 的方式輕易的新增或移除功能以符合未來的需求。
- 沒有和其他函式庫相依。
- 打包後的檔案很小，壓縮後只有 4.7KB。

### Demo

#### 範例 1

這是一個簡單的範例，示範如何使用 Final Form。

如下圖所示，同樣也是填寫 Name 和 Email 的簡易表單，由於 Name 必須超過 3 個字，因此提交按鈕是 disabled 狀態，無法送出。

![Final Form 範例](https://cythilya.github.io/assets/react-form/final-form-example-1.png)

blur 欄位 Name 後可看到錯誤訊息。

![Final Form 範例](https://cythilya.github.io/assets/react-form/final-form-example-2.png)

修正欄位後，提交按鈕變成 enabled 狀態，即可送出。

![Final Form 範例](https://cythilya.github.io/assets/react-form/final-form-example-3.png)

從實際程式碼來看 Final Form 的使用方式。

利用 createForm 建立表單，並指定三個參數 initialValues、onSubmit、validate，分別是表單的初始值、提交時呼叫的 callback 和驗證規則，其中 onSubmit 必填。

```javascript
import { createForm } from 'final-form';

const form = createForm({ initialValues, onSubmit, validate });
```

訂閱表單要監聽的屬性，例如：表單是否合法（valid）、目前在哪個欄位（active）、目前表單所有欄位的值（values）、哪些欄位已被修改過（dirty）。

```javascript
// Subscribe to form state updates
const unsubscribe = form.subscribe(
  formState => {
    // Update UI
  },
  { // FormSubscription: the list of values you want to be updated about
    active: true,
    dirty: true,
    valid: true,
    values: true
  }
})
```

訂閱欄位要監聽的屬性，例如：錯誤訊息（error）、是否被觸碰（touched）、目前的值（value）等。

```javascript
// Subscribe to field state updates
const unregisterField = form.registerField(
  'name',
  (fieldState) => {
    // Update field UI
    const { blur, change, focus, ...rest } = fieldState;
    // In addition to the values you subscribe to, field state also includes functions that your inputs need to update their state.
  },
  {
    // FieldSubscription: the list of values you want to be updated about
    error: true,
    touched: true,
    value: true,
  },
);
```

[Demo](https://726wprxw3q.codesandbox.io/final-form/register/)，[原始碼](https://codesandbox.io/s/726wprxw3q)。

#### 範例 2：React wrapper for Final Form

簡單實作 React wrapper 來使用 Final Form。同樣也是填寫 Name 和 Email 的簡易表單，由於 Name 必須超過 3 個字，因此提交按鈕是 disabled 狀態，無法送出。

![Final Form 範例](https://cythilya.github.io/assets/react-form/final-form-wrapper-example.png?1234)

程式碼範例如下，這裡用一個 Form wapper 把 Final Form 包起來，開發者只要依舊指定 initialValues、onSubmit、validate，並且設定欄位要顯示的名稱和屬性 name。表單指定要監聽四個屬性 valid、pristine、submitting、values，欄位則是沒有指定就是監聽全部屬性。

```javascript
<Form // (1) create form
  initialValues={{
    name: 'Ann',
    email: 'sample@test.com',
  }}
  onSubmit={(values) => {
    alert(JSON.stringify(values, 0, 2));
  }}
  validate={validate}
  // (2) subscribe form state
  subscription={['valid', 'pristine', 'submitting', 'values']}
>
  {/* (3) subscribe all field state */}
  <Form.Field label='Name' name='name' />
  <Form.Field label='Email' name='email' />
</Form>
```

[Demo](https://726wprxw3q.codesandbox.io/final-form/contact/)，[原始碼](https://codesandbox.io/s/726wprxw3q)。

## React Final Form

![Reacg Final Form](https://cythilya.github.io/assets/react-form/react-final-form-cover.png)

- React Final Form 將 Final Form 包裝起來，這樣就能在 React 的環境中使用。也就是說，由 Final Form 保存狀態，React Final Form 只是一層 wrapper。
- 由剛剛提到的 Final Form 可知，Final Form 只提供訂閱的狀態，這是因為使用者每個輸入或任何動作都會造成重新渲染，改善方式是表單可訂閱想要被通知的狀態，當被訂閱的狀態有更新時才做通知。注意，如果不指定訂閱的狀態就是訂閱全部的狀態，也無法啟動欄位的狀態訂閱了。
- 非常輕量，壓縮後的打包檔案只有 3KB。

### Workflow

![Workflow](https://cythilya.github.io/assets/react-form/react-final-form-workflow.png)

說明

- React Final Form 的工作流程如這張圖所示，React Final Form 將表單的狀態存在 React Final Form 的 form instance 裡面。
- 當使用者對某個 input 打字，這時候就會去更新表單的中有訂閱的狀態。
- 接著，由於狀態被修改，因此重新渲染有訂閱且更新狀態的欄位，使用者就能看到剛才輸入的值和更新後的表單狀態。

### Demo

#### 範例 1：Decorators

React Final Form 的擴充性高，可利用 decorator 的方式輕易的新增或移除功能以符合未來的需求，並且也能保持 React Final Form 的體積很小。

例如，我們希望在提交表單後，能 foucs 在第一個有錯的欄位上，那就可以裝上 final-form-focus 這個 decorator 協助達成。

如下圖所示，表單的兩個欄位 Name 與 Password 皆為必填但未填，因此按下提交按鈕後，都顯示錯誤訊息，並因為裝了 final-form-focus 而能將游標停在第一個欄位，也就是欄位 Name 當中。

![React Final Form: Decorators](https://cythilya.github.io/assets/react-form/react-final-form-example-1.png)

[Demo](https://726wprxw3q.codesandbox.io/react-final-form/login)，[原始碼](https://codesandbox.io/s/726wprxw3q)。

#### 範例 2：利用訂閱機制改善效能問題

一開始這個表單的欄位全部都沒有通過驗證規則，例如：name 為必填但是目前沒有值，email 不合規則，此時表單訂閱的狀態中 valid 值為 false。

![React Final Form: 利用訂閱機制改善效能問題](https://cythilya.github.io/assets/react-form/react-final-form-example-2.png)

name 和 email 彼此不會影響渲染，只有自己狀態改變，例如因打字輸入而改變 value 時才會重新渲染。

![React Final Form: 利用訂閱機制改善效能問題](https://cythilya.github.io/assets/react-form/react-final-form-example-3.png)

由於表單有訂閱狀態 valid，因此只有當全部欄位都通過驗證，也就是 valid 由 false 變成 true 時，表單才會重新渲染，可以看到渲染次數由 2 變成 3。

![React Final Form: 利用訂閱機制改善效能問題](https://cythilya.github.io/assets/react-form/react-final-form-example-4.png)

### 優點

- 有足夠的 API 能實作細緻的 UI，這樣開發者就不需要重新實作取得表單狀態的方法，而能減少撰寫的程式碼。
- API 和 Redux Form 相似，如果之前用 Redux Form 就不用重新學習。
- 將狀態存在 Final Form instance state，而非存在全域的 store。
- 針對效能的改善，React Final Form 提供訂閱機制來減少不必要的渲染。也就是說，當內部狀態有改變時才去呼叫 `setState()` 來做重新渲染的動作。
- 打包後的檔案很小，壓縮後只有 7.7KB。
- 表單和欄位層級都支援同步與非同步驗證。
- 擴充性高，可利用 decorator 的方式輕易的新增或移除功能以符合未來的需求，而因為這樣而能保持打包後的體積很小。

### 缺點

- 無內建驗證工具，目前也沒有好的可搭配的驗證工具，驗證部份要自己開發。

## Formik & Yup

![Formik](https://cythilya.github.io/assets/react-form/formik-cover.png)

- React 官方推薦，據說是可以無痛建立表單。
- [Formik](https://github.com/jaredpalmer/formik) 是表單函式庫，[Yup](https://github.com/jquense/yup) 是驗證工具，Formik 有個 config「validationSchema」可和 Yup 互相搭配，有設定 validationSchema 這個 config 時，就能將 Yup 回傳的錯誤訊息自動轉換成一個物件，其 key 是欄位的 name，而 value 就是錯誤訊息。

### Workflow

Formik 的工作流程是這樣的...

![Workflow](https://cythilya.github.io/assets/react-form/formik-workflow.png)

說明

- Formik 將表單的狀態存在 form instance 的 state 裡面。
- 當使用者對某個 input 打字時就會去更新表單的狀態。
- 接著，由於狀態被修改，因此重新渲染表單，使用者看到剛才輸入的值和更新後的元件。

### Yup 好在哪裡？

#### 不需再看到 if/else 判斷句

傳統的寫法是用 if/else 根據條件判斷要用哪些規則，雜亂且難以維護。如下範例，表單欄位中有 name 這個欄位，若 name 沒填則顯示錯誤訊息「Required.」（必填），若 name 的字數小於 4 個字，則報錯「Length must exceed 3 characters.」（字數需要超過 3 個字）。

```javascript
if (!values.name) {
  errors.name = 'Required.';
} else if (values.name.length < 4) {
  errors.name = 'Length must exceed 3 characters.';
}
```

利用 Yup 改寫後，簡單清楚易懂。

```javascript
const schema = Yup.object().shape({
  name: yup
    .string()
    .required('Required.')
    .min(4, 'Length must exceed 3 characters.'),
});
```

#### Cross-validation

下圖是一個簡單的電子報訂閱表單的範例，驗證規則是如果沒有勾選要訂閱電子報（subscribe）的話，填完名稱（name）就可以送出；若要訂閱電子報，勾選「訂閱電子報」之後，信箱（email）欄位成為必填，因此就必須填寫。

![Formik: Cross-validation](https://cythilya.github.io/assets/react-form/formik-example-1.png)

利用 Yup 可以很輕易地做到根據條件動態決定驗證規則，如下程式碼所示，email 的規則是根據 subscribe 的值而決定的，`when` 後接要觀察的欄位名稱，在此觀察 subscribe 這個 checkbox 是否被勾選，若有勾選（亦即 `is` 為 true），則做 `then` 後所接的事情，否則做 `otherwise` 後所接的事情，因為這裡沒有「否則」要做什麼，所以程式碼中就沒有示範了。

```javascript
email: yup.string().when('subscribe', {
  is: true,
  then: fieldSchema => fieldSchema
    .required('Required.')
    .isEmail('Invalid email address.'),
}),
subscribe: yup.boolean(),
```

[Demo](https://726wprxw3q.codesandbox.io/formik/subscribe)，[原始碼](https://codesandbox.io/s/726wprxw3q)。

#### 可混合同步和非同步的驗證

可混合同步和非同步的驗證，而且可以用這樣的方式撰寫 `.sync().async().sync().async()`...簡潔易懂。

```javascript
validationSchema: yup.object().shape({
  name: yup
    .string()
    .required('Required.')
    .isNameAvailable('Name is taken!') // 非同步
    .min(4, 'Length must exceed 3 characters.')
  })
})
```

[Demo](https://726wprxw3q.codesandbox.io/formik/contact)，[原始碼](https://codesandbox.io/s/726wprxw3q)。

#### 有用的小工具

Yup 提供一些有用的工具，像是去除前後空白、將字串轉為全部大寫或小寫等。

如下範例所示，當使用者輸入字串時，可能前後都有空白，這時候 Yup 可先將字串去除前後空白後再做驗證，而不會造成空白也是一個字元的狀況。

例如，在以下程式碼的狀況下，輸入五個空白或字串「Alice」，都是可以通過驗證的。

```javascript
name: yup.string().required('Required.');
```

若加上 `trim()`，則五個空白會被去除，而顯示錯誤訊息；字串「Alice」依然可以通過驗證。

```javascript
name: yup
  .string()
  .required('Required.')
  .trim();
```

[Demo](https://726wprxw3q.codesandbox.io/formik/subscribe)，[原始碼](https://codesandbox.io/s/726wprxw3q)。

### FastField

若只是使用一般的`<Field>`，則每次欄位更新時，其他欄位都會一同重新渲染；但若改成 `<FastField>` 則沒有相依關係的欄位就不會重新渲染。這是由於 `<FastField>` 內部實作 `shouldComponentUpdate()` 來決定是否要重新渲染的緣故。

下圖是 Formik 的 `<FastField>` 的流程圖，只有需要被更新的欄位（direct update）才會重新渲染，否則就 block 住。

![Workflow](https://cythilya.github.io/assets/react-form/formik-workflow-with-fastfield.png)

來看一個範例，如下圖所示，這是一個簡易的表單，包含兩個欄位 Name 和 Password，右邊灰色圓圈內的數字表示元件的渲染數，由上而下依序是 Name 欄位（FastField）、Password 欄位（FastField），一開始數字都是 1。

![Formik FastField Example](https://cythilya.github.io/assets/react-form/formik-example-3.png)

當對 Name 欄位輸入資料「Summer」時，由於只有 Name 欄位是會被直接更新的，因此旁邊只有 Name 欄位旁邊的灰色小圈圈數字會增加。

![Formik FastField Example](https://cythilya.github.io/assets/react-form/formik-example-2.png)

[Demo](https://726wprxw3q.codesandbox.io/formik/signup)，[原始碼](https://codesandbox.io/s/726wprxw3q)。

### 優點

- 有足夠的 API 能實作細緻的 UI，這樣開發者就不需要重新實作取得表單狀態的方法，而能減少撰寫的程式碼。
- 將狀態存在 Formik instance state，而非全域。
- 針對效能的改善，Formik 提供 FastField 來減少渲染數。
- 打包後的檔案很小，壓縮後只有 12.7KB。
- Formik 的表單和欄位都支援同步與非同步驗證，相較 Redux Form 來說方便許多，也有多一個選擇。
- Yup 能將驗證規則撰寫得清楚易懂。

### 缺點

雖然功能包山包海，功能齊全，但相較 Final Form 可用 decorator 來擴充功能，彈性較小。

## Summary

### 如何選擇好的表單函示庫？

再次回顧本文一開始提到的，要怎麼選擇好的表單函式庫呢？我們可以考慮以下幾點...

- 這個函式庫是怎麼做狀態管理的？會不會有效能問題？
- API 充足嗎？能為開發者減少撰寫多少程式碼？通常 API 充足的函式庫，打包後的檔案體機會比較大，只能取捨一下了。
- 如何撰寫驗證邏輯？
- 擴充性如何？是否能輕易新增和移除功能以符合未來的需求？
- 文件是否詳細？範例是否充足？之後若遇到問題是否能很快被解決？
- 打包後的檔案大小。
- Github 星星數 and NPM 下載數。這是比較不重要的考量點，畢竟過去火紅的專案，目前可能已不符合需求，但已長時間累積了很多的星星數和下載數；而新出來的專案可能因起步較晚而沒有很多的星星數和下載數，但做得卻非常好。
- 函式庫提供的特點是否能解決專案的需求？

...

以下一一說明。

### 狀態管理

- Redux Form：使用 Redux 儲存所有表單的狀態，並且對於多餘的渲染並無改善方法。
- React Final Form：存在表單實體的狀態中，並使用訂閱狀態的方式做效能優化。
- Formik：存在表單實體的狀態中，經由實作 `<Fastfield>` 元件內的 `shouldComponentUpdate()` 做效能優化。

### 程式碼撰寫

Redux Form、React Final Form 和 Formik 皆提供足夠的 API 以實作細緻的表單，因此節省開發人員不少時間。

### 驗證

- Redux Form
  - 無內建驗證工具，但可搭配其他驗證工具。
  - 非同步驗證只支援表單層級的驗證。
- React Final Form
  - 無內建驗證工具，目前也沒有好的可搭配的驗證工具，驗證部份要自己開發。
  - 表單和欄位都支援同步與非同步驗證。
- Formik
  - Yup 是很棒的驗證工具，寫起來清楚簡潔，好懂好維護。
  - 表單和欄位都支援同步與非同步驗證。

### Extensibility

- Redux Form 和 Formik：無擴充性。
- React Final Form：擴充性高，可利用 decorator 的方式輕易的新增或移除功能以符合未來的需求，也因此能保持打包後的體積很小。

### Examples and documents

- Redux Form：文件和範例不夠清楚詳細。
- React Final Form 和 Formik 的文件和範例都很充足詳細。

### Bundle size

![Bundle size](https://cythilya.github.io/assets/react-form/bundle-size.png)

- Redux Form：檔案體積大，壓縮後約 26.7KB。
- React Final Form + Final Form：壓縮後約 7.7KB。
- Formik：Formik 壓縮後是 12KB，Yup 是 21.6KB.

### Github stars and NPM downloads

![Github stars and NPM downloads](https://cythilya.github.io/assets/react-form/github-stars-npm-downloads.png)

- Redux Form：每週下載數約 **三十七萬** 次，累積星星數約 **一萬** 顆。
- React Final Form：每週下載數約 **六萬五** 次，累積星星數約 **三千九** 顆。
- Formik：每週下載數約 **三十二萬** 次，累積星星數約 **一萬四千** 顆。

## 總結

在經過以上探索與討論後，做個總結...

在目前我所經手的專案上，由於 (1) 對表單功能有強烈需求，必須顧慮開發上的便捷，並且 (2) 專案很老了，常常需要重構，要能有效整理程式碼，因此 Formik & Yup 對我而言就是很好的選擇；而也有些專案對體積大小斤斤計較、並且不需要這麼多功能，或必須維持高彈性、因應需求端的快速變化，那就很適合使用 React Final Form。

話說這麼多，就是希望大家在茫茫大海中，都能找到適合自己的表單函式庫摟！👍 👍 👍

## References

- [Comparison of form libraries in react](https://codebrahma.com/form-libraries-in-react/)
- [Erik Rasmussen — 🏁Final Form: Form state management via Observers](https://www.youtube.com/watch?v=fxEW4jgoX-4)
- [Final Form: The road to the checkered flag](https://codeburst.io/final-form-the-road-to-the-checkered-flag-cd9b75c25fe)
- [Formik vs Final Form](https://github.com/jaredpalmer/formik/issues/533)

關於作者：
[@cythilya](https://cythilya.github.io/) 前端工程師，喜歡交換明信片、設計簡單的小物、旅遊和看電影。
