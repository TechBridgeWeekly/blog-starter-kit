---
title: React Native in 24 Hours
date: 2016-11-10 22:29:57
tags: react, react_native, android
author: huli
---

{% img  /img/huli/rn/main.jpg 200 title %}

# 前言

上個禮拜的時候，我們公司舉辦了一年一度的黑客松，一隊有四個人。因為我才剛加入公司大概兩個禮拜而已，所以也沒認識什麼人。不過，剛好當初找我進來的同事問我要不要一起參加，就跟著報名了。

黑客松的時間是禮拜五早上十一點到禮拜六同一時間，一共 24 個小時。我的三個隊友，一個是 PM、一個是資安部門、一個是 SA（System Admin） 部門。因此，在他們知道我現在是前端工程師，以前是 Android 工程師以後，理所當然地，Mobile App 的部分就由我負責了。

這也是這篇標題的由來：React Native in 24 hours，想跟大家分享在 24 小時之內，可以用 React Native 做出什麼樣的作品，以及過程中碰到的困難及挑戰。

# Idea

先簡單談一下我們這組那時候想做的東西，是一個 Password generator and manager。

大家都知道，記憶密碼是一件很麻煩的事情，所以，很多人會在每個網站都用同一組密碼。而大家也知道，這樣的壞處就是，當你其中一個網站的密碼外洩以後，其他網站也會跟著遭殃，這是一件滿可怕的事情。

因此有很多人會用密碼管理服務，只要記住這個服務的密碼，其他密碼都靠這個網站幫你儲存。可是，我的隊友覺得這樣依然會有問題，那就是這個網站仍然有安全性的風險。有沒有可能完全不儲存密碼呢？

有！那就是不要用儲存的，而是用「產生」的，只要有一套邏輯負責產生密碼，保證輸入一樣，輸出也一樣，就可以利用這一套邏輯每一次都產生密碼。

舉個例子，假設這套邏輯是

```
password = sha_256(root_password + domain_name + 'I_AM_WEEK_SALT')
```

這樣你只要有你的主密碼（root_password）跟你要登入的網站，就能幫你產生出一組獨特的密碼。

這就是這個產品的核心概念。

不過，上面那套邏輯有個問題是：假如我某個網站密碼外洩了，我想換一個，怎麼換？
這時候又要引進一個新的參數叫做 changeID，是一個數字，把密碼產生的邏輯變成

```
password = sha_256(root_password + domain_name + changeID + 'I_AM_WEEK_SALT')
```

就可以任意更換無限次密碼。不過其實這樣，也頂多就是一個密碼生成器。
當要加上「管理」的功能時，就比較麻煩了，例如說我們想要幫使用者儲存他的帳號，就可以自動填入。

因此我們就要有後端的 DB 去記錄這些資料，並且要有登入機制，而且一旦引入登入機制，就違反了「不想儲存任何密碼」的這個初衷。
總之，在討論一些 feature 要不要做的時候，花了很多時間，討論了很久。

因為這些內容其實有點瑣碎，因此我就不多提了，如果你對這個概念有興趣，可以參考 [LessPass](https://lesspass.com/#/)。這跟我隊友想的 idea 有 87% 像，而且做得很完整，可以參考看看。

# 說好的 React Native 呢？

好了，大概介紹完產品之後，終於要進入到 React Native 的部分。因為我之前有寫過 React + Redux 的網站，也有 run 過一次 React Native 的簡單範例（Hello World），所以對 React Native 不算陌生，至少能 build 的起來！

就讓我們先從這個 App 的第一頁開始吧！

{% img  /img/huli/rn/main.jpg 200 title %}

這一頁很簡單，就是一張大張背景圖，上面放 logo 跟一些字，再加兩個按鈕跟一個可以點的「Join Now」。其實以前在寫 Android 的時候，最麻煩的不是程式碼，而是版面！直到現在，我還是覺得自己跟 Android 的 Layout 很不熟，沒辦法排出自己想要的版面。

但是現在我們有了 React Native，有了新的排版方式：flexbox。

如果你不知道什麼是 flexbox，我可以很簡單的介紹一下，基本上就是你可以選擇你要直的排還是橫的排、要靠上靠下對齊還是置中、空隙應該怎麼分配、每個版面佔多少比例等等。我自己覺得是個滿直覺的排版方式。

以上面那個版面為例，就可以用直的來排，然後按照比例分成四塊，按鈕那個區塊內部用橫的排，然後切對半：

{% img  /img/huli/rn/main-border.jpg 200 title %}

其實這就很像 React 用 component 來切的概念，切成很多很多細小的組件再合在一起。

我當初寫 code 的時候，都把[這個分頁](http://www.weibo.com/1712131295/CoRnElNkZ?ref=collection&type=comment#_rnd1478796364957)開在旁邊，是很淺顯易懂的圖片說明，可以很方便的就找到自己想要的排版方式應該怎麼用。

至於在按鈕的部分，我是用 [APSL/react-native-button](https://github.com/APSL/react-native-button) 這個第三方的套件，但詳細原因我也忘記了，可能是用原生的碰到什麼問題，所以去找了第三方的來用。

這邊附上最後寫出來的部分程式碼：

```javascript
  render() {
    return (
      <View style={styles.container}>
        <View style={styles.bgImageWrapper}>
          <Image source={require('../img/bg.png')} style={styles.bg} />
        </View>

        <Image source={require('../img/logo-white.png')} style={styles.image}/>
        <Text style={styles.text}>Only you have your password</Text>
        <View style={styles.btnGroup}>
          <Button style={styles.btnGuest} textStyle={{fontSize: 18, color: 'white'}} onPress={this.onGuestClick.bind(this)}>
            Use as guest
          </Button>

          <Button style={styles.btnSignIn} textStyle={{fontSize: 18, color: 'white'}} onPress={this.onSignInClick.bind(this)}>
            Sign in
          </Button>
        </View>
        <Text style={styles.textJoin} onPress={this.onJoinClick.bind(this)}>Join now</Text>
  
      </View>
    );
  }
```

這邊有兩點要提一下，第一點是 `textStyle` 的部分最好也先宣告成變數再使用，但因為是黑客松所以比較少時間去做這些調整，就比較不好的直接這樣寫了。第二點是`this.onGuestClick.bind(this)`這樣的方式其實不好，但不知道為什麼，我沒辦法在`constructor`裡面先`bind`，所以只好這樣寫了。

其實只要能把這個 `render` 的函式寫出來，這一頁就看起來有模有樣了，可是問題是，我們還有其他很多頁。要怎麼切換到別的畫面呢？

# Navigator

如果要在多個畫面之間切換，就要靠`Navigator`這個組件了。從[官方文件](https://facebook.github.io/react-native/docs/using-navigators.html)可以大致看出用法，或是可以參考[別人寫的教學](https://facebook.github.io/react-native/docs/using-navigators.html)。

直接附上程式碼再來解釋

```js
import React, { Component } from 'react';
import {
  Navigator
} from 'react-native';

import MainScene from './scenes/MainScene';

export default class Zeropass extends Component {
  render() {
    var defaultName = "MainScene";
    var defaultComp = MainScene;

    return (
      <Navigator
        initialRoute={{
          name: defaultName,
          component: defaultComp
        }}
        configureScene={(route) => {
          return Navigator.SceneConfigs.PushFromRight;
        }}
        renderScene={(route, navigator) => {
          var Component = route.component;
          return (
            <Component {...route.params} navigator={navigator} />
          );
        }}
      />
    )
  }
}
```

`initialRoute`就是一開始的時候要給什麼參數，可以想成是 `defaultState` 的感覺，`configureScene`是跟換場動畫有關係的，這邊直接用內建的`PushFromRight`，會有一個還不錯的從右邊推進來的效果。

`renderScene`則是精華所在，就是`Navigator`的`render`函式，說明應該要渲染出什麼東西。

這邊先取`route.component`，這個`component`對應到的就是我在`initialRoute`這邊給的`component`這個 key，之後如果要換頁的話也要用這個 key 帶東西進來。

並且傳入`navigator`讓組件可以呼叫，以及加上`...route.params`，就可以帶額外的參數進來。

這只是最基礎的架構而已，但實際上如果要換頁，應該要怎麼寫呢？

```js
  toNext() {
    const { navigator } = this.props;
    if(navigator) {
      navigator.push({
        name: 'GuestLoginScene',
        component: GuestLoginScene,
      })
    }
  }
```

因為在`renderScene`的時候，我們有把`navigator`傳進去，所以在每一個 component 都可以用 `this.props` 取出來，想要換頁的話就只要 `push` 一個物件進去就好了。物件的格式自己決定好就行了。所以你會發現這邊的格式跟剛剛`initialRoute`傳進去的格式一模一樣。

好了，所以第一頁完成了、換頁的問題也解決了。剩下的就是把其它頁面刻出來，好像就差不多了。前途真是一片光明璀璨，看來 24 小時太多了。

## 等等，可是我們還有 Tab 啊

{% img  /img/huli/rn/pwd.jpg 200 title %}

當初在做這個頁面的時候，原本的設計是 DrawerLayout，就是 Android 風格的從左邊滑出來的列表。可是因為在做這個 App 時想要跨平台通吃，所以 Drawer 方案可見是行不通的。況且，我看了一下官方文件，實作上感覺也沒那麼簡單。

因此，最後找了第三方套件的[react-native-tab-view](https://github.com/react-native-community/react-native-tab-view)來用。因為無論在 iOS 或是在 Android，都可以看到 Tab 的出現，所以會比 Drawer 更好一些。

這個 Tabview 的用法相當簡單，客製化程度滿高的，只要自己實作幾個函式就好。

```js
const icons = {
  account: require('../img/icon_account.png'),
  edit: require('../img/icon_edit.png'),
  setting: require('../img/icon_setting.png'),
  help: require('../img/icon_help.png'),
}

export default class TabViewExample extends Component {

  constructor(props) {
    super(props);

  }

  state = {
    index: 0,
    routes: [
      { key: 'account', title: 'Account', icon: 'account'},
      { key: 'new', title: 'Create', icon: 'edit' },
      { key: 'setting', title: 'Setting', icon: 'setting' },
      { key: 'help', title: 'Help', icon: 'help' },
    ],
  };

  _handleChangeTab = (index) => {
    this.setState({ index });
  };

  _renderIcon = ({ route }) => {
    return (
      <Image
        source={icons[route.icon]}
        style={styles.icon}
        color='white'
      />
    );
  };

  _renderHeader = (props) => {
    return (
      <TabBar 
        renderIcon={this._renderIcon}
        tabStyle={styles.tab} 
        labelStyle={styles.label}
        {...props} />
    );
  };

  _renderScene = ({ route }) => {
    console.log(route.key);
    switch (route.key) {

      case 'account':
        return <AccountTab />;
      case 'new':
        return <CreateTab />;
      case 'help':
        return <HelpTab />;
      case 'setting':
        return <SettingTab />;
      default:
        return null;
    }
  };

  render() {
    return (
        <TabViewAnimated
          style={styles.container}
          navigationState={this.state}
          renderScene={this._renderScene}
          renderFooter={this._renderHeader}
          onRequestChangeTab={this._handleChangeTab}
        />
    );
  }
}
```

跟`Navigator`其實有異曲同工之妙，都是靠`renderScene`這個函式去決定如何渲染出畫面。在這邊就很簡單的根據目前所選到的 key 去渲染出相對應的組件即可。

可是，假如 Tab 的頁面也是巢狀的呢？例如說我第一個 Tab 可能是輸入密碼的畫面，輸入完按下確定之後話跳到下一個畫面，這個要怎麼做呢？

我自己猜應該是也可以用`navigator`來做，在`renderScene`的時候渲染出`navigator`，其他的就跟我們剛開頭介紹的差不多。

意思就是，每一個 Tab 其實都像是一個小的 App，有自己的`navigator`來管理自己的狀態。

但是在黑客松的時候，我一時半刻沒有想到這樣的解法，於是就手刻了一個最直覺、最暴力的。

```js
import SiteScene from './SiteScene';
import PasswordScene from './PasswordScene';

export default class CreateTab extends Component {

  constructor(props) {
    super(props);
    
    // 2 page
    this.state = {
      page: 1,
      site: ''
    }
  }

  goBack() {
    this.setState({
      page: 1
    })
  }

  toNext(site) {
    this.setState({
      ...this.state,
      page: 2,
      site
    })
  }

  render() {

    const { page, site } = this.state;

    return (
        <View style={styles.container}>
          <View style={styles.top}>
            <Text style={styles.back} onPress={this.goBack.bind(this)}>
              { page==2 ? ' ← ' : ' ' }
            </Text>
          </View>
          {page==1 && <SiteScene toNext={this.toNext.bind(this)}/>}
          {page==2 && <PasswordScene site={site}/>}
        </View>
    );
  }
};
```

在 tab 裡面用 state 來管理目前的 index，因為只有兩個頁面所以還滿好做的，根據 index 渲染出相對應的頁面即可。然後還可以用一個簡單的 navbar 包起來，就可以點上面的箭頭回到上一頁。

{% img  /img/huli/rn/website.jpg 200 title %}

{% img  /img/huli/rn/pwd.jpg 200 title %}

就這樣，tab 的問題也解決了。看起來一切都 work 的不錯，可以自由自在在各個頁面之間切換自如了。
接下來，好像就只剩下串 API 了。

# API

因為有支援 async/await 語法，所以寫起來十分清爽。下面這一段程式碼是要去 server 抓取使用者儲存過資訊的 domain 回來。
為了方便起見，我把所有的 API call 都包在`API`這個檔案裡面，用`fetch`去拿資料。

``` js
const API_URL = {
    'getInfo': 'http://api.com/api/v1/users',
}

const API = {

  getInfo: async function(uid) {

    let response = await fetch(`${API_URL.getInfo}/${uid}`);
    let json = await response.json();

    return json;
  }
}

export default API;
```

```js
async componentDidMount() {

  let domains = [];
  this.setState({
    spinnerShow: true
  });

  try {
    const response = await API.getInfo(store.getId());
    domains = response.domains;
  } catch(err) {
    console.log(err);
  }

  this.setState({
    dataSource: ds.cloneWithRows(domains),
    spinnerShow: false
  })
}
```

（不過我後來想一下，這樣子把`componentDidMount`直接包成 async 好像不太好，應該獨立成一個函式去呼叫）

# 體驗跨平台的威力

因為我自己是用 Android 的，所以我在開發的時候都是直接用 Android 實機測試。不得不提已經被講到爛掉的，React Native 的 hot reload，用起來真的很爽快，只要儲存檔案之後就可以在 App 上看到新的畫面。（雖然原生的 Android 現在也支援了，但我還沒有機會體驗到就是了）

其實原本我的隊友只有預期我開發出 Android 的版本（他們都用 iPhone），但殊不知 React Native 太過強大，當我把 App 開發到九成的時候，想說來試試看 build 到 iOS 上好了，發現跟我想像中的不一樣。

我以為要 build 的時候應該會碰到很多錯誤然後要慢慢修，或者是沒有 Apple 的開發者帳號就沒辦法 build 到手機上之類的。

但我完完全全錯了。React Native 就是那麼簡單，超級輕鬆就可以有一個 iOS 的版本。也根本不用什麼 Apple 開發者帳號，就可以安裝到手機上面測試（只是要開一些安全性設定就是了）。

在 build 的時候是有碰到一點小問題，但拜過 Google 大神之後基本上就解決了。

（不過會那麼輕鬆，也是因為我基本上沒用到 Native 的功能，排版也是兩個平台都長一模一樣，所以可以共用 100% 的程式碼。）

除了 iOS 很輕鬆以外，Android 如果要產生可以上 Google play 的安裝包也很容易，官方教學寫的超級清楚，就參數填一填以後打個指令，剩下的全部都幫你做好。

# 結論

這篇文章主要是想分享一下當初碰到的困難以及開發的歷程，花最久時間的是排版（因為對 flexbox 沒那麼熟，所以不知道排出來會是怎樣），還有 Tab 那一段也花了一點時間研究作法。但總體來說，我覺得 React Native 的開發效率是很高的。

因為我中間有睡覺，所以實際上 coding 應該 18 個小時左右，就可以做出一個可以動、有接 API 又跨平台的 App。意思就是說如果你的 App 沒有很複雜的話，基本上是可以在兩三天之內就做出一個還不錯的 prototype。

只要掌握幾個元素：`View`, `Button`, `Image`, `Navigator`, `ListView`, `TextInput`，差不多就可以完成 80% 的工作了。

我最喜歡的還是它的排版方式，對前端工程師來說比較熟悉。開發環境的設置我覺得也比 Native 的簡單許多，而且可以用 JavaScript 我覺得也是一大好處。

之前看了那麼多 React Native 的介紹文，我覺得最快速能認識的方式還是自己跳下去寫 code。如果大家假日有空的話，也可以試試看來場自己一個人的黑客松，挑戰在 24 小時以內用 React Native 做出一個簡單的 App，相信一定會很有收穫的。

最後，附上這個 App 的 Github：[zeropass-react-native](https://github.com/aszx87410/zeropass-react-native)
如果有興趣的話可以參考看看
（因為時間緊迫，所以很多東西都是只求能動就好，很多都是錯誤示範，真的只是「僅供參考」）

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好