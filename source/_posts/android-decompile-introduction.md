---
title: 人人都會的 Android Apk 反編譯
date: 2016-03-24 23:30:00
tags: 
    - Android
    - Decompile
    - Apktool
    - Smali
    - Dex2jar
    - Mobile
    - 行動網路
    - 反編譯
    - APK
author: huli
---

對於 Android 工程師來說，了解如何反編譯可以增進自己對 Android 底層的理解，也可以思考如何保護自己的 apk 不被反編譯。

對於一般人來說，許多現成的工具可以幫助我們非常輕鬆的、只要打打幾個指令就可以反編譯 apk，看到 java source code，滿足自己的好奇心。

本篇文章只介紹一些工具的使用，適合初學者觀看。若是想了解更底層的知識，可以參考文末附上的延伸閱讀。

## 事前準備
首先，我們需要一個用來被破解的 apk，簡單用任何你平常熟悉的工具自己 build 一個就好了。基本架構很簡單，只要一個 `MainActivity` 跟兩個`TextView`就好：

``` java MainActivity.java
public class MainActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        TextView text = (TextView)findViewById(R.id.text);
        text.setText("Taiwan No1");
    }
}
```

``` xml activity_main.xml
<LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
    android:layout_width="match_parent"
    android:orientation="vertical"
    android:layout_height="match_parent">

    <TextView
        android:text="@string/hello_world"
        android:layout_width="wrap_content"
        android:layout_height="wrap_content" />

    <TextView
        android:id="@+id/text"
        android:layout_width="wrap_content"
        android:layout_height="wrap_content" />

</LinearLayout>
```

安裝到手機上之後，會看到這樣的畫面：

![device-2016-03-20-152510.png](http://user-image.logdown.io/user/7013/blog/6977/post/661513/uVN5gnA0QC6PEdujMIZg_device-2016-03-20-152510.png)

## 實際動手
好，這個就是我們要拿來測試的 apk 了！
接著你需要一些非常好用的工具：

1. [apktool](http://ibotpeaches.github.io/Apktool/)
2. [jd-gui](http://jd.benow.ca/)
3. [dex2jar](https://sourceforge.net/projects/dex2jar/)

如何安裝就不再贅述了，讀者們可以參考看看文件或是上網搜尋一下就會有一堆解答～
`apktool`是拿來把 apk 拆開用的，可以反編譯 apk 之後，看到 `smali` 檔案跟 `resource`
`dex2jar`可以把 apk 轉成 jar，再用`jd-gui`檢視 java code

接著我們開啟 terminal，到剛剛那個示範 apk 的目錄底下，執行`apktool d APKNAME.apk`
![螢幕快照 2016-03-20 下午3.32.47.png](http://user-image.logdown.io/user/7013/blog/6977/post/661513/Ahk6ZkFOQm7ECISEyxMI_%E8%9E%A2%E5%B9%95%E5%BF%AB%E7%85%A7%202016-03-20%20%E4%B8%8B%E5%8D%883.32.47.png)

執行以後，會自動生成一個`APKNAME`的資料夾，裡面就是反編譯出來的東西了。

```
.
├── AndroidManifest.xml
├── apktool.yml
├── original
├── res
└── smali
```

其中比較值得講的是`smali`這個資料夾，其實這裡面就是你的 source code，只是格式不太一樣。
你可以在`smali`這資料夾裡面找到你的`MainActivity.java`，內容如下：
（覺得長得很奇怪是很正常的事，但是認真多看幾眼，你會發現其實沒那麼難懂）

``` java MainActivity.java
.class public Lapktest/huli/com/apkdecompile/MainActivity;
.super Landroid/app/Activity;
.source "MainActivity.java"

# direct methods
.method public constructor <init>()V
    .locals 0

    .prologue
    .line 8
    invoke-direct {p0}, Landroid/app/Activity;-><init>()V

    return-void
.end method

# virtual methods
.method protected onCreate(Landroid/os/Bundle;)V
    .locals 2
    .param p1, "savedInstanceState"    # Landroid/os/Bundle;

    .prologue
    .line 12
    invoke-super {p0, p1}, Landroid/app/Activity;->onCreate(Landroid/os/Bundle;)V

    .line 13
    const v1, 0x7f040019

    invoke-virtual {p0, v1}, Lapktest/huli/com/apkdecompile/MainActivity;->setContentView(I)V

    .line 14
    const v1, 0x7f0c0050

    invoke-virtual {p0, v1}, Lapktest/huli/com/apkdecompile/MainActivity;->findViewById(I)Landroid/view/View;

    move-result-object v0

    check-cast v0, Landroid/widget/TextView;

    .line 15
    .local v0, "text":Landroid/widget/TextView;
    const-string v1, "Taiwan No1"

    invoke-virtual {v0, v1}, Landroid/widget/TextView;->setText(Ljava/lang/CharSequence;)V

    .line 16
    return-void
.end method
```

你可以仔細對照一下剛剛自己寫的 java code，會發現只是換了種格式而已：

``` java
setContentView(R.layout.activity_main);
```
其實就等於

``` java
.line 13
const v1, 0x7f040019
invoke-virtual {p0, v1}, Lapktest/huli/com/apkdecompile/MainActivity;->setContentView(I)V
```

你可能會好奇，這個`0x7f040019`是哪來的？
事實上，你可以在`res/values/public.xml`這個檔案裡面找到答案：

``` xml
<public type="layout" name="activity_main" id="0x7f040019" />
```

到這裡，應該就可以大概猜出 Android 在編譯時候的流程：

1. 把所有資源檔壓縮、處理並且包在一起，產生`id與記憶體位置對照表`
2. 把程式碼裡面所有的`R.xx.xxx`透過剛剛產生的表，換成實際的記憶體位置
3. 把 java code 變成 smali code（有點像把 C 變成組合語言的程式碼那樣）

## 修改
在剛剛的`smali`裡面，有這麼一段：

``` java
.line 15
.local v0, "text":Landroid/widget/TextView;
const-string v1, "Taiwan No1"

invoke-virtual {v0, v1}, Landroid/widget/TextView;->setText(Ljava/lang/CharSequence;)V
```

讓我們把`Taiwan No1`換成`T@iw@n n0!`。
還記得另一個`TextView`有用到`R.string.hello_world`嗎？
在`res/values/strings.xml`裡面，可以找到這一串的定義：

``` xml
<string name="hello_world">Hello world!</string>
```

改成：

``` xml
<string name="hello_world">HELLO WORLD</string>
```

確定都有改完以後，就可以把這些程式碼再度「組裝」回去。
還記得剛剛反編譯的指令嗎？`apktool d APK_NAME.apk`
這邊的`d`就是`decompile`的意思，所以如果要逆向組裝回去，就是`b`，`build`。

`apktool b APK_NAME`

執行完之後可以在`APK_NAME/dist`下面找到一個 apk。
但要注意的是這個 apk 還沒有被 sign 過，因此無法安裝。
可以隨便生成一個 keystore 或是找現成的來簽署。
`jarsigner -verbose -digestalg SHA1 -keystore ~/KEY.keystore APK_NAME.apk KEY_ALIAS`

安裝完以後就會看到這樣的畫面：

![device-2016-03-20-160501.png](http://user-image.logdown.io/user/7013/blog/6977/post/661513/RNKaPElHQA2BJ02proFr_device-2016-03-20-160501.png)

沒錯！就是這麼簡單，一個 apk 就這樣被修改了！

可是`smali`的程式碼不好懂，能不能直接看到 java code呢？
這時候剛剛推薦的工具`dex2jar`與`jd-gui`就派上用場了
前者可以把 apk 變成 jar，後者可以開啟一個 jar 並且顯示 java code
兩個組合在一起，就可以直接看到原本的程式碼了

`dex2jar`下載下來之後會有一堆的 shell script，`dex2jar`就是我們想要的那個
`./d2j-dex2jar.sh app.apk`
執行完之後會有一個 jar，用 jd-gui 打開，會看到你的程式碼一覽無遺：

![螢幕快照 2016-03-20 下午4.10.15.png](http://user-image.logdown.io/user/7013/blog/6977/post/661513/zrnTKCQgT0OeIPbkkfp8_%E8%9E%A2%E5%B9%95%E5%BF%AB%E7%85%A7%202016-03-20%20%E4%B8%8B%E5%8D%884.10.15.png)

## 總結
沒接觸過反編譯的人可能會很驚訝：什麼！要改掉一個 apk 居然這麼簡單！
沒錯，就是這麼簡單，而且這只是一個很基本的範例而已。事實上，你想要加入新的程式碼、加入新的資源（圖片、聲音等等）也是可以的。
也就是說，你不只可以修改，還可以擴充原本的 apk！

但也有些方法可以防止不肖人士反編譯 apk，例如說加殼、混淆、動態載入等，關於這些方法我們下次有機會再介紹給大家囉！

## 延伸閱讀
1. [Android 反編譯與防止被反編譯](https://magiclen.org/android-decompiler/)
2. [[Android] 程式碼混淆(ProGuard)與反組譯](http://aiur3908.blogspot.tw/2015/07/android-proguard.html)
3. [[Android] 反組譯 破解Android的apk安裝檔](http://blog.davidou.org/archives/553)
4. [反编译的常用工具与使用方法](http://www.wangchenlong.org/2016/03/19/reverse-analyze-apk/)
5. [Smali--Dalvik虚拟机指令语言-->【android_smali语法学习一】](http://blog.csdn.net/wdaming1986/article/details/8299996)
6. [android反编译-smali语法](http://blog.isming.me/2015/01/14/android-decompile-smali/)


關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好