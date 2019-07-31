---
title: 快速取得 apk 相關資訊
date: 2016-05-20 00:20:08
tags:
   - Android
   - aapt
   - keytool
   - hashkey
author: huli
---

## 前言
在 [之前](http://blog.techbridge.cc/2016/03/24/android-decompile-introduction/)，我們曾經介紹過如何反編譯 Android Apk。而藉由反編譯，我們可以知道許多跟 apk 有關的資訊，例如說 `AndroidManifest.xml`，只要這個檔案就可以看出 apk 的一些基本資訊，還可以看到整個 apk 的程式碼跟使用到的資源（圖檔、影片、聲音等等）。

但若是今天我們只想要知道基本資訊，完全不想知道 apk 是怎麼寫的，也根本不關心它用到哪些資源，那該怎麼辦呢？反編譯需要一點時間，愈大的 apk 需要愈久，有沒有更好的方法呢？

## 需要哪些資訊？
首先我們先來定義一下「基本資訊」指的是哪些。以我來說，我想取得的基本資訊有以下六點：

1. package name
2. version code
3. version name
4. launch activity
5. Google SHA1 Fingerprint
6. Facebook Keyhash

前四個的用途在於，假設你今天做了一個公司內部的 apk 上架系統，如果有了前四項資訊，就可以做跟 Google Play 相似的驗證，例如說驗證 package name 跟上次上傳的是否一樣、版本號是否比上一次的高等等。

至於後兩個呢？有串接過 Google 跟 Facebook 登入的讀者就會知道，這兩個是串登入時必備的東西。你要在設定裡面新增這兩組 key，才能夠使用登入功能，否則會出現驗證錯誤之類的字眼。

知道我們需要哪些以後，就來動手做吧！

## 好用的 keytool
`keytool` 是系統內建，與認證相關的指令。  
我們可以用 `keytool -list -printcert -jarfile NAME.apk` 提取出一些資訊：

```
簽署者 #1:

簽章:

擁有者: CN=Android Debug, O=Android, C=US
發出者: CN=Android Debug, O=Android, C=US
序號: 4b52355e
有效期自: Sun Jan 17 05:53:34 CST 2010 到: Mon Jan 17 05:53:34 CST 2011
憑證指紋:
   MD5:  14:99:01:12:7A:69:CD:75:4F:31:75:8C:59:F6:71:63
   SHA1: 24:69:FD:17:6B:C3:43:FC:3A:85:EC:4B:C5:D7:9F:09:4A:71:60:80
   SHA256: 57:EB:73:81:D7:08:E6:45:FE:26:99:FB:3C:1F:37:1E:EE:38:39:20:E0:2D:C6:76:0E:84:2B:DD:1C:5C:C9:70
   簽章演算法名稱: SHA1withRSA
   版本: 3
```

以這個 apk 來說，列出了：擁有者、發出者、有效期限、憑證指紋等等的資訊，而其中的 `SHA1` 就是 Google 登入會用到的資訊。

那 Facebook Keyhash 呢？從 [官方文件](https://developers.facebook.com/docs/android/getting-started#release-key-hash) 可以知道，其實就只是把 sha1 先變成 binary 然後再做 base64 而已。

只要有了 sha1，搭配一些指令，就可以很簡單的生成 Facebook Keyhash。

## 萬能的 aapt
aapt 的全名是：Android Asset Packaging Tool，超級好用！  
可以先來看看 aapt 到底可以做哪些事情
由於我們需要的是取出資訊，因此直接看 dump 的部份：

```
 aapt d[ump] [--values] WHAT file.{apk} [asset [asset ...]]
   badging          Print the label and icon for the app declared in APK.
   permissions      Print the permissions from the APK.
   resources        Print the resource table from the APK.
   configurations   Print the configurations in the APK.
   xmltree          Print the compiled xmls in the given assets.
   xmlstrings       Print the strings of the given compiled xml assets.
```

有興趣的讀者可以每一個都試試看，看會出現什麼結果。以我們的需求來講，badging 是最符合的

`aapt dump badging NAME.apk`

```
package: name='com.gmail.aszx87410.movie_to_nine' versionCode='1' versionName='1.0'
sdkVersion:'8'
targetSdkVersion:'16'
uses-permission:'android.permission.INTERNET'
uses-gl-es:'0x20000'
uses-feature-not-required:'android.hardware.telephony'
uses-feature:'android.hardware.screen.portrait'
uses-feature-not-required:'android.hardware.screen.landscape'
application-label:'今晚九點電影2.0'
application-label-he:'今晚九點電影2.0'
application-label-es:'今晚九點電影2.0'
application-label-iw:'今晚九點電影2.0'
application-icon-120:'res/drawable-ldpi/icon.png'
application-icon-160:'res/drawable-mdpi/icon.png'
application-icon-240:'res/drawable-hdpi/icon.png'
application-icon-320:'res/drawable-xhdpi/icon.png'
application-icon-480:'res/drawable-xxhdpi/icon.png'
application: label='今晚九點電影2.0' icon='res/drawable-mdpi/icon.png'
launchable-activity: name='com.ansca.corona.CoronaActivity'  label='今晚九點電影2.0' icon=''
uses-feature:'android.hardware.touchscreen'
uses-implied-feature:'android.hardware.touchscreen','assumed you require a touch screen unless explicitly made optional'
main
other-activities
other-receivers
other-services
supports-screens: 'small' 'normal' 'large' 'xlarge'
supports-any-density: 'true'
locales: '--_--' 'he' 'es' 'iw'
densities: '120' '160' '240' '320' '480'
native-code: '' 'armeabi-v7a'
```

將將將將～  
我們所需要的資訊全部出現在這裡了，還附帶權限列表、app logo、app 名稱等等的資訊
做到這邊，全部需要的東西都有了，剩下就只是切出字串與整合而已

## 總結
今天這篇文章簡單介紹了 `keytool` 跟 `aapt` 的使用，主要是想要不靠 `apktool`，利用其他工具取出我們想要的資訊，既省時又省力。

如果你有興趣知道最後做出來的成品長怎樣，[apkinfo.sh](https://github.com/aszx87410/apkinfo.sh) 是我放在 github 上面的一個小專案，用途就跟這篇文章所教的一樣，就是取出 apk 的相關資訊。


關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好