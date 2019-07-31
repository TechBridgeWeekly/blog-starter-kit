---
title: 與 DDoS 奮戰：nginx, iptables 與 fail2ban
date: 2016-08-12 23:20:38
tags:
  - DDoS
  - nginx
  - iptable
  - fail2ban
author: huli
---

## 前言
最近發生主機被大量 request 攻擊的事件，而且慘的是這台主機放的是論壇服務  
假設攻擊的點是論壇首頁，每次 request 都會去 query DB，而且有一堆 join  
其中還有些是 POST 的指令會讓 db update  
就這樣一直瘋狂又 select 又 update 導致 DB lock，cpu 飆高掛掉  

如果論壇是自己寫的，還可以在 DB 跟 application 之間加上 redis 之類的快取  
但偏偏這個論壇系統是別人的，沒有辦法動  

先簡單講一下架構，為了分散流量，前面有一台 AWS ELB 做 load balancing，後面有兩台機器，所有 request 都會先到 ELB，再自動到後面兩台其中一台。

## WAF
被攻擊之後怎麼辦呢？第一個想到的就是從 aws 提供的服務：WAF 來擋  
https://aws.amazon.com/tw/waf/

可是發現 WAF 跟原本想的不一樣，他沒有辦法設定像是：「擋掉 10 秒鐘內發超過 100 個 request 的 IP」這種規則

## nginx
只能繼續在網路上找解法，找到從 nginx 來擋的解法：  

[nginx防止DDOS攻击配置](https://www.52os.net/articles/nginx-anti-ddos-setting.html)
[通过Nginx和Nginx Plus阻止DDoS攻击](http://www.infoq.com/cn/news/2016/01/Nginx-AntiDDoS)
[Module ngx_http_limit_req_module](http://nginx.org/en/docs/http/ngx_http_limit_req_module.html)
```
http {

  //觸發條件，限制 ip 每秒 10 個 request
  limit_req_zone $binary_remote_addr zone=one:10m rate=10r/s; 
  
  server {
    location  ~ \.php$ {

      //執行的動作
      limit_req zone=one burst=5 nodelay;   
    }
  }
}
```

總之就是利用 `limit_req_zone`，這個 nginx 提供的東西  
宣告一個 10mb 的 zone 叫做 `one` 來儲存狀態，這裡的 `10r/s` 指的就是一秒 10 個 request  

接著在你想擋的地方加上：`limit_req zone=one burst=5 nodelay;`，就可以擋掉了  
nginx 會把處理 request 的數量調整成「最多 1 秒 10 個」，如果那個 ip 在同一時間有超過 5 個 request 還沒處理的話，就會傳回 `503 service temporarily unavailable`，這邊的 5 就是 `burst` 設定的值  
傳回去的 status code 也可以自己指定，例如說：`limit_req_status 505;`

儘管這個解法看起來很棒，但不知道為什麼，加了之後好像沒有用似的  
伺服器的警報還是一直在響，DB 還是持續飆高  

## iptables
在請教過其他同事之後，得知 iptabls 也可以擋，而且還是直接從 tcp 層擋
找到下面兩篇資料：

[淺談DDoS攻擊防護](http://blog.eztable.com/2011/05/17/how-to-prevent-ddos/)  
```
-A INPUT -p tcp –dports 80 -j WEB_SRV_DOS
-A WEB_SRV_DOS -p tcp –syn –dports 80 -m recent –rcheck –second 30 –hitcount 200 -j LOG –log-prefix "[Possible DOS Attack]"
-A WEB_SRV_DOS -p tcp –syn –dports 80 -m recent –rcheck –second 30 –hitcount 200 -j REJECT  
-A WEB_SRV_DOS -p tcp –syn –dports 80 -m recent –set  
-A WEB_SRV_DOS -p tcp –dports 80 -j ACCEPT
```

[運用 iptables 限制同一IP單位時間連線數](http://ishm.idv.tw/?p=188)  
```
-A INPUT -p tcp --dport 80 -m recent --rcheck --seconds 1 --hitcount 5 --name HTTP_LOG --rsource -j DROP
-A INPUT -p tcp --dport 80 -m recent --set --name HTTP_LOG --rsource
-A INPUT -p tcp --dport 80 -j ACCEPT
```

兩個的原理都一樣，是透過`-m recent –rcheck –second 30 –hitcount 200` 這段敘述，描述說你要擋住幾秒內發送幾次的 request，把這個連線 reject 或是 drop。  

直接從 iptables 去擋聽起來是個更好的解法，這樣 request 連 nginx 都不會進去就被擋住了，可是天不從人願，用了之後發現還是不行！怎麼會這樣呢！

## fail2ban
心灰意冷之下，同事又推薦一個好東西叫做：fail2ban，查了一下之後發現用法非常簡單，而且原理很好懂，決定用別台機器來測測看，測試成功之後再套用到正式環境的機器。

[用 Fail2Ban 防範暴力破解 (SSH、vsftp、dovecot、sendmail)](http://www.vixual.net/blog/archives/252)  
[fail2ban教學](http://blog.vic.mh4u.org/2011/272)  
[Ubuntu 中使用 fail2ban 針對大量 access 做判斷及阻擋](http://chuhw.pixnet.net/blog/post/167657289-ubuntu-%E4%B8%AD%E4%BD%BF%E7%94%A8-fail2ban-%E9%87%9D%E5%B0%8D%E5%A4%A7%E9%87%8F-access-%E5%81%9A%E5%88%A4%E6%96%B7%E5%8F%8A)  

綜合其中幾篇的敘述，可以得出以下流程  

1.修改`vim /etc/fail2ban/jail.local`  
2.寫入  

```
[http-get-dos]
enabled = true
port = http
filter = http-get-dos
logpath = /var/log/nginx/access.log # 要判斷的log
maxretry = 100 # 最多幾次
findtime = 5 # 時間區間
bantime = 600 # 要 ban 多久
action = iptables[name=HTTP, port=http, protocol=tcp]
```

上面的規則就是：5 秒內嘗試 100 次失敗之後就 ban 600 秒  

3.新增`/etc/fail2ban/filter.d/http-get-dos.conf`  
這邊的檔名就是對應到剛剛 `jail.local` 設定的名稱  

```
[Definition]
failregex = ^<HOST>- - .*\"(GET|POST).*
ignoreregex =
```

這裡的`failregex`要根據你的 log 去寫，像是 nginx 的 access log 長這樣：  

```
106.184.3.122 - - [21/Jul/2016:11:38:29 +0000] "GET / HTTP/1.1" 200 396 "-" "Go-http-client/1.1"
```

你就寫一個可以抓到`<HOST>`，也就是 ip 的 regular expression  

都設定好之後，重開一下應該就有效了，就會發現一直發 request 之後自己順利的被 ban 掉  
可以用 `iptables --list` 看一下自己是不是真的有被 ban  

fail2ban 的原理應該就是去看你指定的 log 檔跟規則，用這個檔案去判斷是不是超出設定的規則，超出的話就把 ip 抓出來，加規則到 iptables 裡面去擋掉，時間到了之後再把規則移除掉  

## 根據伺服器架構調整
試到這邊，終於成功了！可是既然原理也是 iptables，為什麼剛剛不行呢？  
還記得一開始我提過伺服器架構嗎？前面一台 ELB，後面兩台 web server  
因為 ELB 是 AWS 提供的服務，所以客製化程度很低，甚至連 ssh 進去都不行  
因此上面嘗試的方案都是個別套用到那兩台 web server  

這時候問題就來了：  
> 咦？那這樣子 web server 的 request 的來源，不就都是 ELB 的 ip 了嗎？

沒錯，你突破盲點了！之前沒有用就是卡在這邊  
你用 iptables 來檔，因為來源都是 ELB 的 ip，所以會擋掉的只有 ELB，而不是真正的攻擊來源，所以會造成 ELB 被擋掉，整個服務就因為一個攻擊者被搞的超級慢

所以在這個網路環境底下，iptables 是行不通的！  
那 nginx 呢？還記得我們的規則嗎？  

```
limit_req_zone $binary_remote_addr zone=one:10m rate=10r/s; 
```

`$binary_remote_addr`抓到的也都會是 ELB 的 ip  

這時候靈機一動突然想到，那能不能根據`X-Forwarded-For`這個 header 來設定呢？就會是真的 IP 了  
找到這一篇：[nginx rate limiting with X-Forwarded-For header](http://serverfault.com/questions/487463/nginx-rate-limiting-with-x-forwarded-for-header)

把`$binary_remote_addr`換成`$http_x_forwarded_for`

搞定！大功告成  
經歷一番千辛萬苦，最後終於在 nginx 把攻擊流量擋掉  
用 [JMeter](http://jmeter.apache.org/) 測試之後也發現確實成功了，多的 request 會直接回傳 503，最後成功解決主機被攻擊的問題。

關於作者： 
[@huli](http://huli.logdown.com/) 野生工程師，相信分享與交流能讓世界變得更美好