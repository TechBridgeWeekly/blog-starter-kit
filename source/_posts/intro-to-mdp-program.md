---
title: Markov Decision Process 的程式範例
date: 2018-12-22 23:01:50
tags:
    - Human-Robot Interaction
    - Markov Decision Process
author: pojenlai
---

## 前言

之前跟大家介紹過 [Markov Decision Process（MDP） 的原理跟數學推導](https://blog.techbridge.cc/2018/10/27/intro-to-mdp-and-app/) 還有 [在 Human-Robot Interaction 上的應用](https://blog.techbridge.cc/2018/11/24/intro-to-mdp-on-hri/)，今天我們就帶大家透過一個程式來體驗實作上的細節。有興趣的讀者可以透過這個例子將這個程式應用到機器人上！

## 快速複習 MDP 的基本概念

在進入問題之前，我們先簡單複習一下 MDP，首先是 MDP 的定義：

![img](https://i.imgur.com/1Q9VFLe.jpg)

然後，假設已經知道在每個 state 會獲得多少 reward，我們就可以用 value iteration 的演算法算出在各 state 該採取哪個 action 可以獲得最高的 reward（也就是獲得 optimal policy）：

![img](https://i.imgur.com/MfyBayV.jpg)

如果覺得看不太懂也沒關係，可以回去看 [Markov Decision Process（MDP） 的原理跟數學推導](https://blog.techbridge.cc/2018/10/27/intro-to-mdp-and-app/)。

## 程式場景說明

今天我們要討論的程式，是假設我們要實作如下場景：

今天你有一隻機器人要幫你清理桌子，桌上有一個寶特瓶（bottle）跟一個玻璃杯（glass），機器人應該要把寶特瓶跟玻璃杯都拿起來，但你還不太確定這隻機器人能不能順利地完成清理的任務。

隨著你對這隻機器人的觀察，你可能會越來越相信或越來越不信他會把事情做好，而當你不信這隻機器人的時候，你就會想要出手干涉他的行為。

## 跟 MDP 的符號相接

首先，我們要先知道我們必須定義清楚 state、action、transition matrix 跟 reward。因為這是所有 MDP 問題必備的元素。

再來，因為我們現在的問題牽涉到人類相不相信機器人，所以 state 會從單純只有 world state（bottle、glass 的狀態）延伸到還有 human state。而且，action 也有分 human 跟 robot 的 action。

我們把 state 跟 action 定義清楚：

1.  人類的 state 可以分成相信或不相信

    ![img](https://i.imgur.com/fKLMXgd.jpg)

2. 世界中的 state 可以根據 bottle 和 glass 在桌上、機器人手上、人手上來區分（兩種物體都各有三種可能，所以共有 3*3=9 種 state）

    ![img](https://i.imgur.com/hjxqnsB.jpg)

然後，我們把 transition 定義清楚：

1. 當機器人選擇去拿 bottle 或 glass，人類出手干預的機率

    從下表中可以看出，若人相信機器人，當機器人要拿 bottle 時，人干預的機率是 0.1；若人相信機器人，當機器人要拿 glass 時，人干預的機率是 0.2；依此類推。

    ![img](https://i.imgur.com/syrrbBK.jpg)

    這張圖中有點小錯誤，第一列的最右欄應該是 $P(a^H_t=Intervene|s^H_t, a^R_t)$。

2. 當人已經出手干預（或不干預），world state 會怎麼變化

    ![img](https://i.imgur.com/s8uBCiq.jpg)

3. 人的 state 會怎麼變化

    ![img](https://i.imgur.com/cUWOq8W.jpg)

有了以上的這些機率之後，我們就可以計算 transition matrix。

最後，我們需要定義 reward：

![img](https://i.imgur.com/iVddwVR.jpg)

## 審視一下現在的 model，還缺一些推導

看到這邊，大家應該會覺得不太懂，怎麼感覺起來跟之前學到的 MDP 不太一樣？之前我們學到的是我們只有一個機器人會採取 action，但因為 action 會帶來的結果是 non-deterministic，所以我們才需要 MDP 來 model。

但按照我們上面的定義，我們會畫出一個這樣的 graph：

![img](https://i.imgur.com/cJgqu1L.jpg)

- human state（$s^H_t$）跟 robot action（$a^R_t$） 會決定 human action（$a^H_t$）
- $s^H_t$、$a^R_t$ 跟 $a^H_t$ 會決定下一時刻 t+1 的 human state
- ...依此類推

這跟我們原本認知的 MDP：

![img](https://i.imgur.com/CfDJGGq.jpg)

長得不大一樣。

## 開始推導成 MDP

巧妙的地方來了，我們可以將 state space 用 human state 跟 world state 一起定義

$$ S = S^H * S^W$$

所以這張圖裡面的 human state space 和 world state space 就可以合併（紅的合在一起、藍的合在一起）：

![img](https://i.imgur.com/kAh8Gc2.jpg)

然後，我們可以把 $a^H_t$ 視為 MDP 裡面的不確定因素。因為機器人無法掌握人的行為，所以機器人不知道自己今天若去抓 bottle，到底能不能到達 "成功抓起 bottle" 的 state，而這就是符合原本 MDP 精神的地方。

雖然接下來還有一些數學上的推導，不過個人覺得有點太過細節，有興趣的讀者可以去看看 [這個 note 的 page.5-6](http://stefanosnikolaidis.net/course-files/Lecture4_notes.pdf)，對於機率的 marginalization 無感的話可以看我之前寫的 [Why do we need marginalization in probability? (為什麼我們在機率中需要討論邊緣化)](https://pojenlai.wordpress.com/2018/11/11/why-do-we-need-marginalization-in-probability-%E7%82%BA%E4%BB%80%E9%BA%BC%E6%88%91%E5%80%91%E5%9C%A8%E6%A9%9F%E7%8E%87%E4%B8%AD%E9%9C%80%E8%A6%81%E8%A8%8E%E8%AB%96%E9%82%8A%E7%B7%A3%E5%8C%96/)。

## 程式碼

整段 code 主要的順序還是

1. 定義 state
2. 定義一些基本的轉換機率（為了計算　transition matrix）
3. 計算 transition matrix
4. 定義 reward
5. 用 value iteration 算出 optimal policy

然後就上 code 啦：

```Matlab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% CSCI 699: Computational Human-Robot Interaction %%%%%
%%%%% Fall 2018, University of Southern California    %%%%%
%%%%% Author: Stefanos Nikolaidis, nikolaid@usc.edu   %%%%%
%%%%% Commented by: Po-Jen Lai, pojenlai@usc.edu      %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
clear all
close all
 
## 定義 state
 
human_states_str = {'no_trust', 'trust'};
 
Label.BOTTLE = 1;
Label.GLASS = 2;
 
Label.ON_TABLE = 1;
Label.PICKED_BY_ROBOT = 2;
Label.PICKED_BY_HUMAN = 3;
 
Label.NO_TRUST = 1;
Label.TRUST = 2;
 
objstate_str = {'on_table','picked_robot','picked_human'};
ractions_str = {'pick_bottle','pick_glass'};
 
## 定義一些基本的轉換機率（為了計算　transition matrix）
 
%{no_trust,trust} x{bottle, glass}
PROB_TRUST_INCREASE = [0.8 0.9;
    0.0 0.0];
 
%{no_trust,trust} x {bottle,glass}
PROB_TRUST_INTERVENE = [0.3 0.8;
    0.1 0.2];
 
counter = 1;
for ii = 1:3 %for each object
    for jj = 1:3 %for each  state
        world_states(counter,:) = [ii,jj];
        counter = counter + 1;
    end
end
num_human_states = length(human_states_str);
num_world_states = counter -1;
num_trust_states = 2;
num_states = num_world_states*num_trust_states;
num_ractions = length(ractions_str);
 
## 計算 transition matrix
 
Trans = zeros(num_states,num_ractions,num_states);
 
for sh = 1:num_human_states
    for sw = 1:num_world_states
        for ra = 1:num_ractions
            world_state = world_states(sw,:);
            ss = (sh-1)*num_world_states + sw;
            
            %picked by robot
            new_world_state = world_state;
            new_world_state(ra) =Label.PICKED_BY_ROBOT;
            nsw = findWorldState(new_world_state,world_states);
            
            nsh = sh; %trust stays the same
            nss = (nsh-1)*num_world_states + nsw;
            Trans(ss,ra,nss) = (1-PROB_TRUST_INTERVENE(sh,ra))*(1-PROB_TRUST_INCREASE(sh,ra));
            
            if sh == 1 %trust can increase only if low
                nsh = sh+1;
                nss = (nsh-1)*num_world_states + nsw;
                Trans(ss,ra,nss) = (1-PROB_TRUST_INTERVENE(sh,ra))*PROB_TRUST_INCREASE(sh,ra);
            end
            
            %picked by human
            new_world_state = world_state;
            new_world_state(ra) =Label.PICKED_BY_HUMAN;
            nsw = findWorldState(new_world_state,world_states);
            
            nsh = sh; %trust stays the same
            nss = (nsh-1)*num_world_states + nsw;
            Trans(ss,ra,nss) = PROB_TRUST_INTERVENE(sh,ra);
        end
    end
end
 
disp(' ')
disp('Transition Matrix')
for ss = 1:num_states
    for ra = 1:num_ractions
        sh = floor(ss/(num_world_states+1)) + 1;
        sw = ss - (sh-1)*num_world_states;
        nIndices = find(Trans(ss,ra,:)>0);
        %do not worry about invalid actions
        if (world_states(sw,ra) == Label.ON_TABLE)
            str = strcat('if~', human_states_str{sh} , ' and bottle is~' , objstate_str(world_states(sw,1)) , ' and glass is~' , objstate_str(world_states(sw,2)) , ' and robot does~' , ractions_str{ra},':');
            disp(str);
            for nn = 1:length(nIndices)
                nss = nIndices(nn);
                
                nsh = floor(nss/(num_world_states+1)) + 1;
                nsw = nss - (nsh-1)*num_world_states;
                str = strcat('then the prob of ~', human_states_str{nsh} , ' and bottle is~' , objstate_str(world_states(nsw,1)) , ' and glass is~' , objstate_str(world_states(nsw,2)),':',num2str(Trans(ss,ra,nss)));
                disp(str);
            end
        end
    end
end
 
## 定義 reward
 
%reward function
Rew = zeros(num_states,num_ractions);
for ss = 1:num_states
    for ra = 1:num_ractions
        sh = floor(ss/(num_world_states+1)) + 1;
        sw = ss - (sh-1)*num_world_states;
         
         %say bonus if starts with glass
         if (world_states(sw,Label.GLASS)==Label.PICKED_BY_ROBOT)&& (world_states(sw,Label.BOTTLE)==Label.ON_TABLE)
           Rew(ss,ra) = 5;
         end
          
        %if we are not in a final state
        if (world_states(sw,1)==Label.ON_TABLE) || (world_states(sw,2)==Label.ON_TABLE)
            if world_states(sw,ra)~= Label.ON_TABLE %penalize infeasible actions
                Rew(ss,ra) = -1000;
            end
        end
    end
end
 
%add positive reward for goal.
goal = findWorldState([Label.PICKED_BY_ROBOT, Label.PICKED_BY_ROBOT],world_states);
for ra = 1:num_ractions
    for sh = 1:2
        ss = (sh-1)*num_world_states + goal;
        Rew(ss,ra) = 10;
    end
end
 
%print reward function
disp(' ')
disp('reward function')
for ss = 1:num_states
    for ra = 1:num_ractions
        sh = floor(ss/(num_world_states+1)) + 1;
        sw = ss - (sh-1)*num_world_states;
        str = strcat('if~', human_states_str{sh} , ' and bottle is~' , objstate_str(world_states(sw,1)) , ' and glass is~' , objstate_str(world_states(sw,2)) , 'and robot action is~',ractions_str{ra}, ' then reward is: ',num2str(Rew(ss,ra)));
        disp(str);
    end
end
 
## 用 value iteration 算出 optimal policy
 
%value iteration
T = 3;
V = zeros(num_states,1);
policy = zeros(num_states,T);
new_V = zeros(num_states,1);
Q = zeros(num_states, num_ractions);
for tt = T:-1:1
    for ss = 1:num_states
         
        %check if terminal state
        if ss == 12
            debug = 1;
        end
        sh = floor(ss/(num_world_states+1)) + 1;
        sw = ss - (sh-1)*num_world_states;
        if ((world_states(sw,1)~=Label.ON_TABLE) && (world_states(sw,2)~=Label.ON_TABLE))
            new_V(ss) = Rew(ss,1);
            policy(ss,tt) = 1;
            continue;
        end
         
         
        maxV = -1e6;
        maxIndx = -1;
        for ra = 1:num_ractions
            res = Rew(ss,ra);
            for nss = 1:num_states
                res = res +  Trans(ss,ra,nss)*V(nss);
            end
            Q(ss,ra) = res;
            if res > maxV
                maxV = res;
                maxIndx = ra;
            end
        end
        new_V(ss) = maxV;
        policy(ss,tt) = maxIndx;
    end
    V = new_V;
end
 
disp(' ')
disp('policy')
%print policy
for tt = 1
    tt
    for ss = 1:num_states
        sh = floor(ss/(num_world_states+1)) + 1;
        sw = ss - (sh-1)*num_world_states;
        if ((world_states(sw,1) == Label.ON_TABLE)||(world_states(sw,2) == Label.ON_TABLE)) %we care only about feasible states
            str = strcat('if~', human_states_str{sh} , ' and bottle is~' , objstate_str(world_states(sw,1)) , ' and glass is~' , objstate_str(world_states(sw,2)) , 'then robot does: ',ractions_str{policy(ss,tt)});
            disp(str);
        end
    end
end
```

## 總結

今天跟大家簡單介紹了一下該怎麼寫 MDP 的程式，雖然沒有細到每一行程式碼都解釋清楚，但對於想要自己弄懂並應用的讀者來說應該已經有些幫助，若有問題歡迎你在下方留言討論！

## 延伸閱讀

1. [Computational Human-Robot Interaction](http://www.stefanosnikolaidis.net/comphri.html)
2. [awesome-human-robot-interaction](https://github.com/Po-Jen/awesome-human-robot-interaction)

關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人、電腦視覺和人工智慧有少許研究，正在學習[用心體會事物的本質](https://buzzorange.com/techorange/2017/07/10/elon-musk-first-principle/)跟[不斷進入學生心態改進](https://www.ted.com/talks/eduardo_briceno_how_to_get_better_at_the_things_you_care_about)。
