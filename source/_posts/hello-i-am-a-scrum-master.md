---
title: 當一個 Scrum Master 是一個怎樣的體驗？
date: 2019-01-05 10:23:23
author: kdchang
tags: 
    - scrum
    - scrum master
    - product owner
    - dev tame
    - 軟體工程師
    - 軟體工程
    - software engineering
    - 敏捷開發
    - agile
---

![當一個 Scrum Master 是一個怎樣的體驗？](/img/kdchang/scrum101/i-am-scrum-master.png)

# 前言
[Scrum](https://zh.wikipedia.org/wiki/Scrum) 是一個以團隊為基礎來開發複雜系統和產品的敏捷開發框架和方法論。相信很多軟體開發人員都聽過 Scrum，而且也把它運用在產品開發中，然而每個團隊在使用 Scrum 時都會遇到不同的問題，有些是共通性的問題，有些是個別團隊所遇到的問題。過去一段時間在筆者的職業生涯中因緣際會地經歷了開發者、產品負責人以及 Scrum Master 等角色，對於不同角色的特性和所應具備的特質和能力以及 Scrum 的能與不能都有些許的認識。接下來，本文將透過一個 Scrum Master 視角去探討 Scrum Master 在 Scrum Team 中所扮演的角色和可能遇到的挑戰和機遇。

![當一個 Scrum Master 是一個怎樣的體驗？](/img/kdchang/scrum101/scrum-rules.png)

# Scrum 先修課：關於 Scrum 你必須先明白的專有名詞
## Role 角色
1. Product Owner
    負責規劃產品 road map，負責釐清使用者和 Stakeholder 的需求並轉化成具有商業價值的 User Story，並評估 Story 的優先順序，決定哪些項目要從 Backlog 進入到 sprint 當中。此外，Product Owner 也必須和 Dev Team 成員討論出 Story 的 Acceptance criteria 和負責驗收成果

2. Scrum Master / Agile Coach
    又稱敏捷教練，負責導入 Scrum 流程並培養團隊成員具備 agile 思維。Scrum Master 負責主持 Scrum 相關會議與 process 和協助管理分析 Sprint 產出物，此外也必須協助整個流程持續改善，最終讓團隊達到 process self-managed 狀態。一般而言，Scrum Master 在團隊裡通常沒有實際管理權但必須協助其他成員在遇到 block 事情時去除障礙，有時也要扮演輔導長的角色激勵團隊士氣。簡言之，Scrum Master 是一個必須對於產品開發流程以及溝通能力有一定掌握並具備一定的技術實力的角色。說真的，真正要當的好 Scrum Master，不容易

3. Development Team / Dev Team
    具備跨職能的開發團隊成員（可以完成 User Story 的所有開發工作，包含開發、測試等），一般遵守兩塊 pizza 原則，以不超過 6-8 人為原則。團隊成員必須具備互助合作的騎士精神，以團隊目標為目標，在互相協助的基礎下一起在 Sprint 結束前完成於 Sprint 開始時所承諾應該交付的任務

4. Stakeholder
    利害關係人，需求的起源地。在直接面對消費者市場的產品團隊可能是一般消費者，若是開發內部工具或是面對企業市場的產品團隊，利害關係人可能會是 AM（客戶經理）、BD（商務拓展）、Tech Support（技術支援）、Marketing（行銷）等單位的同仁

![當一個 Scrum Master 是一個怎樣的體驗？](/img/kdchang/scrum101/scrum-process.jpg)

## Event 活動
1. Sprint
    Sprint 在橄欖球中為衝刺的意思，指的是一個 Scrum 的週期。通常以 2-4 週為一個單位，理論上除了緊急 bug 或是 task 外，不能於 Sprint 開始後額外新增 Story 進入 Sprint 當中

2. Sprint Planning，會議時間：1-2 hours
    於 Sprint 開始前舉辦的會議。根據 Product Owner 的優先順序和團隊可以消耗的點數（velocity）評估哪些 Story 需要進入到 Sprint 當中的會議。由 Scrum Master 主持，Product Owner 和 Dev Team 參與，由 Dev Team 認領想要負責的 Story，Dev Team 於會議結束後根據 Story 劃分成更細部的 task，然後 Scrum Master 確認沒問題後開始 Sprint 的週期

3. Sprint Gromming (Backlog Refinement/Stroy time)，會議時間：一次 2-3 hours 或分成兩次，每次 1-1.5 hours
    於 Sprint 中間舉辦的會議，主要目的在評估下一個 Sprint 中要從 Backlog 中拿出來做的 Story 點數的估計（Planning poker）。一般透過 Product Owner 說明 User Story 然後和 Dev Team 討論定義出以 User 角度表述的 Acceptance criteria。記得這邊的 Story Point 是相對值並不是完全 mapping 到時間，而且每個 team 的狀況都會不一樣，主要是根據你們 team 對於這項 Story 的 Complexity、Uncertainty 和 Effort 去做評估，一般使用費式數列表示（1、2、3、5、8、13、21...）。這個會議十分重要，透過 Planning poker 的過程可以讓大家對於 Story 的認知和知識可以 align 到同一條線上（例如估計點數不一致時彼此說明和討論，增進對於 Story 和系統的了解）。記得，若是 User Story 過大，沒辦法在一個 Sprint 完成會建議把它切成比較小的 Story。若是連 Product Owner 都沒辦法描述很清楚的 User Story 會要求 Product Owner 先去釐清需求在下一次 Sprint Gromming 會議或是 Sprint Planning 前再次 repoint
    
4. Sprint Review (Sprint Demo)，會議時間：兩次 30mins（利害關係人/內部團隊）
    在 Sprint Review 中，Scrum Master 會邀請 Stakeholder 參與會議，由 Scrum Master 簡介這次 Sprint 所完成的 Shippable Product Increment（可交付的產品增量）和 Dev Team 展示開發成果。讓市場業務端和系統開發端可以彼此溝通並確認產品方向與市場吻合，若是有需要產品改善或是新的需求產生將由 Product Owner 釐清後放入 Backlog 中

5. Sprint Retrospective（Sprint Retro），會議時間：一次 1 hour
    Sprint Retro 簡而言之式是 Scrum team 於 Sprint 結束時的閉門會議。通常在透過感謝的對象來緩和氣氛，並檢討這次 Sprint 的流程是否有需要檢討的部份或是做的不錯的部份，若是沒有完全完成所承諾要完成的事情，必須了解原因（例如低估複雜度或是中間有遇到 block 影響開發）。通常會搭配 Sprint Planning Doc 和 Burn down chart 以及上次的 Sprint Retrospective Doc 列出的 todo item 來進行檢討

6. Daily Scrum (Daily Standup Meeting)
    每天約 15 分鐘的站立會議，每位 Scrum team 更新自己昨天完成的事項、今天預計完成的事項以及是否有遇到什麼 block 阻礙。若是有 block 將由 Scrum Master 溝通協調並調度團隊資源協助移除成員障礙。若是有無法在 Sprint 結束前完成的 Story 也要讓 Product Owner 提早知道。另外，Burn down 也是需要在 Daily Scrum 大家要一起觀看 Sprint 是否有正常運作的指標

7. User Stroy Mapping / workshop，會議時間：一次 1.5-2 hours
    一般不能算是標準的 Sprint Event，通常是 Product Owner 在對於開發新產品的掌握度比較低，需要開發成員一起協助定義 User Story 所召開的會議

## Artifact 產出物
1. Backlog
    裝有待完成的 Story 或是 Bug 的袋子，在 Sprint Gromming 時根據 Priority 進行點數估計，於 Sprint Planning 決定哪些已估計 Story 需要進入這次的 Sprint 當中

2. Sprint Planning Doc
    記錄當次 Sprint 所承諾要完成的 Story 和需要修復的 Bug（通常為插單緊急 Bug 或是不緊急但有在 Gromming 進行點數估計的 Story），Dev Team 成員需要定期更新所負責的 Story 的相關文件

3. Sprint Gromming Doc
    記錄當次 Sprint 所承諾要完成的 Story / Acceptance criteria 和需要修復的 Bug 的點數估計。

4. Sprint Retrospective Doc
    記錄 Sprint 檢討會議中的待辦事項和檢討事項

5. Burn down chart / Burn up chart
    以團隊為單位，記錄 Sprint 完成狀況的圖表，Burn down chart 記錄點數完成下降的趨勢。Burn up chart 記錄累積完成的點數和 scope 的趨勢

    ![當一個 Scrum Master 是一個怎樣的體驗？](/img/kdchang/scrum101/burn-down-burn-up.jpg)

6. Scrum Board
    記錄 Sprint 過程中每一個 Story 以及 Story 下 task 的狀況，是在 Todo 還是 In Progress 還是 Done

    ![當一個 Scrum Master 是一個怎樣的體驗？](/img/kdchang/scrum101/scrum-board.png)

7. Definition of Done（DoD）
    團隊定義的驗收成果標準，例如開發一個新功能必須含：Dev + QA + Doc + Deploy 才算 Done，是每一個 Story 都必須遵守的完成標準

8. Epic / Theme / User Story / Acceptance criteria
    User Story 是用使用者需求角度撰寫的有商業價值的一段敘述，而 Acceptance criteria 則是更為細部的補充描述。當有數個 story 隸屬於某一個相關 feature 時通常會把它歸為同一個 Theme，若是一個大的產品或是系統需要橫跨數個 sprint 時會把這些 Story 都歸類為同一個 Epic

    ```
    # User Stroy
    我是一個網站使用者
    我想要可以提交 feedback
    這樣網站開發者就可以根據使用者的回饋去優化網站

    # Acceptance criteria
    當使用者點選右下角的圓點按鈕後，可以看到表單，表單上面有姓名、email 和回饋訊息三個欄位
    當使用者填完表單送出後若成功會閃出一個提交成功的訊息
    當使用者表單項目有缺漏時會有提示訊息提示
    ```

# 那些擔任 Scrum Master 學到的事
在了解 Scrum 相關的專有名詞後我們就可以開始我們 Sprint 啦！但事實上，Scrum 只是一種框架和方法論，實際我們在執行 Scrum 時往往會因為團隊成員的不同或是組織文化的不同而產生不同的效果。接著讓我們來看看實際上在執行 Scrum 會有哪些需要留意的觀念

1. 基礎建設和團隊素質很重要
    CI/CD 基礎建設、DevOps 文化、Agile 精神、團隊是否具備開發上所需的所有技能等都是 Scrum 運作能否順暢很重要的一環，若是基礎建設不穩，很多時候我們的點數估計往往會低估，造成 Sprint 運作上的困難。另外若是團隊成員都具備 T 型人才的能力也將讓 Scrum 運作更順暢

2. 我們是一個職業球隊
    Sprint 是以團隊為基礎，Burn down chart 不是一個人的 Burn down chart，所以若是有團隊夥伴遇到困難是需要其他夥伴協助（例如：對於系統架構不熟悉），其他比較熟悉的夥伴應該主動給予協助。若是有夥伴無法如期完成任務，其他若有夥伴已經完成手上的事情，也可以主動協助一起完成剩下的工作，確保 Sprint 結束後當初承諾的任務都是有可交付的產出

3. Scrum 是一面照妖鏡
    透過 Scrum 可以讓當初不清楚的使用者需求、無根據的預估工作時間和無限制需求的亂象可以被攤在陽光下檢視和討論，才有機會透過 Scrum process 來管理和改善

4. 通常絆腳石是你不能說名字的那個人
    在推行 Scrum 時，公司高層和文化的支持也是十分重要，因為通常公司推展 Scrum 或是敏捷開發會失敗，通常絆腳石是你不能說名字的那個人

5. 保持你的步伐
    排定好時間後就固定下來，讓團隊成員可以調整好自己的節奏並有更多事情可以專注在任務完成上而非冗長會議或是雜事上

6. 要測量才能進步
    每次 Sprint 結束一定要有檢討並透過數據圖表等檢視可以改善的部分，持續優化整個流程

7. 軟體開發流程沒有銀彈
    講了那麼多，事實上 Scrum 也並非萬能，碰上常常會有臨時性任務或是 operation issue 特性任務的團隊，或許可以考慮使用 Kanban，更甚於 Scrum。另外，像是技術研究單位，使用 Scrum 也未必是一種好選擇

# 總結
以上透過一個 Scrum Master 視角去探討 Scrum Master 在 Scrum Team 中所扮演的角色和可能遇到的挑戰和機遇。Scrum Master 是一個必須對於產品開發流程以及溝通能力有一定掌握並具備一定的技術實力的角色。說真的，真正要當的好 Scrum Master，不容易。事實上，Scrum 真正目標在於建立一個自我管理的組織，讓 Process 而非人來管理。當 Scrum 真正成功的時候，就是 Scrum Master 可以退休的時候了。

關於作者：
[@kdchang](http://blog.kdchang.cc) 文藝型開發者，夢想是做出人們想用的產品和辦一所心目中理想的學校。A Starter & Maker. JavaScript, Python & Arduino/Android lover.:) 

# 參考文件
1. [燃尽图 （Burn up and Burn down Chart）—介绍](https://zhuanlan.zhihu.com/p/28314519)
2. [Clear Acceptance Criteria and Why They’re Important](https://rubygarage.org/blog/clear-acceptance-criteria-and-why-its-important)
3. [當一個 Scrum Master 是一個怎樣的體驗？](https://blog.kdchang.cc/2019/01/04/what-is-the-experience-of-be-a-scrum-master/)

（image via [eohcoastal](https://www.eohcoastal.co.za/uncategorized/tell-effective-scrum-master/)）
