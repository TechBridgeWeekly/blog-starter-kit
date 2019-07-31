---
title: 設計模式 - 工廠方法及抽象工廠
date: 2017-05-22 07:12:25
tags:
	- design pattern
author: jyt0532
---

以下文章是閱讀 [深入淺出 Design Pattern](http://www.oreilly.com.tw/product_java.php?id=a163)，還有 [聖經](https://www.amazon.com/Design-Patterns-Elements-Reusable-Object-Oriented/dp/0201633612)的筆記，要更深入的理解一定要去看這兩本書。圖片截圖自 lynda.com 的[Foundations of Programming: Design Patterns](https://www.lynda.com/Developer-Programming-Foundations-tutorials/Foundations-Programming-Design-Patterns/135365-2.html)，這是學習設計模式非常好的課程。

### 來點個 Pizza 吧

今天想設計一個PizzaStore，裡面可以點 Pizza。

```java
public class PizzaStore {
    Pizza orderPizza(String type){
    	Pizza pizza;
    	if(type.equals("cheese")){
	    pizza = new CheesePizza();
    	}else if(type.equals("greek")){
	    pizza = new GreekPizza();
    	}else if(type.equals("pepperoni")){
	    pizza = new PepperoniPizza();
    	}
    	pizza.prepare();
    	pizza.cook();
    	return pizza
    }
}
```

看起來不差。compile-time 的時候 pizza.prepare() 跟 pizza.cook() 是哪一種 pizza 我不用知道 ，我只要保證各個 Pizza 的 subclass 有實作 prepare() 和 cook() 就可以 。

Polymorphism Rocks!

但只要有新的 Pizza 推出或是有舊的要拿掉，需要改這裡的 if-else 有點麻煩。

### 簡單工廠模式

別忘了 **Design Pattern rule #1: Encapsulate what varies**

把剛剛 orderPizza() 裡面的所有 if-else 拉出來到一個工廠裡，這個工廠專門製作 pizza。

```java
public class SimplePizzaFactory {
    public Pizza createPizza(String type) {
	Pizza pizza = null;
	if(type.equals("cheese")){
	    pizza = new CheesePizza();
    	}else if(type.equals("greek")){
	    pizza = new GreekPizza();
    	}else if(type.equals("pepperoni")){
	    pizza = new PepperoniPizza();
    	}
    	return pizza;
    }
}
```

就這麼簡單，這就是簡單工廠 。

**簡單工廠管理物件的創造，如果 client 要取得物件，只要給簡單工廠正確的參數就可以。**

然後 pizza 店的 constructor 要丟一個工廠進去。
```java
public class PizzaStore {
    SimplePizzaFactory factory;
    public PizzaStore(SimplePizzaFactory factory) { 
	this.factory = factory;
    }
    public Pizza orderPizza(String type) {
	Pizza pizza;
	pizza = factory.createPizza(type);
	pizza.prepare();
	pizza.cook();
	return pizza;
    }
}
```

簡單工廠模式，讓我們把 pizza 的***創造***和 pizza 的***使用***分開了，減少了 client 對於實作的依賴。

我們成功的判斷出 orderPizza 這個函數裡面會變動的部分，分離出一個工廠，去處理他。如果你今天要改變處理方式，你去改那個工廠或是給我一個新工廠就可以。我不管你怎麼創造的，我只在乎你回傳給我的 object 的 class 是 Pizza 的 subclass。

![Alt text](/img/jyt0532/factory1.png)

簡單工廠模式的優點就是分離了物件的使用和創造。client 不管你怎麼生成的，但缺點也很明確，每當有新的 class 出來工廠就要改，複雜度上升得很快。

適用情況是需要創建的種類比較少，而且客戶對於怎麼創建對象的方法不關心。
 
### 生意不錯 開個分店

實作了簡單工廠模式之後，同樣是 cheese pizza，紐約跟芝加哥的做法就完全不一樣。
我們可以修改 SimplePizzaFactory 讓 createPizza 多吃一個參數 style。
```java
public class SimplePizzaFactory {
    public Pizza createPizza(String style, String type) {
	Pizza pizza = null;
	if(style.equals("NY")){
	    if(type.equals("cheese")){
	    	pizza = new NYStyleCheesePizza();
    	    }else...
	    ...
	}else if(style.equals("chicago")){
	    if(type.equals("cheese")){
                pizza = new ChicagoStyleCheesePizza();
            }else...    
            ...
	}
    	return pizza;
    }
}
```
![Alt text](/img/jyt0532/conan4.gif)
好啦我們都那麼熟了，再演就不像了，我們在這裡應該用繼承。

ChicagoPizzaFactory 跟 NYPizzaFactory 都繼承自 SimplePizzaFactory。
現在我們需要為不同 style 的 PizzaStore 建立不同的工廠。

如果今天我要點 NY 風味的起司 pizza。

```java
NYPizzaFactory nyFactory = new NYPizzaFactory();
PizzaStore nyStore = new PizzaStore(nyFactory);
nyStore.orderPizza("cheese");
```
明天我想點芝加哥風味的起司 pizza
```java
ChicagoPizzaFactory chicagoFactory = new ChicagoPizzaFactory()
PizzaStore chicagoStore = new PizzaStore(chicagoFactory);
chicagoStore.orderPizza("cheese");
```

看似沒啥問題，我們現在其實進入了見山不是山見水不是水的境界。

![Alt text](/img/jyt0532/spin.gif)

為什麼這麼說呢？我們為了decouple**物件的創造**和**物件的使用**，製造了一個工廠。
可是為了 reuse 工廠的 code，我們使用了繼承。
![Alt text](/img/jyt0532/factory3.png)

現在我的 SimplePizzaFactory 其實只是一個介面，我定義了所有繼承了我的 class 應該要做什麼事（返回 customized defined pizza），真正實際創造物件的地方是子類別的實體工廠。

這兩種功能（decouple + hierarchy）同時需要的時候，我們就可以用上今天的主角。

### 工廠方法模式

**工廠方法模式定義了一個建立物件的介面，但由子類決定要實例化的類別為何。工廠方法讓類別把** _**實例化**_ **的動作推遲到了子類**。

我們現在把 createPizza 拉回來 PizzaStore 裡，**讓子類別來決定怎麼 createPizza**。
```java
public abstract class PizzaStore {
    abstract Pizza createPizza(String type);
    public Pizza orderPizza(String type) {
	Pizza pizza = createPizza(type);
	pizza.prepare();
	pizza.cook();
	return pizza;
    }
}
```
createPizza是抽象方法，留給子類別繼承。

讓 NYPizzaStore 去繼承 PizzaStore，實作 createPizza。
```java
public class NYPizzaStore extends PizzaStore {
    Pizza createPizza(String item) {
	if (item.equals("cheese")) {
   	    return new NYStyleCheesePizza();
	} else if (item.equals("veggie")) {
	    return new NYStyleVeggiePizza();
	} else if (item.equals("clam")) {
	    return new NYStyleClamPizza();
	} else if (item.equals("pepperoni")) {
	    return new NYStylePepperoniPizza();
	} else return null;
    }
}
```

原本我物件的建立，交給一個外來的工廠處理，現在我把它交給我的子類別處理，而且父類別還可以 call子類別實作的函數。
![Alt text](/img/jyt0532/factory2.png)
這種會互 call 的 function 通常依賴性都很高，但我們利用工廠模式讓父類別跟子類別的依賴鬆綁（decouple）了。

套用了工廠方法模式之後，怎麼點 pizza 呢？

```java
PizzaStore nyStore = new NYPizzaStore();
Pizza pizza = nyStore.orderPizza("cheese");
```

輕鬆！

### 結構

![Alt text](/img/jyt0532/factory4.png)

* Product（Pizza）: 定義 factoryMethod（createPizza） 所造物件的介面。

* ConcreteProduct（NYStyleCheesePizza）: 實作 Product。

* Creator（PizzaStore）: 宣告 factoryMethod（必須傳回Product），和其他 client 可以 call 的 API。

* ConcreteCreator（NYPizzaStore）: 實作 factoryMethod，回傳 ConcreteProduct 的 instance。

有個小細節，其實工廠方法不一定是 abstract。也可以 Creator 就先偷偷實作 factoryMethod，回傳 Product，subclass 可以選擇要不要 override 工廠方法。


### 優缺點

優點除了跟簡單工廠一樣，隱藏了創建物件的細節，最重要的是加入新產品不需要改動 Creator，你直接繼承 Creator 就好了 。

Client 的用法都是一樣不需要改，完全符合[開放封閉守則](https://www.jyt0532.com/2017/04/18/decorator/#開放封閉守則)。

缺點就是 ConcreteCreator 跟 ConcreteProduct 會成對的增加。比如你今天想做加州披薩，你在定義完加州 pizza之後，還要再定義一個加州 pizza 工廠。

![Alt text](/img/jyt0532/problemFactory.jpg)

### 工廠方法的限制

工廠方法的 factoryMethod，只能創建一個對象，比如說 Pizza 。

但如果我們想要更加細分想創建的東西，比如說 Pizza 的所需原料（麵團、醬料、起司、蛤蠣）。如果我們用工廠方法的話，我們需要為每一個原料都創一個工廠。

在這個例子就是 NY 麵團工廠、NY 醬料工廠、NY 起司工廠、NY 蛤蠣工廠、Chicago 麵團工廠、Chicago 醬料工廠、Chicago 起司工廠、Chicago 蛤蠣工廠。

**因為每個工廠方法只能生產一個產品**。


```java
public class NYPizzaStore extends PizzaStore {
    Pizza createPizza(String item) {
	DoughFactory doughFactory = new NYDoughFactory();
	SauceFactory sauceFactory = new NYSauceFactory();
	CheeseFactory cheeseFactory = new NYCheeseFactory();
	ClamFactory clamFactory = new NYClamFactory();
	if (item.equals("cheese")) {
    	    return new NYStyleCheesePizza(doughFactory, 
		sauceFactory, cheeseFactory);
	} else if (item.equals("veggie")) {
	    return new NYStyleVeggiePizza(doughFactory, 
		sauceFactory, cheeseFactory);
	} else if (item.equals("clam")) {
	    return new NYStyleClamPizza(doughFactory, 
		sauceFactory, cheeseFactory, clamFactory);
	} else if (item.equals("pepperoni")) {
	    return new NYStylePepperoniPizza(doughFactory, 
		sauceFactory, cheeseFactory);
	} else return null;
    }
}
```

這樣實在是太難 maintain 了。**所以我們把相關的產品（NY 麵團工廠、NY 醬料工廠、NY 起司工廠、NY 蛤蠣工廠）組成一個產品族，交給同一個工廠來生產**，鼎鼎大名的抽象工廠就誕生了。

![Alt text](/img/jyt0532/abstractfactory1.png)

抽象的 Pizza 原料工廠定義了每個原料工廠要創建的東西的介面。
每個繼承了 Pizza 原料工廠的具體原料工廠乖乖 implement **所有**需要創建的東西。

再來些例子，如果工廠方法生產的是房子，抽象工廠可以生產一個房子的所有家具（沙發電視電風扇）。
如果工廠方法生產的是武士，抽象工廠可以生產一個武士的所有配備（盔甲、鞋子、手套）。

第一眼看起來很可怕，但其實只是把工廠的責任從生產一個產品，變成生產一個產品族。

### 產品族和產品等級結構

先解釋兩個名詞。

產品等級結構：產品的繼承結構。比如一個抽象類是麵糰，子類別有 Chicago 麵團跟 NY 麵團，這三個形成了一個產品等級結構。

產品族：同一個工廠生產的所有產品，其中的每個產品都是座落在不同的產品等級結構中的其中一個產品。

一圖勝過千言萬語。
![Alt text](/img/jyt0532/abstractfactory2.png)

### 套用抽象工廠後
```java
public class CheesePizza extends Pizza {
    PizzaIngredientFactory = ingredientFactory;
    public CheesePizza(PizzaIngredientFactory ingredientFactory){
	this.ingredientFactory = ingredientFactory;
    }
    void prepare(){
	dough = ingredientFactory.createDough();
	sauce = ingredientFactory.createSauce();
	cheese = ingredientFactory.createCheese();
    }
}
```
至於 PizzaStore 跟他的子類，概念跟工廠方法一樣，由子類決定要實例化的類別為何。
```java
public class NYPizzaStore extends PizzaStore {
    protected Pizza createPizza(String item) {
	Pizza pizza = null;
	PizzaIngredientFactory ingredientFactory = new NYPizzaIngredientFactory();
	if (item.equals("cheese")) { 
  	    pizza = new CheesePizza(ingredientFactory);
	} else if (item.equals("veggie")) {
  	    pizza = new VeggiePizza(ingredientFactory);
	} else if (item.equals("clam")) {
	    pizza = new ClamPizza(ingredientFactory);
	} else if (item.equals("pepperoni")) {
	    pizza = new PepperoniPizza(ingredientFactory);			
	} 
	return pizza;
    }
}
```
點 pizza 的方法就跟工廠方法一樣。
```java
PizzaStore nyStore = new NYPizzaStore();
Pizza pizza = nyStore.orderPizza("cheese");
```

對 client 來說用法一樣，client 並不知道裡面的產品已經變成產品族。

### 抽象工廠
**用一個抽象工廠來定義一個創建** _**產品族**_ **的介面，產品族裡面每個產品的具體類別由繼承抽象工廠的實體工廠決定**。

上面那句話請務必讀懂他

### 結構

![Alt text](/img/jyt0532/abstractfactory3.png)

* AbstractFactory（PizzaIngredientFactory）: 宣告出各個創建同一產品族產品的介面。

* ConcreteFactory（NYPizzaIngredientFactory）: 實作 AbstractFactory。

* AbstractProduct（Dough）: 宣告產品等級結構的物品介面。

* Product（ThickCrustDough）: ConcreteFactory 所建構的成品，需要實作 AbstractProduct。

### 優缺點

1.一樣區隔了每個產品的生成和使用，client 被隔離在產品的 class 之外。client 甚至不知道什麼產品被創建了。他只要知道那個產品有哪些函數可以給他 call，這個特點使得抽換具體工廠這件事變得非常容易。

2.性質類似的產品集中管理（所有 NY 的原料一起管理，或是所有日式的傢俱一起管理）。今天有新的工廠要進來，他需要 implement 的 method 非常明確，照著 AbstractFactory 定義的介面實作就對了。某種程度而言也算是給 client 方便，我保證他只會用到同一個產品族的對象。

3.缺點非常致命，就是當**我想在產品族加一個產品**非常困難。因為我所有子工廠要跟著改，這被稱為開閉原則的傾斜性：**新增產品族容易，但新增產品結構困難**。

### 使用時機
1.一個系統必須和產品的生成/組合，保持獨立。

2.許多類似的產品可以組成產品族，方便集中管理，**而且多於一個的產品族**。

3.只想公開產品 interface 不想公開實作細節。

### 細說抽象工廠

準備好豐收融會貫通的果實了嗎？Go！

1.抽象工廠定義了需要創建的產品族，由 concreteFactory 去實作抽象工廠，所以通常抽象工廠裡的每一個創建 product 的抽象方法，都是用**工廠方法**，由繼承的具體工廠來實現，這也是這兩個名詞常被搞混的原因。
 
2 . 

![Alt text](/img/jyt0532/abstractfactory7.png)

由此圖可以看得出來，如果我們用抽象工廠來實作 pizza 店，我們只需要實作2個工廠（ChicagoPizzaIngredientFactory、NYPizzaIngredientFactory），但如果我們用工廠方法來實作，我們需要實作8個工廠。

所以當你發現你的工廠方法們，遵循著一個產品族的 pattern，試著把這產品族分離出來寫稱抽象工廠的 interface，然後用具體工廠實現。
這就是 **Design Pattern rule #2: Program to an interface, not an implementation**。

3.退化成工廠模式

如果你的抽象工廠裡定義的創建方法只有一個（只有一個產品等級結構），那你的抽象工廠就退化成工廠方法。
![Alt text](/img/jyt0532/abstractfactory5.png)
你把上圖的 Dough 改成 Pizza，就是前半段工廠方法在說的東西

4.退化成簡單工廠模式

如果你只有一個具體工廠
![Alt text](/img/jyt0532/abstractfactory9.png)
因為每個配料只有一個實作，所以沒有使用 interface 的必要，你把上圖的各個配料改成各個 pizza，就是上一篇的簡單工廠模式的例子。

### 總結

只有簡單工廠跟抽象工廠，真的需要實作工廠。

工廠方法指的是一個方法，這個方法負責創造東西，且交由子類別負責繼承。

## 延伸閱讀

1. [Design Pattern(3) - 裝飾者模式](https://www.jyt0532.com/2017/04/18/decorator/)
2. [Design Pattern(2) - 觀察者模式](https://www.jyt0532.com/2017/04/12/observer/)
3. [Design Pattern(1) - 策略模式](https://www.jyt0532.com/2017/04/07/strategy/)

關於作者：
[@jyt0532](https://www.jyt0532.com/) 後端工程師，喜歡學習新知挑戰新事物，最近覺得 Anti pattern 比 Design pattern 有趣。
