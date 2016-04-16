---
title: 用 DDS 開發 ROS 2.0
date: 2016-04-15 22:18:57
tags: 機器人, Robot, ROS, DDS
author: pojenlai
---

## 前言

這篇文章想要向大家介紹ROS 2.0的底層實作概念，雖然比較不會有程式實作的討論，但我覺得這一塊的深度滿值得介紹的，因為使用ROS有好幾種層次:

1. 使用ROS的各種工具來建立自己的應用
2. 在開發上碰到一些問題，修改現成的package來滿足自己的需求
3. 開發自己的演算法，發布自己的package給其他人使用
4. 協助開發與維護ROS的核心

這篇文章要討論的議題已經落在第四個層次，所以對於一般的使用者來說，可能不太具有直接應用的價值，但如果對於ROS的底層實作有更深入的理解(知道他是怎麼開發出來的、有哪些限制、有哪些優點)，就可以在利用ROS撰寫自己的應用時，更能開發出效能最佳化的應用。

## 為什麼要開發ROS 2.0?

事實上，如果ROS 1.0 已經足夠完美，那我們就沒有必要討論ROS 2.0。不過事情當然不是這樣，因為ROS 1.0在開發的時候，是圍繞著一隻機器人來開發的，雖然當初的設計已經讓ROS變得很有彈性，可以被應用在各式各樣的機器人上，但是隨著使用者越來越多，超乎開發者想像的使用情境也越來越多。

也就是說，如果開發者們不積極地開始開發下一代的ROS，遲早會無法滿足越來越複雜而多樣化的需求。對於這些使用情境的具體描述，可以參考[這裡](http://po-jen.github.io/design/articles/why_ros2.html#section-1)。

## 開發ROS系統需要實現的模塊

首先來談論一下建立整個系統上，需要考慮的幾個重點:

- Discovery功能
- Publisher-Subscriber功能
- Service 與 Action功能

Discovery功能的意義是，只要有新的node啟動，就能在整個ROS node的網路中被看見(概念很像是我打開手機的wifi熱點分享，其他裝置就應該要可以發現有這個wifi熱點)。

接下來的Publisher-Subscriber功能、Service功能跟Action功能其實就是ROS使用者熟悉的Topic、Service跟Actionlib啦，本質上這幾種功能在處理的都是node之間的溝通(也就是程式之間的溝通，大家可以想像要讓一隻機器人正常運作，電腦上需要運行的程式一定是很多個，而且需要彼此溝通，所以底層的溝通機制需要有人來實作，不然就是…想開發機器人程式的你得自己實作)。如果你不太確定自己知不知道我在說什麼，可以看看這一篇[區分Topic、Service跟Actionlib的文章](https://pojenlai.wordpress.com/2012/11/03/ros-topic-service-and-actionlib/)。

## DDS的系統層概念

想要實作上面這些功能，DDS並不是唯一的選擇，但是，OSRF(Open Source Robotic Foundation)的開發者經過嘗試之後，覺得這是最好的開發選項。細節理由可以看延伸閱讀的第3篇文章，這部分已經有中文翻譯了。

![api_levels](/img/pojenlai/api_levels.png)

從上面這張圖可以清楚地看出，使用者所需要接觸到的只有最上面的兩層。使用者自己寫的code就屬於Userland Code，而使用者自己寫的code中呼叫到的ROS API (例如ros::init())就屬於ROS client library API，而DDS的API則是在更底層被ROS client library API所使用。

有趣的地方是，為了保持彈性，OSRF的開發者們希望使用者可以自己選擇底層使用的是哪一個版本的DDS (DDS像是一種標準，所以可以有不同公司提供自己的實作版本)。

## 一點細節的延伸

上面討論的都是概念的理解，對於技術有興趣的你想必沒辦法接受，所以就讓我們來看一點技術細節吧!

我們還是一樣先站在開發者的角度，最基本我們需要提供的工具就是Node初始化的函式對吧，這個函式的長相就像:

```cpp
Node::Node(std::string name): running_(true)
{
	/*----------------------親切的中文註解來囉!!!----------------------*/
	nodes_.push_back(this);
	subscription_iterator_ = subscriptions_.end();
	name_ = name;
	//取得了DDS的DomainParticipantFactory的instance，很像是一個node產生器的感覺
	dpf_ = DDS::DomainParticipantFactory::get_instance();
	checkHandle(dpf_.in(), "DDS::DomainParticipantFactory::get_instance");
	DDS::DomainId_t domain = DDS::DOMAIN_ID_DEFAULT;
 
	//實際產生一個participant，應該就是一個node
	participant_ = create_participant( domain, PARTICIPANT_QOS_DEFAULT, NULL,DDS::STATUS_MASK_NONE);
	checkHandle(participant_.in(), "DDS::DomainParticipantFactory::create_participant");
	/*----------------------看到這裡就好囉!!!----------------------*/
 
	// Create the default QoS for Topics
	DDS::ReturnCode_t status = participant_get_default_topic_qos(default_topic_qos_);
	checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
	default_topic_qos_.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;
 
	// Create the default QoS for Publishers
	status = participant_get_default_publisher_qos(default_publisher_qos_);
	checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
	default_publisher_qos_.partition.name.length(1);
	default_publisher_qos_.partition.name[0] = "ros_partition";
 
	// Create the default QoS for Subscribers
	status = participant_get_default_subscriber_qos(default_subscriber_qos_);
	checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
	default_subscriber_qos_.partition.name.length(1);
	default_subscriber_qos_.partition.name[0] = "ros_partition";
 
	// Create a waitset for spin
	waitset_ = new DDS::WaitSet();

	// Create a parameter server for this node
	create_parameter_server(name);
}
```

大家先不要嚇到，一下有太多細節本來就不可能看懂，大家只需要看我用註解標記起來的區域，體驗一下什麼叫做ROS client library API呼叫DDS API的感覺就好。

那對於一個使用者來說，假設我今天要啟動一隻機器人，那就需要呼叫建立node的函式，看起來就像:

```cpp
TurtleApp(int& argc, char** argv): QApplication(argc, argv)
{
	rclcpp::init(argc, argv);
	nh_ = rclcpp::create_node("turtlesim");
}
```

你一定覺得奇怪，rclcpp::create_node()跟上面我講的Node::Node()建構子根本接不起來啊? 所以這邊就要補上一點點程式碼，想必你就懂了:

```cpp
void rclcpp::init(int argc, char** argv)
{
	if (globally_initialized)
	{
		throw AlreadyInitializedError();
	}
	
	/* Register a signal handler so DDS doesn not just sit there... */
	if (signal(SIGINT, Node::static_signal_handler) == SIG_ERR)
	{
		fputs("An error occurred while setting a signal handler.\n", stderr);
	}
	globally_initialized = true;
}
 
Node::Ptr rclcpp::create_node(const std::string &name)
{
	return Node::Ptr(new Node(name));
}
```

## 總結

OK!簡介就到這邊啦，如果對於實作細節有興趣深入的讀者，不妨去看看ROS 2.0的github repo，詳細的程式碼全部都是開源的，所以可以從中學習開發的細節。

## 延伸閱讀
1. [為什麼要開發ROS 2.0?](http://po-jen.github.io/design/articles/why_ros2.html)
2. [使用ZeroMQ跟相關的函式庫來開發ROS](http://po-jen.github.io/design/articles/ros_with_zeromq.html)
3. [使用DDS來開發ROS](http://po-jen.github.io/design/articles/ros_on_dds.html) (仍在趕工中，歡迎開issue催促翻譯者QQ)
4. [ROS 2.0 wiki](https://github.com/ros2/ros2/wiki)
5. [ROS DDS Prototype (Github Repo)](https://github.com/osrf/ros_dds/tree/master/prototype)


關於作者：
[@pojenlai](https://pojenlai.wordpress.com/) 演算法工程師，對機器人跟電腦視覺有少許研究，最近在鍛鍊自己的執行力
