# ROS综述

## 1.什么是ROS

ROS是一个适用于机器人的开源的元操作系统。

在某些方面，ROS相当于一种**“机器人框架”**，**提供的服务**包括硬件抽象，底层设备控制，常用函数的实现，进程间消息传递，以及包的管理等。

它也提供用于获取、编译、编写、跨计算机运行代码所需的**工具和库函数**。

**ROS工具：Gazebo、RViz、rqt、rosbag、rosbridge、moveit!等。

## 2.ROS的目的

ROS的主要目标是为机器人研究和开发提供**代码复用**的支持。

ROS是一个分布式的**进程（也就是节点）框架**，这些进程被封装在易于被分享和发布的程序包和功能包中（可以通过git部署到本地）。

**在ROS上运行程序可以抽象为结点间的信息交换。

ROS也支持一种类似**代码储存库**（如github）的联合系统，这个系统也可以实现工程的协作及发布。

## 3.操作系统

ROS目前只能在基于Unix的平台上运行。

Windows端口的ROS已经实现，但并未完全开发完成。

## 4.发布版本

ROS核心系统及各种工具和库函数通常在ROS发行版本中发布。ROS发行版本提供了一系列兼容此版本的可被使用或开发的软件。常用的发行版本有melodic等。

## 5.ROS系统架构

### 5.1对ros系统的描述

##### ![img](https://upload-images.jianshu.io/upload_images/28719904-638dc114aa4f4f58.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

### 5.2ROS文件系统

主要指在硬盘里能看到的，在实际运行中起作用的，我们**主要操作的**ROS目录和文件。

### 5.3计算图

计算图是一个由ROS进程组成的点对点网络，描述了ROS运行过程中，各个ROS程序间的关系以及数据交互情况。

可以看作一种ROS文件系统运行的逻辑图。

![img](https://upload-images.jianshu.io/upload_images/28719904-505dcbd0d034a87e.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

**节点（Nodes）**：节点是一个可执行文件，它可以通过ROS来与其他节点进行通信。

**主节点/节点管理器（Master）**：ROS的命名服务，例如帮助节点发现彼此。

**话题（Topics）**：节点可以将消息发布到话题，或通过订阅话题来接收消息。

**消息（Messages）**：订阅或发布话题时所使用的ROS数据类型。

**服务（service）：**是节点之间通讯的另一种方式。服务允许节点发送一个**请求（request）**并获得一个**响应（response）**。

**参数服务器（parameter server）：**用于存储和操作数据。

ROS中提供了具象化计算图级中各种元素的工具：rqt_graph；要使用该工具，需要先安装rqt包，具体方法如下:

`sudo apt install ros-<distro>-rqt`

`sudo apt install ros-<distro>-rqt-common-plugins`

安装完成后，在ROS程序运行时执行命令：

`rosrun rqt_graph rqt_graph`

显示一张计算图。例如

![img](https://upload-images.jianshu.io/upload_images/28719904-e12cc0c8b86dbf8a.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

## 6.如何开始使用ROS

ros软件包运行需要在catkin工作空间中进行，进行ROS开发从创建