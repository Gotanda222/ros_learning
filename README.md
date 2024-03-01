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

ros软件包运行需要在catkin工作空间中进行，进行ROS开发从创建Catkin工作空间开始。

# ROS（Catkin）工作空间创建

## 1.安装Catkin

`sudo apt-get install ros-<distro>-catkin`

<distro>用自己的ROS版本替换。

安装依赖包：

`sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev build-essential`

## 2.创建Catkin工作空间

```
mkdir -p ~/[workspace name]/src

cd ~/catkin_ws/src

catkin_init_workspace
```

完成名为[workspace name]的catkin工作空间的创建。

## 3.构建一个Catkin工作空间并生效配置文件

在工作空间根目录中打开终端，运行

`catkin_make`

指令进行编译，系统自动完成编译和链接过程，构建生成目标文件。

catkin_make指令成功后，紧跟source指令刷新环境，使系统能够找到刚才编译生成的ROS可执行文件。

运行setup.bash文件刷新当前catkin工作空间环境：

`source~/catkin_ws/devel/setup.bash`

保证工作空间路径被添加至环境变量中，在终端中输入：

`echo $ROS_PACKAGE_PATH`

如果显示出刚刚创建的工作空间的src文件夹，则工作区被脚本正确覆盖。

在每次运行catkin_make命令前，都需要运行source命令刷新环境变量；将source命令添加到.bashrc中，将在每次启动终端时自动添加环境变量。

## 4.catkin工作空间的结构

在工作空间根目录下用tree命令，显示文件结构

```
cd~/catkin_ws
sudo apt install tree
tree
```

通过tree命令可以看到catkin工作空间的结构,它包括了src、build、devel三个路径，在有些编译选项下也可能包括其他。但这三个文件夹是catkin编译系统默认的。它们的具体作用如下：

src/: ROS的catkin软件包（源代码包）

build/: catkin（CMake）的缓存信息和中间文件

devel/: 生成的目标文件（包括头文件，动态链接库，静态链接库，可执行文件等）、环境变量

编译过程中，他们的工作流程：

![img](https://upload-images.jianshu.io/upload_images/28719904-2e8af4259e0acd7a.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

build 目录是构建空间的默认位置，同时cmake和make也是在这里被调用来配置和构建你的软件包。而devel目录是开发空间的默认位置, 在安装软件包之前，这里可以存放可执行文件和库。

# ROS Package

## 文件系统架构

![img](https://upload-images.jianshu.io/upload_images/28719904-74413c86ccc271c5.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

ros开发是针对一个个ros软件包（package）的开发，ros软件包也是ros**文件系统**的最小单位。大部分情况下，我们只关心这些packages。

**软件包（Packages）：**包是ROS代码的软件组织单元，每个软件包都可以包含程序库、可执行文件、脚本或其他构件。

软件包中可以包含的内容：

![img](https://upload-images.jianshu.io/upload_images/28719904-cafb91d5d6f01633.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)

**Manifests** (**package.xml****)：**清单（Manifest）是对软件包的描述。它位于软件包之中，用于定义软件包之间的**依赖关系**，并记录有关软件包的元信息，如版本、维护者、许可证等。

想要在ROS环境中编译运行代码和文件，需要在[Catkin工作空间](https://www.jianshu.com/writer#/notebooks/53503383/notes/108273939)中使用到[Catkin编译系统](https://www.jianshu.com/writer#/notebooks/53503383/notes/108253873)。

一个**Catkin管理**的软件包（packages），除了package.xml文以外，还需要**CMakeList.txt**文件。

在catkin工作空间中，软件包存放在src文件夹中。

src文件夹中的源代码文件、描述性文件等，build文件夹中调用cmake的配置文件等，devel中的环境变量文件等，共同组成了ROS文件系统。

## **文件系统工具**

**添加软件包:**

在src目录下

`catkin_create_pkg <package_name> <depends>（std_msg roscpp rospy等）`

**安装软件包:**

`sudo apt install <package_name>`

注意：在创建完成每一个功能包后需要在工作空间根目录下进行编译并source setup.bash文件

**删除软件包：**

`sudo apt purge <package_name> `

**查找功能包：**

`rospack list `

列出所有功能包

`rospack find <package_name>`

查找某个功能包是否存在，如果存在返回安装路径

`roscd <package_name>`

进入某个功能包

**修改某个文件/文件夹的权限**

`sudo chmod 777 <package_name>`

## CMakeLists.txt文件

CMakeLists.txt原本是Cmake编译系统的规则文件，而Catkin编译系统基本沿用了CMake的编译风格，只是针对ROS工程添加了一些宏定义。所以在写法上，catkin的CMakeLists.txt与CMake的基本一致。

任何CMake兼容包都包含一个或多个CMakeLists.txt文件，这些文件描述了如何编译代码以及将其安装到哪里。

使用CMake编译程序时，cmake指令依据CMakeLists.txt 文件生成makefiles文件，make命令再依据makefiles文件编译链接生成可执行文件。

### 总体结构和顺序

```
cmake_minimum_required(VERSION 3.0.2)
project(lab2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
)

 add_message_files(
  FILES
  student.msg
#   Message1.msg
#   Message2.msg
 )
 generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
```

**必需的CMake版本**：

`cmake_minimum_required()`

每一个CMakeList.txt文件必须以所需的CMake版本说明语句开始，Catkin需要2.8.3或者更高的版本。

**软件包名**：

`project()`

软件包报名使用CMake的 project()函数指明，例如以beginner_tutorials命名一个软件包：project(beginner_tutorials)

**查找编译依赖的其他CMake/Catkin包（声明依赖库）**：find_package()

编译一个项目，需要使用CMake 的find_package函数确定依赖的其他CMake包并找到它们.

以catkin的组件的方式 find_package它们是有好处的，因为这个过程以catkin_prefix的形式创建了一组环境变量,如

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
)
```

**消息/服务/操作(Message/Service/Action)生成器**：

在被ROS软件包编译和使用之前，ROS中的消息（.msg）、服务（.srv）和操作（.action）文件需要特殊的预处理器编译步骤。这些宏的要点是生成编程语言特定的文件，以便可以在编程语言中使用消息、服务和操作。编译系统将使用所有可用的生成器（例如gencpp、genpy、genlisp）生成绑定。

提供了三个宏来分别处理消息，服务和操作：

```
add_message_files(),
add_service_files(),
add_action_files()
```

这些宏后面必须调用一个调用生成的宏：

`generate_messages()`

**指定包编译信息导出**：

`catkin_package()`

**添加要编译的库和可执行文件**：

```
add_executable(<node_name> src/<node_name>.cpp)
target_link_libraries(<node_name> ${catkin_LIBRARIES})
```

# ROS Node&Topic

## ROS节点 Node

在ros中，最小的进程单元就是节点node

软件包中有多个可执行文件，运行这些文件就形成了一个个进程，这些进程在ROS中叫做节点

从程序角度来说，node就是一个可执行文件（通常为C++编译生成的可执行文件、Python脚本）被执行，加载到了内存之中

从功能角度来说，通常一个node负责者机器人的某一个单独的功能。由于机器人的功能模块非常复杂，我们往往不会把所有功能都集中到一个node上，而会采用分布式的方式，把鸡蛋放到不同的篮子里

例如有一个node来控制底盘轮子的运动，有一个node驱动摄像头获取图像，有一个node驱动激光雷达，有一个node根据传感器信息进行路径规划，这样做可以降低程序发生崩溃的可能性，试想一下如果把所有功能都写到一个程序中，模块间的通信、异常处理将会很麻烦。

### 节点文件位置

一般来说.cpp文件放在功能包的src目录下，Python脚本放在scripts目录下



### ROS节点的功能

**发布（Publishing）和订阅（Subscribing）话题（Topics）**：

节点可以发布消息到话题，或订阅话题以接收消息。

这是节点间通信的主要方式，用于传输传感器数据、状态信息、控制命令等。

**提供（Providing）和使用（Using）服务（Services）**：

服务是另一种节点间通信方式，允许节点对另一个节点进行同步的请求和响应。

例如，一个节点可以请求另一个节点提供的服务来执行某项操作或计算。

**配置和使用参数（Parameters）**：

节点可以使用ROS参数服务器存储和检索配置信息。

参数可以在运行时设置和修改，允许动态调整节点行为。

**动态重配置（Dynamic Reconfigure）**：

一些节点支持动态重配置，允许在运行时改变参数而不需要重启节点。

这对于调整算法参数或系统行为特别有用。

**记录和回放（Logging and Playback）**：

节点可以使用ROS的记录功能（如rosbag）记录运行时的数据，包括话题消息和服务调用，然后在以后回放这些数据进行调试或数据分析。

**使用tf库进行坐标变换（Coordinate Transformations）**：

节点可以使用tf库管理和转换不同坐标系之间的关系，这对于机器人的导航和感知特别重要。

### 编写一个节点

#### 创建一个talker节点作为publisher

```c++
#include"ros/ros.h"
#include"std_msgs/String.h"
```

这两行是预处理指令，用于包含ROS和标准消息库的头文件。ros/ros.h是ROS的主要头文件，提供了ROS程序中常用的核心功能。std_msgs/String.h是标准消息类型String的头文件，用于节点间的字符串通信。

如果您的 Python 脚本的第一行不是 shebang（如 `#!/usr/bin/env python` 或 `#!/usr/bin/env python3`），那么操作系统可能不知道应该用 Python 解释器来执行这个脚本。确保脚本的第一行正确地指向了 Python 解释器。

```c++
int main(intargc,char**argv){
    #这行定义了主函数，它是程序的入口点。argc和argv是传递给程序的命令行参数的数量和值。
	ros::init(argc, argv,"talker");
    #这行调用ros::init()函数来初始化ROS。它接受命令行参数，并设置节点的名称，这里节点的名称被设置为"talker"。
	ros::NodeHandle n;
    #这行创建一个NodeHandle实例，是与ROS系统通信的主要接口。你可以通过这个NodeHandle实例来创建发布者、订阅者、服务等。当你创建一个ros::NodeHandle实例时，你实际上在为你的节点建立一个通信的上下文。
	ros::Publisher chatter_pub = n.advertise("chatter",1000);
    #这行创建了一个发布者chatter_pub，它可以向"chatter"话题发布std_msgs::String类型的消息。1000是发布队列的大小，它定义了在达到发送给订阅者之前，消息可以积累的数量。可看作一个缓冲池
	ros::Rateloop_rate(10);
    #这行创建了一个Rate对象，用于定义循环的频率。这里设置为10Hz，意味着循环体将尽可能保持每秒10次的执行频率。
	while(ros::ok()) {
        #这行开始了一个循环，它将持续执行，直到收到ROS的关闭信号（例如，通过Ctrl+C）。
		std_msgs::String msg;
        #在循环内部，首先创建一个String类型的消息msg。
		msg.data ="hello world";
        #这行将字符串"hello world"赋值给消息的data字段。
		chatter_pub.publish(msg);
        #这行发布消息到"chatter"话题。
		ros::spinOnce();
        #这行调用spinOnce()函数，它是非阻塞版本的spin()，允许一次调用回调函数（如果有的话）。这在需要持续执行其他操作的循环中很有用。
		loop_rate.sleep();
        #这行使得循环遵循设定的10Hz频率，如果需要，它会让当前线程睡眠以保持循环频率。
		return 0;
	}
}
```

#### 修改CMakeLists.txt

为了编译节点，需要修改CMakeLists.txt文件，告诉catkin如何构建代码

##### 在add_executable和target_link_libraries中添加节点：

`add_executable(talker src/talker.cpp)`

talker：这是生成的可执行文件的名称。

src/talker.cpp：这指定了源代码文件的位置。

` target_link_libraries(talker ${catkin_LIBRARIES})`

talker：这是你之前用add_executable命令创建的目标可执行文件的名称。在这个上下文中，它指的是你希望链接库的可执行文件。

`${catkin_LIBRARIES}：`这是一个变量，代表了所有catkin组件库的集合。当你在find_package(catkin REQUIRED COMPONENTS ...)中列出ROS组件时，catkin会将这些组件的库链接到${catkin_LIBRARIES}变量。这样，你就可以确保你的程序链接到了所有必要的ROS库。

##### 确保在编译节点之前先生成必要的消息或服务代码。

`add_dependencies(talker <package_name>_generate_messages_cpp)`

talker：这是你要编译的目标，即你的节点，该节点依赖于自定义消息或服务时，你需要确保这些消息或服务的代码在编译该节点之前已经生成。

<package_name>__generate_messages_cpp：这是一个自动生成的依赖项，它代表了你的包中定义的所有消息和服务的C++代码生成任务。这个名称遵循一个特定的格式：<package_name>_generate_messages_cpp。

##### rosnode常用名令

`rosnode list`: 列出当前活动的所有节点。

`rosnode info <node_name>`：显示关于特定节点的信息，如订阅的话题、发布的话题、服务等。

`rosnode ping <node_name>`：测试节点是否响应，以检查它是否活动。

## ROS Topic

ROS中的话题通信机制是一种基于发布/订阅模式的通信方式，使得节点能够相互独立地发送和接收消息。这种机制是ROS中实现模块化和解耦合的关键部分，允许不同的节点在没有直接知道对方存在的情况下交换数据。

话题通信使用的消息（Messages）是定义好的数据结构，用于节点之间的数据交换。消息使得发布者和订阅者之间可以共享结构化的数据，比如传感器的读数、状态信息、控制命令等。

### 发布者和订阅者

**发布者（Publisher）**：**一个节点可以成为一个或多个话题的发布者**。发布者节点向特定的话题发送消息，这些消息可以是传感器数据、状态信息、控制命令等。

**订阅者（Subscriber）**：**一个节点可以订阅一个或多个话题**。当订阅的话题有新消息时，订阅者节点会接收这些消息，并可以根据接收到的数据执行相应的处理。

### 话题通信的工作流程

**节点启动**：节点开始运行，初始化ROS客户端库。

**声明发布者和订阅者**：

发布者节点声明它将发布到哪个话题，以及使用的消息类型。

订阅者节点声明它想要订阅的话题和消息类型，以及定义一个回调函数来处理接收到的消息。

**名称解析**：ROS Master帮助节点解析话题名称，确保发布者和订阅者对应同一个话题。

**通信建立**：一旦订阅者和发布者匹配，它们之间的连接就会建立。订阅者会直接从发布者接收消息，而不需要经过中央服务器。

**消息传递**：发布者节点将消息发布到话题，所有订阅该话题的节点都将接收这些消息并调用设定的回调函数进行处理。

特点和优势

**解耦合**：发布者和订阅者不需要知道对方的存在，它们只关心消息和话题，这增强了系统的模块性和可扩展性。

**灵活性**：可以随时添加或移除发布者和订阅者，不影响其他节点。

**多对多通信**：多个发布者可以发布到同一个话题，多个订阅者可以订阅同一个话题，实现复杂的通信模式。

**异步通信**：发布者和订阅者之间的通信是异步的，允许节点独立地以自己的频率工作。

通过这种通信机制，ROS支持复杂的数据交换和节点间的协作，为构建高度复杂且互联的机器人系统提供了基础。

# ROS msg

在ROS中，消息（msg）是定义节点之间通信时传递数据的格式。消息文件（.msg）定义了数据的结构，这些数据可以是简单的数据类型（如整数、浮点数、字符串）或者更复杂的类型（包括数组和嵌套的消息）。

## 消息类型分类

### 标准消息类型

**定义位置**：标准消息类型在ROS的标准包中定义，如std_msgs、sensor_msgs、geometry_msgs等。这些消息类型已经预定义好，可直接在ROS应用中使用。

**通用性**：这些消息类型设计用于满足广泛的通用需求。例如，std_msgs包含简单的数据类型（如String、Int32、Float64等），而sensor_msgs包含用于不同传感器数据的消息类型，如Image、LaserScan等

### 自定义消息类型

**定义位置**：自定义消息类型由用户根据自己的特定需求在自己的ROS包中定义。这些消息类型在msg文件夹中定义，并需要用户指定每个消息的结构。

**特定性**：自定义消息允许用户定义包含特定数据结构的消息类型，以满足其应用程序的独特需求。当标准消息类型不足以表达特定的数据时，就需要使用自定义消息。

**构建和依赖管理**：使用自定义消息需要在CMakeLists.txt和package.xml文件中添加额外的配置，以确保消息可以被正确地生成和编译。这包括添加对message_generation和message_runtime的依赖，并确保消息和服务的文件被正确处理。

## 定义一个msg文件

#### **创建msg文件夹**

`mkdir msg`

在package根目录中创建名为msg的文件夹.

#### **创建.msg文件**

在消息定义文件中，指定消息的数据结构，每个字段一行，格式为类型 字段名。

例如，msg文件夹中创建一个名为Person.msg的消息定义如下：

```
string name
int32 age
```

定义完成后，需要在CMakeLists.txt和package.xml文件中添加相应的配置，以确保消息可以被正确地生成和使用。

#### **修改package.xml文件**

添加对message_generation和message_runtime的依赖。这确保消息生成工具可用，且生成的消息可在运行时由其他包引用。

```
 <build_depend>message_generation</build_depend>

 <exec_depend>message_runtime</exec_depend>
```

#### 修改CMakeLists.txt文件

1.找到并取消注释或添加find_package调用中的message_generation和其他任何需要的消息类型依赖。

2.在add_message_files函数中，添加你的消息文件，以确保它们被正确处理。

```
add_message_files(
  FILES
  MyCustomMessage.msg
)
```

FILES关键字后面跟着的是需要生成的自定义消息文件的列表。这些文件应该位于你的ROS包的msg目录下。每个文件定义了一个独立的消息类型，其内容描述了消息的数据结构，即消息中包含的数据字段及其类型。

3.调用generate_messages函数，确保依赖的消息类型也被包括。

```
generate_messages(
  DEPENDENCIES
  std_msgs  # 或其他依赖的消息类型
)
```

自定义消息依赖于std_msgs

（非必要）4. 在catkin_package调用中添加message_runtime，以及任何其他包依赖。

```
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
```

在CMakeLists.txt文件中的catkin_package函数用于声明一个ROS包的构建和运行时依赖关系。当你创建了自定义消息并希望这些消息**能够被其他包使用时**，需要在catkin_package调用中声明这些依赖。

## 检查创建的msg类型

`rosmsg show <package_name>/<msg_name>`

## 使用自定义的msg

在C++中，你可以创建这个消息的实例，如下：

```
#include "<package_name>/Person.h"

<package_name>::Person person_msg;

person_msg.name = "Alice";

person_msg.age = 30;
```

然后，你可以发布这个消息到一个话题，或者将其作为服务的一部分发送。

Person.h 是由ROS消息生成系统自动创建的一个头文件，它基于你在your_package包中定义的Person.msg消息文件。



# ROS 服务service & srv

## ROS服务

ROS服务是ROS中的一种通信机制，允许节点之间进行同步双向通信。与话题（Topics）不同，服务允许一个节点向另一个节点发送请求，并等待响应。这种通信方式类似于远程过程调用（Remote Procedure Call, RPC）。

在ROS节点中，你可以实现服务服务器（service server）和服务客户端（service client）。**服务服务器**会监听来自服务客户端的请求，处理它们，并返回响应。**服务客户端**发送请求给服务服务器，并等待响应。

假设你有一个服务服务器节点，它实现了AddTwoInts服务。当服务客户端调用这个服务并发送两个整数时，服务服务器会计算它们的和并返回结果。

**可以使用std_srvs包提供的标准服务或是使用srv文件自定义服务类型**。

### 使用std_srvs/SetBool服务类型实现服务

**创建服务端**

在package目录下创建scripts文件夹用于存放python脚本

在scripts文件夹中创建server.py

```python
#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 该例程将提供print_string服务，std_srvs::SetBool

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def stringCallback(req):
    if req.data:
        rospy.loginfo("Hello ROS!")
        return SetBoolResponse(True, "Print Successully")
    else:
        return SetBoolResponse(False, "Print Failed")

def string_server():
    rospy.init_node('string_server')
    s = rospy.Service('print_string', SetBool, stringCallback)
    #一个服务器对象，三个参数分别为
    #服务名称，是一个字符串，定义了服务的名称；
    #服务类型，通常是从 .srv 文件自动生成的一个 Python 类；
    #处理函数 ，一个回调函数，当服务收到请求时会被调用。这个函数应该接收一个与服务请求消息类型相对应的参数，并返回一个与服务响应消息类型相对应的对象。
    print("Ready to print hello string.")
    rospy.spin()

if __name__ == "__main__":
    string_server()
```

**创建客户端**

创建client.py

```py
#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import rospy
from std_srvs.srv import SetBool, SetBoolRequest

def string_client():
	
    rospy.init_node('string_client')	
    rospy.wait_for_service('print_string')
    try:
        string_client = rospy.ServiceProxy('print_string', SetBool)
		#创建一个代理服务器，参数分别为
        #服务名称，为字符串
        #服务类型，通常是从 .srv 文件自动生成的一个 Python 类；
        response = string_client(True)
        return response.success, response.message
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print("Response : [%s] %s" %(string_client()))
```

修改.py文件权限为可执行文件，分别运行即可实现基础服务

`chmod +x server.py`

`chmod +x client.py`

## ROS srv 自定义服务

自定义服务类型在ROS中通过.srv文件定义，这些文件存储在ROS包的srv目录下。一个.srv文件定义了服务的请求和响应的数据结构。文件分为两部分，上半部分定义请求消息，下半部分定义响应消息，两部分之间用---分隔。

### 自定义服务类型

**创建.srv文件**

例如，在srv文件夹下创建AddTwoInts.srv，具体内容如下：

```
int64 a
int64 b
---
int64 sum
```

**修改package.xml文件**

```
<build_depend>message_generation</build_depend>

<exec_depend>message_runtime</exec_depend>

<build_export_depend>message_runtime</build_export_depend>

<build_depend>std_msgs</build_depend>

<exec_depend>std_msgs</exec_depend>

<build_depend>std_srvs</build_depend>

<exec_depend>std_srvs</exec_depend>
```

**修改CMakeLists.txt文件**

1.**找到依赖包**：确保find_package调用包含了**message_generation**和**std_srvs**

```
find_package(catkin REQUIRED COMPONENTS

  rospy

  std_msgs

  message_generation

  std_srvs

)
```

2.**添加服务文件**：使用add_service_files()函数将.srv文件加入到构建过程中。

```
add_service_files(

  FILES

  YourService1.srv

  YourService2.srv

)
```

**3.生成消息和服务**：使用generate_messages()函数来生成消息和服务，确保依赖关系正确。

```
generate_messages(

  DEPENDENCIES

  std_msgs

  std_srvs  # 如果你的服务使用了std_srvs中的类型

)
```

4.**包含和库的导出**：在catkin_package()调用中，包含message_runtime以确保消息和服务的运行时依赖被考虑。

```
catkin_package(

  CATKIN_DEPENDS message_runtime std_msgs std_srvs

)
```

创建完成srv文件后需要进行编译。

### 检查创建的srv类型

`rossrv show <package_name>/<srv_name>`

### 使用srv

向服务客户端发送一个包含特定 `a` 和 `b` 值的请求时，服务服务器会接收到这个请求，并在回调函数中对这两个值进行操作。在这个例子中，操作是将它们相加。

#### 创建服务端

在scripts文件夹下新建名为add_server.py的脚本。

在服务端中我们要**定义服务的名称**以及**服务响应的回调函数**。

```python
#!/usr/bin/python3
# -*- coding: utf-8 -*-
#告诉操作系统用 Python 解释器来执行这个脚本。采用utf-8编码
import rospy
from lab2.srv import AddTwoInts, AddTwoIntsResponse


def addCallBack(req):
    #服务回调函数，其中的参数 req 代表服务的请求消息。当一个服务客户端向这个服务发送请求时，req 会包含客户端发送的所有数据。
    sum = req.a + req.b
    return AddTwoIntsResponse(sum)

def add_server():
    rospy.init_node("add_server")
    sum = rospy.Service('add_two_ints',AddTwoInts,addCallBack)
    print("Ready to print sum.")
    rospy.spin()

if __name__ == "__main__":
    add_server()

```

#### 创建客户端

在scripts文件夹下新建名为add_client.py的脚本。、

在客户端需要声明服务代理

```python
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import sys
from lab2.srv import AddTwoInts

def add_client():
    rospy.init_node('add_client')
    rospy.wait_for_service('/add_two_ints')
    try:
        add_client = rospy.ServiceProxy('/add_two_ints', AddTwoInts)
        #系统代理对象，这个代理对象允许节点向服务器发送请求；参数为服务名称以及服务类型。
        a = int(sys.argv[1])
        b = int(sys.argv[2])
        #读取输入的两个参数，sys.argv：这是一个命令行参数的列表，sys.argv[0] 是脚本的名字，sys.argv[1] 是第一个参数，依此类推。
        response = add_client(a,b)
        #add_client被调用时，会向服务器发送a,b两个参数，并返回服务器相应的结果。
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print("Show sum:\n%s" %(add_client()))
```

在运行节点前，修改.py文件权限为可执行脚本。