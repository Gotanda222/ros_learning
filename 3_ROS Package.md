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

# 