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