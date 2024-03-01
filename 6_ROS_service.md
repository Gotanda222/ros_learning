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