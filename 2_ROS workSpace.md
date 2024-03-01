# ROS workSpace（Catkin）工作空间创建

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