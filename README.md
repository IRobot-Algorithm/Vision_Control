
[![clang-format Check](https://github.com/lunarifish/Vision_Control/actions/workflows/style_check.yaml/badge.svg)](https://github.com/lunarifish/Vision_Control/actions/workflows/style_check.yaml)
[![CodeFactor](https://www.codefactor.io/repository/github/lunarifish/vision_control/badge)](https://www.codefactor.io/repository/github/lunarifish/vision_control)

# Code Structure

├── firmware: stm32固件，根据[IRbot2022电控组云台框架]( https://github.com/Qylann/gimbal-standard)进行构建  \

└──docker \
&emsp;&emsp;├── deploy: 存放部署时使用的Dockerfile \
&emsp;&emsp;├── develop: 存放开发时使用的Dockerfile \

└──sh 存放常用脚本\

└── src \
&emsp;&emsp;├── skider_interface: skider自定义通信接口  \
&emsp;&emsp;├── skider_hw: 底层通信包，存放底层接口通信节点 \
&emsp;&emsp;├── skider_sensor: 传感器处理包，存放传感器的数据处理节点   \
&emsp;&emsp;└── skider_control: 控制包，存放了一个云台控制节点和一个底盘控制节点  \

# Dependencies

```
-ros2=galactic(can not be foxy!)
-libusb=1.0.26(can be other version)
-rt kernel(eg. 5.15.133-rt69)
-Eigen3
```

```shell
# setup libusb
wget https://github.com/libusb/libusb/releases/download/v1.0.26/libusb-1.0.26.tar.bz2 
tar -jxvf libusb-1.0.26.tar.bz2 
sudo apt update 
sudo apt-get install libudev-dev
cd libusb-1.0.26
sudo ./configure 
sudo make install

# check libusb
cd example
sudo make listdevs
sudo ./listdevs

# 出现类似以下内容即为成功
1d6b:0003 (bus 4, device 1)
8087:0033 (bus 3, device 3) path: 10
0483:5740 (bus 3, device 2) path: 1
1d6b:0002 (bus 3, device 1)
1d6b:0003 (bus 2, device 1)
1d6b:0002 (bus 1, device 1)
```

[related links]( https://blog.csdn.net/jiacong_wang/article/details/106720863?spm=1001.2014.3001.5502)

to setup the realtime kernel, please check this [article]( https://zhuanlan.zhihu.com/p/675155576)

# Check the hardware

## usb communication

usb通信接受从C板传来的imu数据及遥控器数据

在launch skider_hw之前需要赋权usb。

```shell
lsusb   #查看usb设备
```

找到stm32 usb虚拟串口，假设为

```
Bus 003 Device 009: ID 0483:5740 STMicroelectronics Virtual COM Port
```

一般Bus id不会改变，而Device id在每次重新插入后都会发生变化 ,所以一般这样赋权此usb端口

```shell
sudo chmod 777 /dev/bus/usb/003/*
# or 
./sh/usb.sh
```

## can communication

两条can总线分别挂载了底盘电机和云台电机。

> NOTE: 测试用的车的CAN1的HL线颜色反了，红色是L黑色是H

``` shell
./sh/can.sh
# 检查can0 can1顺序
candump can0    #底盘
candump can1    #云台
```

## remote

遥控器上的两个拨杆布局如下：

```
remote:
buttons[5]---     ---buttons[2]
buttons[4]---     ---buttons[1]
buttons[3]---     ---buttons[0]
```

左右拨杆分别拨到上中下某一位置时对应数组元素置1，其余为0

两个摇杆布局如下：

```
     axis[3]                axis[1]
     ^                      ^
     |                      |
<--------->axis[2]     <--------->axis[0]
     |                      |
     v                      v
```

```axis[4]```是遥控器左上角的轮子，向左转是正向右转是负

## chassis wheels

底盘轮子电机布局如下：

```
    //2-----battary-----1
    //|                 |
    //|                 |
    //|                 |
    //|                 |
    //|                 |
    //|                 |
    //|                 |
    //3-----------------4
```
can 反馈报文 id = 200 + index

## gimbal

各电机的RX标准帧ID：

```c
#define AMMOR 0x201
#define AMMOL 0x202
#define ROTOR 0x203
#define YAW   0x205 
#define PITCH 0x206
```

# 4.Build and Launch

注意执行单步构建时skider_hw、skider_sensor、skider_control依赖于skider_interface

然后依次launch skider_hw、skider_sensor、skider_control

```shell
# [可选]清理构建残留
cd sh
./clean.sh
###########

colcon build
source install/setup.bash

######
ros2 launch skider_hw skider_hw.launch.py
ros2 launch skider_sensor skider_sensor.launch.py
ros2 launch skider_control skider_control.launch.py
######
# 或者
######
sh/control.sh
######
```

# 5.Useful commands

```shell
./sh/control.sh

./sh/kill_control.sh

candump can1 | grep 1FF

candump can0 | grep 2004
```

出现其他问题可优先检查：

- sh文件内控制路径、自瞄路径及nuc密码
- 参数文件是否正确读取
- can0 can1有没有弄反
- gimbal_demo_node.hpp包含自瞄代码自定义target.msg路径

----------------------------------------------------------------------------------------------------------------------------------

# Docker based installation

## Method1: pull from ghcr

```shell
docker pull ghcr.io/shitoujie/vision_control:latest
docker run -it --name vc_devel \
--privileged --network host \
-v /dev:/dev \
ghcr/shitoujie/vision_control \
```

考虑到上述方法连国内网络时速度极慢，因此可以参考[这篇文章](**https://www.cnblogs.com/rainbow-tan/p/17775385.html**)使用国内的镜像源进行docker pull

在配置文件 /etc/docker/daemon.json 中加入：

```json
{
"registry-mirrors":["https://docker.nju.edu.cn/"]
}
```

```shell
sudo systemctl restart docker.service # 重新启动 docker
```

执行`docker info`，如果从输出中看到如下内容，说明配置成功。

```
Registry Mirrors:
 https://docker.nju.edu.cn/
```

现在可以使用以下命令来拉取镜像:

```shell
docker pull ghcr.nju.edu.cn/shitoujie/vision_control:latest
docker run -it --name vc_devel \
--privileged --network host \
-v /dev:/dev \
ghcr.nju.edu.cn/shitoujie/vision_control \
```

## Method2: build from Dockerfile

```shell
# 1.develop（开发）
cd docker/develop

docker build -t vc_develop_image .
docker run -it --name vc_develop \
--privileged --network host \
-v /dev:/dev -v /home/oem/ros_ws:/ros_ws \
vc_develop_image \

cd /ros_ws/Vision_Control
colcon build
# 启动之前先检查launch文件内的参数路径

# 2.deploy（部署）
cd docker/deploy

docker build -t vc_delpoy_image .
docker run -it --name vc_deploy \
--privileged --network host \
-v /dev:/dev -v /home/oem/ros_ws/src:/ros_ws/src \
vc_delpoy_image \

docker build -t vc_image1 .
docker run -it --name vc_devel1 \
--privileged --network host \
-v /dev:/dev -v /home/oem/ros_ws/src:/ros_ws/src \
vc_image1 \
```

注意运行开发的镜像时，我们使用`-v /home/oem/ros_ws:/ros_ws`将宿主机`/home/oem/ros_ws`下的内容映射到docker容器内`/ros_ws`处，因此我们只需修改`/home/oem/ros_ws`下的源码、参数，docker内的也会对应更改。

因此此处要**注意**将`/home/oem/ros_ws`**改为**你对应的路径。

开发完毕后将宿主机下的代码提交github即可

## Useful commands

```shell
# use the following command to add more terminals
docker exec -it vc_devel bash
```
