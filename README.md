

# 1 .Code Structure

├── firmware: stm32固件，根据IRbot2022电控组云台框架进行构建 https://github.com/Qylann/gimbal-standard  \
└── src \
&emsp;&emsp;├── skider_interface: skider自定义通信接口  \
&emsp;&emsp;├── skider_hw: 底层通信包，存放底层接口通信节点 \
&emsp;&emsp;├── skider_sensor: 传感器处理包，存放传感器的数据处理节点   \
&emsp;&emsp;└── skider_control: 控制包，存放了一个云台控制节点和一个底盘控制节点  \




# 2 .Dependencies
``` 
-ros2=galactic(can not be foxy!)
-libusb=1.0.26(can be other version)
-rt kernel(eg. 5.15.133-rt69)
-Eigen3
```



``` 
# setup libusb
https://github.com/libusb/libusb/releases/download/v1.0.26/libusb-1.0.26.tar.bz2 
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

related links: https://blog.csdn.net/jiacong_wang/article/details/106720863?spm=1001.2014.3001.5502



to setup the realtime kernel, please check this article https://zhuanlan.zhihu.com/p/675155576



# 3 .Check the hardware
## usb communication

usb通信接受从C板传来的imu数据及遥控器数据

在launch skider_hw之前需要赋权usb。 

```
lsusb	#查看usb设备
```
找到stm32 usb虚拟串口，假设为
```
Bus 003 Device 009: ID 0483:5740 STMicroelectronics Virtual COM Port
```
一般Bus id不会改变，而Device id在每次重新插入后都会发生变化 ,所以一般这样赋权此usb端口
```
sudo chmod 777 /dev/bus/usb/003/*
# or 
./sh/usb.sh
```


## can communication

can通信分别控制底盘电机，云台电机，

``` 
./sh/can.sh
# 检查can0 can1顺序
candump can0	#底盘
candump can1	#云台
```

## remote

the remote buttons' layout are as follows

``` 
remote:
buttons[5]---     ---buttons[2]
buttons[4]---     ---buttons[1]
buttons[3]---     ---buttons[0]
```

## chassis wheels

the chassis wheels' layout are as follows

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
    
	can id = 200 + index
```

## gimbal

``` 
#define AMMOR 0x201
#define AMMOL 0x202
#define ROTOR 0x203
#define YAW 0x205 
#define PITCH 0x206
```



# 4.Build and Launch

注意执行单步构建时skider_hw、skider_sensor、skider_control依赖于skider_interface

然后依次launch skider_hw、skider_sensor、skider_control

``` 
colcon build

source install/setup.bash

ros2 launch skider_hw skider_hw.launch.py

ros2 launch skider_sensor skider_sensor.launch.py

ros2 launch skider_control skider_control.launch.py
```



# 5.Useful commands

``` 
./sh/control

./sh/kill_control
```







出现其他问题可优先检查：
检查sh文件内控制路径、自瞄路径及nuc密码
检查参数文件是否正确读取
检查can0 can1顺序
检查gimbal_demo_node.hpp包含自瞄代码自定义target.msg路径



----------------------------------------------------------------------------------------------------------------------------------



# Docker based installation

## Method1: pull from ghcr



``` 
docker pull ghcr.io/shitoujie/vision_control:latest
docker run -it --name vc_devel \
--privileged --network host \
-v /dev:/dev \
ghcr/shitoujie/vision_control \
```

考虑到上述方法速度极慢，因此使用国内的镜像源进行docker pull

``` 
# 在配置文件 /etc/docker/daemon.json 中加入：
{
"registry-mirrors":["https://docker.nju.edu.cn/"]
}

```

``` 
sudo systemctl restart docker.service # 重新启动 docker

```

执行`docker info`，如果从输出中看到如下内容，说明配置成功。

``` 
Registry Mirrors:
 https://docker.nju.edu.cn/
```

``` 
# 现在可以使用以下命令来拉取镜像
docker pull ghcr.nju.edu.cn/shitoujie/vision_control:latest
docker run -it --name vc_devel \
--privileged --network host \
-v /dev:/dev \
ghcr.nju.edu.cn/shitoujie/vision_control \
```



related links:**https://www.cnblogs.com/rainbow-tan/p/17775385.html**



## Method2: build from Dockerfile

``` 
cd docker

docker build -t vc_image .
docker run -it --name vc_devel \
--privileged --network host \
-v /dev:/dev \
vc_image \
```



## Usefel commands

``` 
# use the following command to add more terminals
docker exec -it vc_devel bash
```



