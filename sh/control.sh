#!/bin/bash

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/.."
PASSWORD="nuc"

echo $PASSWORD | sudo -S ifconfig can0 down
echo $PASSWORD | sudo -S ip link set can0 type can bitrate 1000000
echo $PASSWORD | sudo -S ifconfig can1 down
echo $PASSWORD | sudo -S ip link set can1 type can bitrate 1000000
echo $PASSWORD | sudo -S ifconfig can0 up
echo $PASSWORD | sudo -S ifconfig can1 up

# 经过五次开机测试 STMicroelectronics Virtual COM Port有四次在/dev/bus/usb/003/* 有一次在/dev/bus/usb/001/*
echo $PASSWORD | sudo -S chmod 777 /dev/bus/usb/003/*
echo $PASSWORD | sudo -S chmod 777 /dev/bus/usb/001/*

# 复制配置文件到install目录
echo $PASSWORD | sudo chmod +x $PROJECT_DIR/sh/compare_and_cp.py
$PROJECT_DIR/sh/compare_and_cp.py $PROJECT_DIR/src/skider_control/config/params.yaml $PROJECT_DIR/install/skider_control/share/skider_control/config/params.yaml
$PROJECT_DIR/sh/compare_and_cp.py $PROJECT_DIR/src/skider_hw/config/hardware_settings.yaml $PROJECT_DIR/install/skider_sensor/share/skider_hw/config/hardware_settings.yaml
$PROJECT_DIR/sh/compare_and_cp.py $PROJECT_DIR/src/skider_sensor/config/sensor_settings.yaml $PROJECT_DIR/install/skider_sensor/share/skider_sensor/config/sensor_settings.yaml

# ip -details link show can0
bash -c "source /opt/ros/galactic/setup.bash;source $PROJECT_DIR/install/setup.bash;cd $PROJECT_DIR;ros2 launch skider_hw skider_hw.launch.py" &
bash -c "source /opt/ros/galactic/setup.bash;source $PROJECT_DIR/install/setup.bash;cd $PROJECT_DIR;ros2 launch skider_sensor skider_sensor.launch.py" &
bash -c "source /opt/ros/galactic/setup.bash;source $PROJECT_DIR/install/setup.bash;cd $PROJECT_DIR;ros2 launch skider_control skider_control.launch.py" &
