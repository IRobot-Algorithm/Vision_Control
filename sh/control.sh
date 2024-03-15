#!/bin/bash

PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/.."

echo "nuc" | sudo -S ifconfig can0 down
echo "nuc" | sudo -S ip link set can0 type can bitrate 1000000
echo "nuc" | sudo -S ifconfig can1 down
echo "nuc" | sudo -S ip link set can1 type can bitrate 1000000
echo "nuc" | sudo -S ifconfig can0 up
echo "nuc" | sudo -S ifconfig can1 up

# 经过五次开机测试 STMicroelectronics Virtual COM Port有四次在/dev/bus/usb/003/* 有一次在/dev/bus/usb/001/*
echo "nuc" | sudo -S chmod 777 /dev/bus/usb/003/*
echo "nuc" | sudo -S chmod 777 /dev/bus/usb/001/*

# ip -details link show can0
bash -c "source /opt/ros/galactic/setup.bash;source $PROJECT_DIR/install/setup.bash;cd $PROJECT_DIR;ros2 launch skider_hw skider_hw.launch.py" &
bash -c "source /opt/ros/galactic/setup.bash;source $PROJECT_DIR/install/setup.bash;cd $PROJECT_DIR;ros2 launch skider_sensor skider_sensor.launch.py" &
bash -c "source /opt/ros/galactic/setup.bash;source $PROJECT_DIR/install/setup.bash;cd $PROJECT_DIR;ros2 launch skider_control skider_control.launch.py" &