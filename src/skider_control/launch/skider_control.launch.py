import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node

SCRIPT_DIR = os.path.split(os.path.realpath(__file__))[0]               # 获取这个脚本所在目录
PACKAGE_DIR = os.path.join(SCRIPT_DIR, os.pardir)                       # 向上两级，到skider_control/
PARAM_CONFIG_PATH = os.path.join(PACKAGE_DIR, 'config', 'params.yaml')  # params.yaml的路径

def generate_launch_description():
    # 检查参数配置文件是否存在，不存在就退出
    if not os.path.exists(PARAM_CONFIG_PATH):
        print("[ERROR][skider_control.launch.py]: config/params.yaml does not exist")
        sys.exit(1)
        
    return LaunchDescription(
        [
            Node(
                package='skider_control',
                executable='gimbal_demo_node',
                name='gimbal_demo_node',
                output='screen',
                parameters=[PARAM_CONFIG_PATH],
                
            ),
            Node(
                package='skider_control',
                executable='chassis_demo_node',
                name='chassis_demo_node',
                output='screen',
                parameters=[PARAM_CONFIG_PATH],
                
            ),
        ]
    )