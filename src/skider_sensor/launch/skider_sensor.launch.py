import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PARAM_CONFIG_PATH = os.path.join(get_package_share_directory('skider_sensor'), 'config', 'sensor_settings.yaml')

def generate_launch_description():
    # 检查参数配置文件是否存在，不存在就退出
    if not os.path.exists(PARAM_CONFIG_PATH):
        print(f"[ERROR][{__file__}]: config/sensor_settings.yaml does not exist")
        sys.exit(1)
    
    return LaunchDescription(
        [
            Node(
                package='skider_sensor',
                executable='remote_sensor_node',
                name='remote_sensor_node',
                output='screen',
                parameters=[PARAM_CONFIG_PATH],
            ),
            Node(
                package='skider_sensor',
                executable='imu_sensor_node',
                name='imu_sensor_node',
                output='screen',
                parameters=[PARAM_CONFIG_PATH],
            )
        ]
    )
