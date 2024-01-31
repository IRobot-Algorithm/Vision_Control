import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='skider_hw',
                executable='gimbal_hw_node',
                name='gimbal_hw_node',
                output='screen',
                parameters=[os.path.join(get_package_share_directory('skider_hw'), 'config', 'hardware_settings.yaml')],
                # parameters=["/home/qylann/worksapce/skider/src/skider_hw/config/hardware_settings.yaml"],
            )
        ]
    )
