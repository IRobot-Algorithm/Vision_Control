import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='skider_sensor',
                executable='remote_sensor_node',
                name='remote_sensor_node',
                output='screen',
                parameters=[os.path.join(get_package_share_directory('skider_sensor'), 'config', 'sensor_settings.yaml')],
                # parameters=["/home/qylann/worksapce/skider/src/skider_sensor/config/sensor_settings.yaml"],
            ),
            Node(
                package='skider_sensor',
                executable='imu_sensor_node',
                name='imu_sensor_node',
                output='screen',
                parameters=[os.path.join(get_package_share_directory('skider_sensor'), 'config', 'sensor_settings.yaml')],
                # parameters=["/home/qylann/worksapce/skider/src/skider_sensor/config/sensor_settings.yaml"],
            )
        ]
    )
