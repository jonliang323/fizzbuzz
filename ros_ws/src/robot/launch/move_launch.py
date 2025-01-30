from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_node = Node(
        package='robot',
        executable='imu',
        output='screen'
    )

    move_node = Node(
        package='robot',
        executable='move',
        output='screen'
    )

    return LaunchDescription([imu_node, move_node])