from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    state_machine_node = Node(
        package='robot',
        executable='imu',
        output='screen'
    )

    raven_node = Node(
        package='robot',
        executable='move',
        output='screen'
    )

    return LaunchDescription([state_machine_node, raven_node])