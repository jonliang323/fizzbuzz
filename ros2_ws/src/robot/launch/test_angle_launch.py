from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    state_machine_node = Node(
        package='robot',
        executable='state_machine',
        output='screen'
    )

    raven_node = Node(
        package='robot',
        executable='raven_subscriber',
        output='screen'
    )

    return LaunchDescription([state_machine_node, raven_node])