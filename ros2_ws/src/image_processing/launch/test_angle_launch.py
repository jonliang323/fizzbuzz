from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mock_state_machine_node = Node(
        package='image_processing',
        executable='mock_state_machine',
        output='screen'
    )

    raven_node = Node(
        package='image_processing',
        executable='raven_subscriber',
        output='screen'
    )

    return LaunchDescription([mock_state_machine_node, raven_node])