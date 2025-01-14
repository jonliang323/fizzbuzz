from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cube_detect_node = Node(
        package='image_processing',
        executable='cube_detect_subscriber',
        output='screen'
    )

    state_machine_node = Node(
        package='image_processing',
        executable='state_machine_subscriber',
        output='screen'
    )

    raven_node = Node(
        package='image_processing',
        executable='racen_subscriber',
        output='screen'
    )

    return LaunchDescription([cube_detect_node, state_machine_node, raven_node])