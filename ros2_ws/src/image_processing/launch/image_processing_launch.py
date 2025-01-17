from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cube_detect_node = Node(
        package='image_processing',
        executable='cube_detect_subscriber',
        output='screen'
    )

    # test_motor_node = Node(
    #     package='image_processing',
    #     executable='test_motor_subscriber',
    #     output='screen'
    # )

    state_machine_node = Node(
        package = 'image_processing',
        executable = 'state_machine_subscriber',
        output = 'screen'
    )

    raven_node = Node(
        package='image_processing',
        executable='raven_subscriber',
        output='screen'
    )

    # return LaunchDescription([cube_detect_node, test_motor_node, raven_node])
    return LaunchDescription([cube_detect_node, state_machine_node, raven_node])