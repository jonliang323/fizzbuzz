from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pic_node = Node(
        package='image_processing',
        executable='pic_subscriber',
        output='screen'
    )

    return LaunchDescription([pic_node])