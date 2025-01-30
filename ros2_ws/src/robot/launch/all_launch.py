import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_cal_path = os.path.join(
        get_package_share_directory("robot"), "config", "brio_101.yaml"
    )

    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        parameters=[
            {
                "image_size": [640, 480],
                "camera_info_url": "file://" + camera_cal_path,
                "automatic_white_balance": False,
                "auto_exposure": 1,
            },
        ],
    )

    vision_node = Node(
        package='robot',
        executable='vision',
        output='screen'
    )

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

    plan_node = Node(
        package='robot',
        executable='plan',
        output='screen'
    )

    return LaunchDescription([v4l2_camera_node, vision_node, imu_node, move_node, plan_node])
