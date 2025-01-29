from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    camera_cal_path = os.path.join(
        get_package_share_directory("image_processing"), "config", "brio_101.yaml"
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

    cube_detect_node = Node(
        package='image_processing',
        executable='cube_detect_subscriber',
        output='screen'
    )

    state_machine_node = Node(
        package = 'image_processing',
        executable = 'state_machine',
        output = 'screen'
    )

    raven_node = Node(
        package='image_processing',
        executable='raven_subscriber',
        output='screen'
    )

    return LaunchDescription([v4l2_camera_node, cube_detect_node, state_machine_node, raven_node])

# def generate_launch_description():
    

#     raven_node = Node(
#         package='image_processing',
#         executable='raven_subscriber',
#         output='screen'
#     )

#     test_elevator_node = Node(
#         package='image_processing',
#         executable='test_elevator_subscriber',
#         output='screen'
#     )

#     return LaunchDescription([test_elevator_node, raven_node])
