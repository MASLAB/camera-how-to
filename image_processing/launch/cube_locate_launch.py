from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cube_locater_node = Node(
        package="image_processing", executable="cube_locater", output="screen"
    )

    # camera_cal_path = os.path.join(
    #     get_package_share_directory("image_processing"), "config", "brio_101.yaml"
    # )
    # v4l2_camera_node = Node(
    #     package="v4l2_camera",
    #     executable="v4l2_camera_node",
    #     parameters=[
    #         {
    #             "image_size": [640, 480],
    #             "camera_info_url": "file://" + camera_cal_path,
    #         },
    #     ],
    # )

    # return LaunchDescription([cube_locater_node, v4l2_camera_node])
    return LaunchDescription([cube_locater_node])
