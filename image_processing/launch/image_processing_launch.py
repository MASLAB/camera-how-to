from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cube_detector_node = Node(
        package='image_processing',
        executable='cube_detector',
        output='screen'
    )

    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        parameters=[
            {"image_size": [640,480]},
        ]
    )

    return LaunchDescription([cube_detector_node, v4l2_camera_node])

