# Copyright (c) 2025 José Miguel Guerrero Hernández
#
# This file is licensed under the terms of the MIT license.
# See the LICENSE file in the root of this repository.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='computer_vision',
            namespace='computer_vision',
            executable='cv_program',
            output='both',
            emulate_tty=True,
            # Use topics from camera
            remappings=[
                ('/camera_info', '/rgb/camera_info'),
                ('/rgb_in', '/rgb/image'),
                ('/depth_in', '/stereo/depth'),
                ('/disparity_in', '/stereo/disparity'),
                ('/left_raw_in', '/left/image'),
                ('/right_raw_in', '/right/image'),
                ('/left_rect_in', '/left_rect/image'),
                ('/right_rect_in', '/right_rect/image'),
                ('/pointcloud_in', '/stereo/points'),
            ],
        )
    ])
