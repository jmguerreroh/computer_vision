# Copyright (c) 2023 José Miguel Guerrero Hernández
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
            # Set to True to process just if there is a subscription,
            # False to process always
            parameters=[
                {"check_subscription_count": False}
            ],
            # Use topics from robot
            remappings=[
                ('/camera_info', '/head_front_camera/rgb/camera_info'),
                ('/image_rgb_in', '/head_front_camera/rgb/image_raw'),
                ('/image_depth_in', '/head_front_camera/depth_registered/image_raw'),
                ('/pointcloud_in', '/head_front_camera/depth_registered/points'),
            ],
        )
    ])
