#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    prob_rob_labs_dir = get_package_share_directory('prob_rob_labs')

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(prob_rob_labs_dir, 'launch', 'turtlebot3_and_door_launch.py')
        ),
        launch_arguments={
            'world': 'door_and_firetruck.world'
        }.items()
    )

    return LaunchDescription([main_launch])
