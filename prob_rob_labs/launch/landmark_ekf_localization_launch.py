import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('map_path', default_value=os.path.join(
            os.getcwd(), 'src', 'prob_rob_labs_ros_2', 'prob_rob_labs',
             'config', 'landmarks_map.json'),
            description='path to landmark map JSON file'),
        DeclareLaunchArgument('landmark_height', default_value='0.5',
                              description='height of the landmarks (meters)'),
        Node(
            package='prob_rob_labs',
            executable='landmark_ekf_localization',
            name='landmark_ekf_localization',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'map_path': LaunchConfiguration('map_path')},
                        {'landmark_height': LaunchConfiguration('landmark_height')}]
        )
    ])
