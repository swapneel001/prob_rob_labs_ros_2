import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('prob_rob_labs')
    rviz_config = os.path.join(pkg_dir, 'config', 'ekf_slam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        Node(
            package='prob_rob_labs',
            executable='ekf_slam',
            name='ekf_slam',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='prob_rob_labs',
            executable='ekf_slam_error',
            name='ekf_slam_error',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='prob_rob_labs',
            executable='ekf_slam_tftree',
            name='ekf_slam_tftree',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        # Launch RViz after a 3 second delay
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
                )
            ]
        ),
    ])