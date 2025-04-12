from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare a launch argument for the image topic
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Topic to which the video processor subscribes'
    )
    goodfeature_image_topic_arg = DeclareLaunchArgument(
        'goodfeature_image_topic',
        default_value='/goodfeature/image_raw',
        description='Visualization topic for goodfeature detector'
    )
    goodfeature_corners_topic_arg = DeclareLaunchArgument(
        'goodfeature_corners_topic',
        default_value='/goodfeature/corners',
        description='Output topic for goodfeature detector'
    )
    max_features_arg = DeclareLaunchArgument(
        'max_features',
        default_value='50',
        description='Maximum number of features to extract'
    )
    run_color_filter_arg = DeclareLaunchArgument(
        'run_color_filter',
        default_value='false',
        description='Set to true to run the color filter first'
    )

    return LaunchDescription([
        # Add the launch argument
        image_topic_arg,
        goodfeature_image_topic_arg,
        goodfeature_corners_topic_arg,
        max_features_arg,
        run_color_filter_arg,
        # Node configuration with the image_topic parameter
        Node(
            package='prob_rob_vision',
            executable='video_processor',
            name='video_processor',
            output='screen',
            parameters=[
                {'image_topic': LaunchConfiguration('image_topic')},
                {'goodfeature_image_topic': LaunchConfiguration('goodfeature_image_topic')},
                {'goodfeature_corners_topic': LaunchConfiguration('goodfeature_corners_topic')},
                {'max_features': LaunchConfiguration('max_features')},
                {'run_color_filter': LaunchConfiguration('run_color_filter')}
            ]
        )
    ])
