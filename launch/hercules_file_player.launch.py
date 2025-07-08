import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('driver', default_value='hercules_file_player', description='Driver package'),
        DeclareLaunchArgument('output', default_value='screen', description='Output type'),
        DeclareLaunchArgument('camera', default_value='stereo', description='Camera name'),
        DeclareLaunchArgument('frame_id', default_value=LaunchConfiguration('camera'), description='Frame ID'),
        DeclareLaunchArgument('proc', default_value='false', description='Enable stereo_image_proc'),
        DeclareLaunchArgument('enable_aeva', default_value='true', description='Enable aeva topic playback'),
        DeclareLaunchArgument('enable_continental', default_value='false', description='Enable continental topic playback'),
        DeclareLaunchArgument('enable_radarpolar', default_value='false', description='Enable radarpolar topic playback'),
        DeclareLaunchArgument('enable_stereo_right', default_value='false', description='Enable stereo_right topic playback'),
        DeclareLaunchArgument('enable_stereo_left', default_value='false', description='Enable stereo_left topic playback'),

        Node(
            package=LaunchConfiguration('driver'),
            executable=LaunchConfiguration('driver'),
            name=LaunchConfiguration('driver'),
            output=LaunchConfiguration('output')
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('proc')),
            package='stereo_image_proc',
            executable='stereo_image_proc',
            name='stereo_image_proc',
            namespace=LaunchConfiguration('camera'),
            parameters=[{
                'stereo_algorithm': 0,
                'prefilter_size': 255,
                'prefilter_cap': 63,
                'correlation_window_size': 11,
                'min_disparity': -67,
                'disparity_range': 176,
                'uniqueness_ratio': 3.0,
                'texture_threshold': 148,
                'speckle_size': 560,
                'speckle_range': 19,
                'fullDP': True,
                'P1': 0.0,
                'P2': 3520.0
            }]
        ),
    ]) 