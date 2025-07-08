from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_aeva',
            default_value='true',
            description='Enable aeva topic playback'
        ),
        DeclareLaunchArgument(
            'enable_continental',
            default_value='false',
            description='Enable continental topic playback'
        ),
        DeclareLaunchArgument(
            'enable_stereo',
            default_value='false',
            description='Enable stereo topic playback'
        ),
        DeclareLaunchArgument(
            'enable_radarpolar',
            default_value='false',
            description='Enable radarpolar topic playback'
        ),
        DeclareLaunchArgument(
            'enable_stereo_right',
            default_value='false',
            description='Enable stereo_right topic playback'
        ),
        DeclareLaunchArgument(
            'enable_stereo_left',
            default_value='false',
            description='Enable stereo_left topic playback'
        ),

        Node(
            package='hercules_file_player',
            executable='file_player',
            name='file_player',
            output='screen',
            parameters=[{
                'enable_aeva': LaunchConfiguration('enable_aeva'),
                'enable_continental': LaunchConfiguration('enable_continental'),
                'enable_stereo': LaunchConfiguration('enable_stereo'),
                'enable_radarpolar': LaunchConfiguration('enable_radarpolar'),
                'enable_stereo_right': LaunchConfiguration('enable_stereo_right'),
                'enable_stereo_left': LaunchConfiguration('enable_stereo_left'),
            }]
        )
    ]) 