from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Force ROS2 to use localhost only to avoid FastDDS network discovery issues
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),

        DeclareLaunchArgument(
            'hs_port',
            default_value='9870',
            description='UDP port for height scan'
        ),
        DeclareLaunchArgument(
            'ss_port',
            default_value='9871',
            description='UDP port for safety status'
        ),
        DeclareLaunchArgument(
            'hs_rows',
            default_value='11',
            description='Height scan grid rows'
        ),
        DeclareLaunchArgument(
            'hs_cols',
            default_value='17',
            description='Height scan grid cols'
        ),
        Node(
            package='udp_test_bridge',
            executable='udp_test_receiver',
            name='udp_test_receiver',
            output='screen',
            parameters=[{
                'hs_port': LaunchConfiguration('hs_port'),
                'ss_port': LaunchConfiguration('ss_port'),
                'hs_rows': LaunchConfiguration('hs_rows'),
                'hs_cols': LaunchConfiguration('hs_cols'),
            }],
        ),
    ])
