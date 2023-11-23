from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('capture_chunk_size', default_value='32768', description='Audio capture chunk size'),
        Node(
            package='turtsim',
            executable='hear',
            output='screen'),
        Node(
            package='turtsim',
            executable='rec',
            output='screen'),
    ])