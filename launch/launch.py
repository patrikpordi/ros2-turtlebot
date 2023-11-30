from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
    DeclareLaunchArgument('rosbag_record', default_value = 'false', choices = ['true', 'false'], description = "Enable rosbag recording"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'turtlebot3_gazebo'), 'launch'), '/turtlebot3_world.launch.py'
            ])
        ),
        # Launch the walker node
        Node(
            package='ros2-turtlebot',
            executable='walking',
            name='Walking_node',
            output='screen'
        ),
            ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('rosbag_record')),
            cmd=['ros2', 'bag', 'record', '-o','walker_bag','-a','-x','depth_cam'],
            shell=True
        )
    ])