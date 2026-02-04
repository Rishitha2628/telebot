import os
import launch, launch_ros
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)

def generate_launch_description():
    map_file_path = os.path.join(
        get_package_share_directory('telebot_bringup'), 
        'maps',                                   
        'telebot_world.yaml'           
    )
    

    lifecycle_nodes = [
                   'map_server'
                   ]

    return LaunchDescription([
        # Map server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file_path}]
        ),


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}]),


    ])
