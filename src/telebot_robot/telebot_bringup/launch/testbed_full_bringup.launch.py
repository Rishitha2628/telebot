#!/usr/bin/python3
import os
import launch, launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node

def generate_launch_description():

  pkg_telebot_gazebo = get_package_share_directory('telebot_gazebo')
  pkg_telebot_description = get_package_share_directory('telebot_description')
  pkg_dir_navigation = get_package_share_directory('telebot_navigation')

  use_sim_time = LaunchConfiguration('use_sim_time')

  declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True', # Or 'False'
        description='Use simulation (Gazebo) clock if true'
    )

  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_telebot_gazebo, 'launch', 'spawn_playground.launch.py'),
    )
  ) 
  
  state_pub = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_telebot_description, 'launch', 'robot_description.launch.py'),
    )
  )

  spawn = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_telebot_gazebo, 'launch', 'spawn_telebot.launch.py'),
    )
  )
  
  # rviz_config_dir = os.path.join(
  #   launch_ros.substitutions.FindPackageShare(package='telebot_description').find('telebot_description'),
  #   'rviz/full_bringup.rviz')

  rviz_config_dir = os.path.join(
    pkg_telebot_description,
    'rviz',
    'full_bringup.rviz'
)
  
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz_node',
    parameters=[{'use_sim_time': True}],
    arguments=['-d', LaunchConfiguration('rvizconfig')]
  )

  map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir_navigation, 'launch', 'map_loader.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
  
  # robot_spawn_x = 0
  # robot_spawn_y = 5
  # robot_spawn_yaw = 0

  # map_origin_x = -10.2
  # map_origin_y = -9.94
  # map_origin_yaw = 0.0

  # # Compute AMCL initial pose
  # amcl_x = -1*(robot_spawn_x - map_origin_x)
  # amcl_y = -1*(robot_spawn_y - map_origin_y)
  # amcl_yaw = -1*(robot_spawn_yaw - map_origin_yaw)
  
  amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir_navigation, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
                # 'initial_x': str(amcl_x),
                # 'initial_y': str(amcl_y),
                # 'initial_yaw': str(amcl_yaw),
                'use_sim_time': use_sim_time
            }.items()
    )
  
  nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir_navigation, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
  
  node_1 = Node(
        package='telebot_navigation',
        executable='controller_node',
        name='controller',
        output='screen'
    )

  node_2 = Node(
      package='telebot_navigation',
      executable='sensor_node',
      name='sensor',
      output='screen'
  )

  return LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_dir,
                                            description='Absolute path to rviz config file'),
    declare_sim_time_arg,
    state_pub,
    gazebo,
    spawn,
    rviz_node,
    map_server_launch,
    amcl_launch,
    nav2_launch
  ])