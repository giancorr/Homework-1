from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit   
    
def generate_launch_description():

    declared_arguments = [] 

    declared_arguments.append(DeclareLaunchArgument('gz_args', default_value='-r -v 1 empty.sdf',
                              description='Arguments for gz_sim'),)

    gazebo_ignition_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('arm_gazebo'),
            'launch','arm_world.launch.py'])]), 
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    gazebo_controller_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('arm_control'), 
            'launch', 'arm_control.launch.py'])])
    )   

    launch = [gazebo_controller_spawner, gazebo_ignition_simulator]

    nodes_to_start = [
        *launch,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start) 
