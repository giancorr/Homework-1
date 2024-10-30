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


def generate_launch_description():

    
    config = os.path.join(get_package_share_directory('arm_controller'),
    "config",
    "params.yaml"
    )

    node = Node(
        package= 'arm_controller',
        name = 'arm_controller_node',
        executable = 'arm_controller_node_ex',
        parameters = [config]
    )

    nodes_to_start = [
        node
    ]
    
    return LaunchDescription(nodes_to_start) 