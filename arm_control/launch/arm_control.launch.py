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

    path = os.path.join(get_package_share_directory('arm_description'))

    xacro_arm = os.path.join(path,"urdf", "arm.urdf.xacro")

    robot_description_arm_xacro = {"robot_description": Command(['xacro', xacro_arm])}

    robot_controllers = os.path.join(get_package_share_directory("arm_control"),"config","arm_control.yaml")
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster","--controller-manager", "/controller_manager"],
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller","--controller-manager", "/controller_manager"],
    )


    nodes_to_start = [
        position_controller,
        joint_state_broadcaster 
    ]

    return LaunchDescription(declared_arguments + nodes_to_start) 
