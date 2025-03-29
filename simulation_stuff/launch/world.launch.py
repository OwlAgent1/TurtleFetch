from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
		
	gazebo_launch = ExecuteProcess(
		cmd=['ign', 'gazebo', '-v', '4', PathJoinSubstitution([FindPackageShare("simulation_stuff"), "worlds", "testworld.sdf"])]
		)
	
	return LaunchDescription([
		gazebo_launch, 
		])
