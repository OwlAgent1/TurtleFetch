from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():
	namespace = "TurtleFetch"
	#Packages Paths
	pkg_turtlefetch_turtlebot4 = FindPackageShare('turtlefetch_turtlebot4')
	pkg_turtlebot4_navigation = FindPackageShare('turtlebot4_navigation')
	pkg_turtlebot4_viz = FindPackageShare('turtlebot4_viz')

	#Launch File Paths
	turtlebot4_localization_launch = PathJoinSubstitution([pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])
	turtlebot4_navigation_launch = PathJoinSubstitution([pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])
	view_robot_launch = PathJoinSubstitution([pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
	
	#Map File Path
	path2map = PathJoinSubstitution([pkg_turtlefetch_turtlebot4, 'maps', 'map.yaml'])
	
	#Behavior Tree File Path
	path2bt = PathJoinSubstitution([pkg_turtlefetch_turtlebot4, 'config', 'nav2.yaml'])
	
	#Localization
	turtlebot4_localization = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([turtlebot4_localization_launch]),
		launch_arguments=[
			('namespace', namespace),
			('map',path2map)
		],
	)

	#turtlebot4_localization = ExecuteProcess(cmd=['gnome-terminal','--','bash','-c',f'ros2 launch turtlebot4_navigation localization.launch.py namespace:={namespace} use_sim_time:=false map:={path2map}; exec bash'])
	
	#Navigation

	turtlebot4_navigation = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([turtlebot4_navigation_launch]),
		launch_arguments=[
			('namespace', namespace),
			('params_file',path2bt)
		],
	)

	#turtlebot4_navigation = ExecuteProcess(cmd=['gnome-terminal','--','bash','-c',f'ros2 launch turtlebot4_navigation nav2.launch.py namespace:={namespace} params_file:={path2bt}; exec bash'])
	
	#Rviz
	'''
	view_robot = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([view_robot_launch]),
		launch_arguments=[
			('namespace', namespace),
		],
	)
	'''
	view_robot = ExecuteProcess(cmd=['gnome-terminal','--','bash','-c',f'ros2 launch turtlebot4_viz view_robot.launch.py namespace:={namespace}; exec bash'])
	
	delayed_action1 = TimerAction(period=15.0, actions=[turtlebot4_navigation])
	
	delayed_action2 = TimerAction(period=35.0, actions=[view_robot])
	
	#Launch Description
	ld = LaunchDescription()
	ld.add_action(turtlebot4_localization)
	#ld.add_action(turtlebot4_navigation)
	#ld.add_action(view_robot)
	ld.add_action(delayed_action1)
	ld.add_action(delayed_action2)
	return ld
