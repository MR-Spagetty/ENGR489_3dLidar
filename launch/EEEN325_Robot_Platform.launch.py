## EEEN325 Robot Platform Launch File
# In this file we will start up the following nodes:
## Lab 1
# teleop_twist_joy
# joy
# LowLevelInterfaceNode
## Lab 2
# teleop_twist_joy
# joy
# LowLevelInterfaceNode
# HighLevelInterfaceNode
# tf2 # (potentially)
## Lab 3
# teleop_twist_joy
# joy
# LowLevelInterfaceNode
# HighLevelInterfaceNode
# TofSensorNode
# tf2 # (potentially)
## still requires manual static transform publishers#
#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link base_footprint
#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link base_laser


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import GroupAction, DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import PushRosNamespace, SetRemap, SetParametersFromFile
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
	xbox_config_location = PathJoinSubstitution([FindPackageShare("eeen489_robot"),"EEEN325_XBOX_series.config.yaml"])
	robot_config_location = PathJoinSubstitution([FindPackageShare("eeen489_robot"),"EEEN325_Low_Level_Interface.config.yaml"])
	twist_mux_config_location = PathJoinSubstitution([FindPackageShare("eeen489_robot"),"twist_mux.config.yaml"])
	# xbox_config_location = os.path.join(
    #   get_package_share_directory('low_level_ros_interface'),
    #   'config',
    #   'EEEN325_XBOX_series.config.yaml'
    #   )
	twist_mux_config = LaunchConfiguration('twist_mux_config')
	twist_mux_config_filepath_arg = DeclareLaunchArgument(
		'twist_mux_config',
		default_value=twist_mux_config_location
	)
	xbox_config = LaunchConfiguration('xbox_config')
	xbox_config_filepath_arg = DeclareLaunchArgument(
		'xbox_config',
		default_value=xbox_config_location
	)
	low_level_interface = Node(
		package='eeen489_robot',
		executable='low_level_ros_interface_node',
		name='low_level',
		remappings=[
            ('/cmd_vel', '/cmd_vel_roboclaw'),
		]
	)
	high_level_interface = Node(
		package='eeen489_robot',
		executable='high_level_ros_interface_node',
		name='high_level'
	)
	twist_mux = Node(
		package='twist_mux',
		namespace='',
		executable='twist_mux',
		name='twist_mux',
		remappings=[
			('/cmd_vel_out','/cmd_vel_roboclaw'),	
		],
		parameters = [twist_mux_config],
	
	
)
	
	joy_node = Node(
		package='joy',
		executable='joy_node',
		name='joy_node',
	
	)
	teleop_node = Node(
		package='teleop_twist_joy',
		executable='teleop_node',
		name='teleop_twist_joy_node',
		remappings=[
            ('/cmd_vel', '/cmd_vel_joy'),
	    
		],
		parameters=[xbox_config]
	)
	# Set the transforms
	eeen325_transforms = IncludeLaunchDescription(
		PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('eeen489_robot'),'EEEN325_Robot_static_transform.launch.py'])])
		)
	# to use a different directory for the joystick launch file edit config_filepath
	xbox_series_twist = GroupAction(
		actions=[
			SetParametersFromFile(xbox_config),
			SetRemap(src='/cmd_vel',dst='/cmd_vel_joy'),
			IncludeLaunchDescription(
		PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('teleop_twist_joy'), 'launch', 'teleop-launch.py'])])
			)
		]
	)	
	A1M8Lidar = IncludeLaunchDescription(  
		PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('sllidar_ros2'),'launch','sllidar_a1_launch.py'])])
		)
	action_list =[
		xbox_config_filepath_arg,
		#xbox_series_twist,
		
		
		low_level_interface,
		high_level_interface,
		eeen325_transforms,
		A1M8Lidar,
		
		twist_mux_config_filepath_arg,
		twist_mux,
		
		joy_node,
		teleop_node,
	]
	ultrasonic_addr= [0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27]
	for addr in ultrasonic_addr: 
		ultrasonic_node = Node(
		package='ultrasonics',
		executable='ultrasonic_publisher',
		name=f'ultrasonic_publisher_{addr}', 
		parameters= [{'addr':addr}]
		)
		action_list.append(ultrasonic_node)
	
	
	return LaunchDescription(action_list)

			
