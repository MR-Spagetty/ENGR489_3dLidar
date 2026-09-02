from launch import LaunchDescription
from launch_ros.actions import Node
from math import pi
import transforms3d
import numpy as np

def rad2Quan(zrot):
    radians = np.deg2rad(zrot)
    arr = transforms3d.euler.euler2quat(0,0,radians)
    return arr


footprint_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'base_link', '--child-frame-id', 'base_footprint'],
        )


lidar_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', str(-3.5/100.0), '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        )
quanrot = rad2Quan(22.5) # (45/2)+45*5 degrees
Ultraa_transform = Node( # 1 "0x20"
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', str(4.527/100.0), '--y', str(0.74/100.0), '--z', '0', '--qx',str(quanrot[1]) , '--qy', str(quanrot[2]), '--qz',str(quanrot[3]), '--qw', str(quanrot[0]), '--frame-id', 'base_link', '--child-frame-id', 'Ultraa'],
        )
quanrot = rad2Quan(-22.5) # (45/2)+45*5 degrees
Ultrab_transform = Node(# marked || IE 2 "0x21"
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', str(4.527/100.0), '--y', str(-3.04/100.0), '--z', '0', '--qx',str(quanrot[1]) , '--qy', str(quanrot[2]), '--qz',str(quanrot[3]), '--qw', str(quanrot[0]), '--frame-id', 'base_link', '--child-frame-id', 'Ultrab'],
        )
quanrot = rad2Quan(22.5+(45*2)+180) # (45/2)+45*5 degrees
Ultrac_transform = Node(# marked ||| IE 3 "0x22" etc
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments =  ['--x', str(1.8944/100.0), '--y', str(-5.677/100.0), '--z', '0', '--qx',str(quanrot[1]) , '--qy', str(quanrot[2]), '--qz',str(quanrot[3]), '--qw', str(quanrot[0]), '--frame-id', 'base_link', '--child-frame-id', 'Ultrac'],
        )
quanrot = rad2Quan(22.5+(45*5)) # (45/2)+45*5 degrees
Ultrad_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x',str(-1.8944/100.0) , '--y', str(-5.677/100.0), '--z', '0', '--qx',str(quanrot[1]) , '--qy', str(quanrot[2]), '--qz',str(quanrot[3]), '--qw', str(quanrot[0]), '--frame-id', 'base_link', '--child-frame-id', 'Ultrad'],
        )
quanrot = rad2Quan((22.5+(45*4))) # (45/2)+45*5 degrees
Ultrae_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x',str(-4.527/100.0) , '--y', str(-3.04/100.0), '--z', '0', '--qx',str(quanrot[1]) , '--qy', str(quanrot[2]), '--qz',str(quanrot[3]), '--qw', str(quanrot[0]), '--frame-id', 'base_link', '--child-frame-id', 'Ultrae'],
        )
quanrot = rad2Quan(22.5+(45*3)) # (45/2)+45*5 degrees
Ultraf_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments =  ['--x', str(-4.527/100.0), '--y', str(0.74/100.0), '--z', '0', '--qx',str(quanrot[1]) , '--qy', str(quanrot[2]), '--qz',str(quanrot[3]), '--qw', str(quanrot[0]), '--frame-id', 'base_link', '--child-frame-id', 'Ultraf'],
        )
quanrot = rad2Quan(-1.0*(22.5+(45*5))) # (45/2)+45*5 degrees
Ultrag_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments =  ['--x', str(-1.8944/100.0), '--y', str(3.377/100.0), '--z', '0', '--qx',str(quanrot[1]) , '--qy', str(quanrot[2]), '--qz',str(quanrot[3]), '--qw', str(quanrot[0]), '--frame-id', 'base_link', '--child-frame-id', 'Ultrag'],
        )
quanrot = rad2Quan(-1.0*(22.5+(45*6))) # (45/2)+45*5 degrees
Ultrah_transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments =  ['--x', str(1.8944/100.0), '--y', str(3.377/100.0), '--z', '0', '--qx',str(quanrot[1]) , '--qy', str(quanrot[2]), '--qz',str(quanrot[3]), '--qw', str(quanrot[0]), '--frame-id', 'base_link', '--child-frame-id', 'Ultrah'],
        )
def generate_launch_description():
    return LaunchDescription([
        footprint_transform,
        lidar_transform,
        Ultraa_transform,
        Ultrab_transform,
        Ultrac_transform,
        Ultrad_transform,
        Ultrae_transform,
        Ultraf_transform,
        Ultrag_transform,
        Ultrah_transform,
        
    ])