from setuptools import setup

package_name = 'eeen489_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 robot package for the ENGR489 platform.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'low_level_ros_interface_node = eeen489_robot.low_level_ros_interface_node:main',
            'high_level_ros_interface_node = eeen489_robot.high_level_ros_interface_node:main',
            'dual_imu_publisher_node = eeen489_robot.dual_imu_publisher_node:main',
            'lidar_3d_pointcloud_node = eeen489_robot.lidar_3d_pointcloud_node:main',
            'swap_scan_axis = eeen489_robot.swap_scan_axis:main',
        ],
    },
)
