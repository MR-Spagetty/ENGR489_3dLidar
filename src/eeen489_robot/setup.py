from glob import glob
from setuptools import setup
from setuptools.command.install import install
import os
import sys

package_name = 'eeen489_robot'

class CustomInstall(install):
    """Custom install command that creates libexec symlinks for ROS2."""
    
    def run(self):
        # Run the standard install
        install.run(self)
        
        # Create libexec symlinks for ROS2 console script discovery
        # The install_lib directory is where the Python package gets installed
        install_lib = self.install_lib
        
        # Navigate from lib/pythonX.Y/site-packages/eeen489_robot to install root
        # We need to go up: site-packages -> pythonX.Y -> lib -> root
        parts = install_lib.split(os.sep)
        
        # Find the 'lib' component and get everything up to (and including) it
        try:
            lib_idx = parts.index('lib')
            install_root = os.sep.join(parts[:lib_idx+1])  # Up to 'lib' inclusive
            install_root = os.sep.join(parts[:lib_idx])     # Go one level up to root
        except ValueError:
            # Fallback: if structure is different, try to get parent of lib
            install_root = os.path.dirname(os.path.dirname(os.path.dirname(install_lib)))
        
        libexec_dir = os.path.join(install_root, 'lib', package_name)
        bin_dir = os.path.join(install_root, 'bin')
        
        if not os.path.exists(bin_dir):
            print(f"Warning: bin directory not found at {bin_dir}", file=sys.stderr)
            return
        
        # Create libexec directory
        os.makedirs(libexec_dir, exist_ok=True)
        
        # List of console scripts to symlink
        scripts = [
            'low_level_ros_interface_node',
            'high_level_ros_interface_node',
            'dual_imu_publisher_node',
            'lidar_3d_pointcloud_node',
            'swap_scan_axis',
        ]
        
        # Create symlinks
        for script in scripts:
            script_path = os.path.join(bin_dir, script)
            symlink_path = os.path.join(libexec_dir, script)
            
            if os.path.exists(script_path):
                # Remove existing symlink if present
                if os.path.islink(symlink_path) or os.path.exists(symlink_path):
                    try:
                        os.unlink(symlink_path)
                    except OSError:
                        pass
                
                # Create relative symlink
                try:
                    os.symlink('../../bin/' + script, symlink_path)
                    print(f"Created symlink: {symlink_path} -> ../../bin/{script}")
                except OSError as e:
                    print(f"Warning: Failed to create symlink for {script}: {e}", file=sys.stderr)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 robot package for the ENGR489 platform.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'low_level_ros_interface_node = eeen489_robot.low_level_ros_interface_node:main',
            'high_level_ros_interface_node = eeen489_robot.high_level_ros_interface_node:main',
            'dual_imu_publisher_node = eeen489_robot.dual_imu_publisher_node:main',
            'lidar_3d_pointcloud_node = eeen489_robot.lidar_3d_pointcloud_node:main',
            'swap_scan_axis = eeen489_robot.swap_scan_axis:main',
        ],
    },
    cmdclass={'install': CustomInstall},
)
