#!/usr/bin/env python3
"""
Post-build script to create ROS2 libexec symlinks for console scripts.
This should be run after colcon build completes.

Usage:
  python3 setup_libexec.py <install_root>

Example:
  python3 setup_libexec.py ~/ENGR489_3dLidar/install/eeen489_robot
"""

import os
import sys
import argparse

def create_libexec_symlinks(package_install_dir):
    """Create symlinks from libexec to console scripts in bin."""
    
    scripts = [
        'low_level_ros_interface_node',
        'high_level_ros_interface_node',
        'dual_imu_publisher_node',
        'lidar_3d_pointcloud_node',
        'swap_scan_axis',
    ]
    
    bin_dir = os.path.join(package_install_dir, 'bin')
    libexec_dir = os.path.join(package_install_dir, 'lib', 'eeen489_robot')
    
    if not os.path.exists(bin_dir):
        print(f"Error: bin directory not found at {bin_dir}")
        return False
    
    os.makedirs(libexec_dir, exist_ok=True)
    print(f"Created libexec directory: {libexec_dir}")
    
    for script in scripts:
        script_path = os.path.join(bin_dir, script)
        if os.path.exists(script_path):
            link_path = os.path.join(libexec_dir, script)
            if os.path.lexists(link_path):
                try:
                    os.remove(link_path)
                except Exception as e:
                    print(f"Warning: Could not remove existing link {link_path}: {e}")
            
            try:
                rel_path = os.path.relpath(script_path, libexec_dir)
                os.symlink(rel_path, link_path)
                print(f"Created symlink: {script}")
            except Exception as e:
                print(f"Error: Could not create symlink for {script}: {e}")
                return False
        else:
            print(f"Warning: Script not found at {script_path}")
    
    print("✓ Libexec symlinks created successfully!")
    return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Create ROS2 libexec symlinks for console scripts',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python3 setup_libexec.py ~/ENGR489_3dLidar/install/eeen489_robot
  python3 setup_libexec.py /path/to/install/eeen489_robot
        '''
    )
    parser.add_argument(
        'install_dir',
        help='Path to the package install directory'
    )
    
    args = parser.parse_args()
    install_dir = os.path.expanduser(args.install_dir)
    
    if not os.path.exists(install_dir):
        print(f"Error: Install directory not found: {install_dir}")
        sys.exit(1)
    
    if create_libexec_symlinks(install_dir):
        sys.exit(0)
    else:
        sys.exit(1)
