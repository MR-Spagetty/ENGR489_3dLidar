"""ENGR489 Robot Platform ROS2 package."""

import os
import sys

def _ensure_libexec_symlinks():
    """
    Ensure ROS2 libexec directory exists with symlinks to console scripts.
    This function runs when the package is first imported.
    """
    try:
        # Get the directory where this __init__.py lives
        package_dir = os.path.dirname(os.path.abspath(__file__))
        # For structure: install/eeen489_robot/lib/python3.12/site-packages/eeen489_robot/__init__.py
        # We need to go up to install/eeen489_robot
        # Or for symlink-install: build/eeen489_robot/eeen489_robot/__init__.py -> src/eeen489_robot/eeen489_robot/__init__.py
        
        # Try to find the install/build root
        # Start from package directory and go up
        for _ in range(5):  # Max 5 levels up
            package_dir = os.path.dirname(package_dir)
            if os.path.basename(package_dir) == 'eeen489_robot' and os.path.exists(os.path.join(package_dir, 'bin')):
                # Found the package install/build root
                install_root = package_dir
                break
        else:
            return  # Couldn't find install root
        
        bin_dir = os.path.join(install_root, 'bin')
        libexec_dir = os.path.join(install_root, 'lib', 'eeen489_robot')
        
        # Only proceed if bin directory exists
        if not os.path.exists(bin_dir):
            return
        
        # Create libexec directory if it doesn't exist
        if not os.path.exists(libexec_dir):
            try:
                os.makedirs(libexec_dir, exist_ok=True)
            except Exception:
                return  # Can't create directory
        
        # Console scripts to symlink
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
            if os.path.exists(script_path):
                link_path = os.path.join(libexec_dir, script)
                # Skip if symlink already exists
                if os.path.islink(link_path) and os.readlink(link_path) == os.path.relpath(script_path, libexec_dir):
                    continue
                # Remove existing link
                if os.path.lexists(link_path):
                    try:
                        os.remove(link_path)
                    except Exception:
                        continue
                # Create relative symlink
                try:
                    rel_path = os.path.relpath(script_path, libexec_dir)
                    os.symlink(rel_path, link_path)
                except Exception:
                    pass  # Silently continue if symlink creation fails
    except Exception:
        pass  # Silently fail - this is just a helper


# Ensure libexec symlinks exist when package is imported
_ensure_libexec_symlinks()

