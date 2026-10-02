"""Minimal Nav2 bring-up for a map-based localization and navigation stack.

This launch file prepares a static map, starts the localization nodes, and then
boots the core Nav2 controllers and planners. The remappings keep the planner and
controller output from colliding with the robot's main velocity command topic.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Return the ROS 2 launch description for the minimal map + navigation stack.

    The YAML file contains the common Nav2 parameters. RewrittenYaml ensures the
    autostart flag is forced on for the lifecycle managers even when the source
    configuration leaves it unset.
    """
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"autostart": "true"},
        convert_types=True,
    )

    # These nodes must be lifecycle-managed during the localization phase.
    lifecycle_nodes = [
        "map_server",
        "amcl",
    ]

    # These nodes are started after localization and represent the active path
    # planning and execution stack.
    navigation_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]

    return LaunchDescription([
        # Required command-line arguments: path to Nav2 YAML and the map to load.
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("map"),

        # Localization stage: map and AMCL initialize first so the robot has a
        # known frame and pose estimate before navigation logic starts.
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                configured_params,
                {"yaml_filename": map_file},
            ],
        ),

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[configured_params],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": lifecycle_nodes,
            }],
        ),

        # Navigation stage: planning and execution nodes start after localization.
        # The command remaps separate the autonomous nav commands from the base
        # teleop or joystick velocity stream used elsewhere in the system.
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[configured_params],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_params],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[configured_params],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[configured_params],
        ),

        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[configured_params],
            remappings=[
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": navigation_lifecycle_nodes,
            }],
        ),
    ])
