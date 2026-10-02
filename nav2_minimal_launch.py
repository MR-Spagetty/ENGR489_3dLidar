import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"autostart": "true"},
        convert_types=True,
    )

    lifecycle_nodes = [
        "map_server",
        "amcl",
    ]

    navigation_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("map"),

        # Localization
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

        # Navigation
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
