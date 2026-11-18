#!/usr/bin/env -S ros2 launch
"""Launch MoveIt2 + RViz2 with the PhantomX Pincher hardware interface (ros2_control).

Adapted from fake.launch.py of snt-spacer/phantomx_pincher (BSD-3-Clause):
the only functional change is the ros2_control plugin, which is our own.
"""

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    declared_arguments = generate_declared_arguments()

    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    launch_descriptions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("phantomx_pincher_hardware"),
                        "launch",
                        "move_group_real.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("ros2_control", "true"),
                # Our plugin instead of "fake"
                (
                    "ros2_control_plugin",
                    "phantomx_pincher_hardware/PhantomxPincherHardware",
                ),
                ("ros2_control_interface", "position"),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("log_level", log_level),
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("phantomx_pincher_moveit_config"),
                "rviz",
                "moveit.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level applied to all nodes launched by this script.",
        ),
    ]
