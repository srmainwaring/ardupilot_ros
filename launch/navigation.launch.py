from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import SetRemap
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path

"""Generate a launch description for the navigation example."""


def generate_launch_description():
    # Navigation
    navigation = GroupAction(
        actions=[
            SetRemap(src="/cmd_vel", dst="/ap/cmd_vel"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        Path(
                            FindPackageShare("nav2_bringup").find("nav2_bringup"),
                            "launch",
                            "navigation_launch.py",
                        )
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "params_file": FindPackageShare("ardupilot_ros").find(
                        "ardupilot_ros"
                    )
                    + "/config"
                    + "/navigation.yaml",
                }.items(),
            ),
        ]
    )

    # RViz.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            str(
                Path(
                    FindPackageShare("ardupilot_ros").find("ardupilot_ros"),
                    "rviz",
                    "navigation.rviz",
                )
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            rviz,
            navigation,
        ]
    )
