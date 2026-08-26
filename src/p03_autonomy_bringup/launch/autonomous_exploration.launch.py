from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    sim_config = LaunchConfiguration("sim_config")

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("p03_autonomy_bringup"),
                    "launch",
                    "manual_navigation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "rviz": rviz,
            "sim_config": sim_config,
            "enable_obstacles": "true",
            "enable_frontier": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("sim_config", default_value="indoor"),
            autonomy,
        ]
    )
