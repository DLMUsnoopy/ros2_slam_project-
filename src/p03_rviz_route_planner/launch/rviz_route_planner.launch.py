from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional robot namespace, for example P1_robot0.",
            ),
            DeclareLaunchArgument(
                "default_frame_id",
                default_value="map",
                description="Frame used when clicked points do not contain a frame id.",
            ),
            Node(
                package="p03_rviz_route_planner",
                executable="p03_rviz_route_planner",
                namespace=LaunchConfiguration("namespace"),
                name="p03_rviz_route_planner",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"default_frame_id": LaunchConfiguration("default_frame_id")},
                ],
            ),
        ]
    )
