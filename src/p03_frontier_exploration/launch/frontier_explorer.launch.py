from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("pose_topic", default_value="/P1_robot0/odom_gt"),
            DeclareLaunchArgument("pose_frame_override", default_value="map"),
            DeclareLaunchArgument("goal_pose_topic", default_value="/P1_robot0/goal_pose"),
            DeclareLaunchArgument("update_rate", default_value="1.0"),
            DeclareLaunchArgument("min_frontier_size", default_value="8"),
            DeclareLaunchArgument("robot_clearance_radius", default_value="0.25"),
            DeclareLaunchArgument("goal_reached_distance", default_value="0.45"),
            DeclareLaunchArgument("selection_strategy", default_value="information_gain"),
            DeclareLaunchArgument("information_gain_radius", default_value="1.0"),
            DeclareLaunchArgument("information_gain_weight", default_value="0.05"),
            Node(
                package="p03_frontier_exploration",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"map_topic": LaunchConfiguration("map_topic")},
                    {"pose_topic": LaunchConfiguration("pose_topic")},
                    {"pose_frame_override": LaunchConfiguration("pose_frame_override")},
                    {"goal_pose_topic": LaunchConfiguration("goal_pose_topic")},
                    {"update_rate": LaunchConfiguration("update_rate")},
                    {"min_frontier_size": LaunchConfiguration("min_frontier_size")},
                    {"robot_clearance_radius": LaunchConfiguration("robot_clearance_radius")},
                    {"goal_reached_distance": LaunchConfiguration("goal_reached_distance")},
                    {"selection_strategy": LaunchConfiguration("selection_strategy")},
                    {"information_gain_radius": LaunchConfiguration("information_gain_radius")},
                    {"information_gain_weight": LaunchConfiguration("information_gain_weight")},
                ],
            ),
        ]
    )
