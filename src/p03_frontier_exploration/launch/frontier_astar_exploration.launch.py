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
            DeclareLaunchArgument("goal_update_distance", default_value="0.01"),
            DeclareLaunchArgument("inflation_radius", default_value="0.15"),
            DeclareLaunchArgument("obstacle_cost_radius", default_value="0.45"),
            DeclareLaunchArgument("obstacle_cost_weight", default_value="4.0"),
            DeclareLaunchArgument("endpoint_search_radius", default_value="0.75"),
            DeclareLaunchArgument("max_shortcut_obstacle_cost", default_value="1.0"),
            DeclareLaunchArgument("robot_clearance_radius", default_value="0.0"),
            DeclareLaunchArgument("start_search_radius", default_value="0.75"),
            DeclareLaunchArgument("min_frontier_size", default_value="8"),
            DeclareLaunchArgument("selection_strategy", default_value="information_gain"),
            DeclareLaunchArgument("information_gain_radius", default_value="1.0"),
            DeclareLaunchArgument("information_gain_weight", default_value="0.05"),
            DeclareLaunchArgument("replan_rate", default_value="5.0"),
            DeclareLaunchArgument("replan_on_map_update", default_value="false"),
            DeclareLaunchArgument("map_preprocess_rate", default_value="2.0"),
            DeclareLaunchArgument("freeze_prefix_waypoints", default_value="2"),
            DeclareLaunchArgument("route_progress_max_distance", default_value="1.0"),
            DeclareLaunchArgument("waypoint_spacing", default_value="0.20"),
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
                    {"robot_clearance_radius": LaunchConfiguration("robot_clearance_radius")},
                    {"start_search_radius": LaunchConfiguration("start_search_radius")},
                    {"min_frontier_size": LaunchConfiguration("min_frontier_size")},
                    {"selection_strategy": LaunchConfiguration("selection_strategy")},
                    {"information_gain_radius": LaunchConfiguration("information_gain_radius")},
                    {"information_gain_weight": LaunchConfiguration("information_gain_weight")},
                ],
            ),
            Node(
                package="p03_rviz_route_planner",
                executable="astar_global_planner",
                name="astar_global_planner",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"map_topic": LaunchConfiguration("map_topic")},
                    {"pose_topic": LaunchConfiguration("pose_topic")},
                    {"pose_frame_override": LaunchConfiguration("pose_frame_override")},
                    {"goal_pose_topic": LaunchConfiguration("goal_pose_topic")},
                    {"goal_update_distance": LaunchConfiguration("goal_update_distance")},
                    {"inflation_radius": LaunchConfiguration("inflation_radius")},
                    {"obstacle_cost_radius": LaunchConfiguration("obstacle_cost_radius")},
                    {"obstacle_cost_weight": LaunchConfiguration("obstacle_cost_weight")},
                    {"endpoint_search_radius": LaunchConfiguration("endpoint_search_radius")},
                    {"replan_on_map_update": LaunchConfiguration("replan_on_map_update")},
                    {"replan_rate": LaunchConfiguration("replan_rate")},
                    {"map_preprocess_rate": LaunchConfiguration("map_preprocess_rate")},
                    {"freeze_prefix_waypoints": LaunchConfiguration("freeze_prefix_waypoints")},
                    {"route_progress_max_distance": LaunchConfiguration("route_progress_max_distance")},
                    {"waypoint_spacing": LaunchConfiguration("waypoint_spacing")},
                ],
            ),
        ]
    )
