from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock."),
            DeclareLaunchArgument("map_topic", default_value="/map", description="Occupancy grid used by A*."),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/P1_robot0/odom_gt",
                description="Robot pose used as the A* start pose.",
            ),
            DeclareLaunchArgument(
                "pose_frame_override",
                default_value="map",
                description="Frame used to interpret pose_topic coordinates. Empty disables override.",
            ),
            DeclareLaunchArgument(
                "goal_pose_topic",
                default_value="/P1_robot0/goal_pose",
                description="RViz 2D Goal Pose topic used as the A* goal.",
            ),
            DeclareLaunchArgument(
                "goal_update_distance",
                default_value="0.01",
                description="Minimum goal-position change in meters that triggers immediate replanning.",
            ),
            DeclareLaunchArgument(
                "inflation_radius",
                default_value="0.30",
                description="Obstacle inflation radius in meters.",
            ),
            DeclareLaunchArgument(
                "obstacle_cost_radius",
                default_value="0.45",
                description="Radius around occupied cells where A* adds a soft clearance penalty.",
            ),
            DeclareLaunchArgument(
                "obstacle_cost_weight",
                default_value="4.0",
                description="Maximum extra traversal cost near occupied cells.",
            ),
            DeclareLaunchArgument(
                "endpoint_search_radius",
                default_value="0.0",
                description="Nearby search radius used when A* start or goal falls on a blocked cell.",
            ),
            DeclareLaunchArgument(
                "heuristic",
                default_value="diagonal",
                description="A* heuristic: zero, diagonal, euclidean, or weighted_diagonal.",
            ),
            DeclareLaunchArgument(
                "heuristic_weight",
                default_value="1.5",
                description="Multiplier used by weighted_diagonal heuristic.",
            ),
            DeclareLaunchArgument(
                "use_diagonal",
                default_value="true",
                description="Allow diagonal grid moves. Set false for four-connected Manhattan motion.",
            ),
            DeclareLaunchArgument(
                "visualize_search",
                default_value="true",
                description="Publish explored A* cells as an RViz marker.",
            ),
            DeclareLaunchArgument(
                "max_visualized_search_cells",
                default_value="2500",
                description="Maximum A* explored cells shown in RViz.",
            ),
            DeclareLaunchArgument(
                "reduce_waypoints",
                default_value="true",
                description="Reduce dense A* waypoints with collision-free line-of-sight shortcuts.",
            ),
            DeclareLaunchArgument(
                "max_shortcut_obstacle_cost",
                default_value="1.0",
                description="Reject waypoint shortcuts through cells with obstacle penalty above this value.",
            ),
            DeclareLaunchArgument(
                "replan_rate",
                default_value="5.0",
                description="Periodic replanning rate in Hz. Zero disables timer replanning.",
            ),
            DeclareLaunchArgument(
                "replan_on_map_update",
                default_value="false",
                description="Trigger planning on processed live-map updates.",
            ),
            DeclareLaunchArgument(
                "map_preprocess_rate",
                default_value="2.0",
                description="Maximum live occupancy-map preprocessing rate in Hz.",
            ),
            DeclareLaunchArgument(
                "route_progress_max_distance",
                default_value="1.0",
                description="Maximum route distance used to advance the cached waypoint prefix.",
            ),
            DeclareLaunchArgument(
                "waypoint_spacing",
                default_value="0.20",
                description="Maximum spacing in meters between published NMPC waypoints.",
            ),
            DeclareLaunchArgument(
                "freeze_prefix_waypoints",
                default_value="2",
                description="Keep this many already-published waypoints unchanged during replanning.",
            ),
            DeclareLaunchArgument(
                "waypoint_reached_threshold",
                default_value="0.10",
                description="Distance in meters for pruning reached cached waypoints.",
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
                    {"heuristic": LaunchConfiguration("heuristic")},
                    {"heuristic_weight": LaunchConfiguration("heuristic_weight")},
                    {"use_diagonal": LaunchConfiguration("use_diagonal")},
                    {"visualize_search": LaunchConfiguration("visualize_search")},
                    {"max_visualized_search_cells": LaunchConfiguration("max_visualized_search_cells")},
                    {"reduce_waypoints": LaunchConfiguration("reduce_waypoints")},
                    {"max_shortcut_obstacle_cost": LaunchConfiguration("max_shortcut_obstacle_cost")},
                    {"replan_rate": LaunchConfiguration("replan_rate")},
                    {"replan_on_map_update": LaunchConfiguration("replan_on_map_update")},
                    {"map_preprocess_rate": LaunchConfiguration("map_preprocess_rate")},
                    {"freeze_prefix_waypoints": LaunchConfiguration("freeze_prefix_waypoints")},
                    {"route_progress_max_distance": LaunchConfiguration("route_progress_max_distance")},
                    {"waypoint_spacing": LaunchConfiguration("waypoint_spacing")},
                    {"waypoint_reached_threshold": LaunchConfiguration("waypoint_reached_threshold")},
                ],
            ),
        ]
    )
