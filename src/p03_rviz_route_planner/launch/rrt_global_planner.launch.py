from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock."),
            DeclareLaunchArgument("map_topic", default_value="/map", description="Occupancy grid used by RRT."),
            DeclareLaunchArgument(
                "sampling_process_topic",
                default_value="rrt_sampling_process",
                description="MarkerArray topic used for recorded RRT sampling replay.",
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/P1_robot0/odom_gt",
                description="Robot start pose topic.",
            ),
            DeclareLaunchArgument(
                "pose_frame_override",
                default_value="map",
                description="Interpret robot pose coordinates in this frame when non-empty.",
            ),
            DeclareLaunchArgument(
                "goal_pose_topic",
                default_value="/P1_robot0/goal_pose",
                description="RViz goal pose topic.",
            ),
            DeclareLaunchArgument(
                "inflation_radius",
                default_value="0.30",
                description="Obstacle inflation radius in meters.",
            ),
            DeclareLaunchArgument(
                "adjust_goal_if_blocked",
                default_value="true",
                description="If the requested goal is blocked, retry with the last free cell before the obstacle.",
            ),
            DeclareLaunchArgument(
                "fallback_on_rrt_timeout",
                default_value="true",
                description="Use adjusted-goal fallback even when the original goal is free but RRT timed out.",
            ),
            DeclareLaunchArgument(
                "max_adjusted_goal_candidates",
                default_value="250",
                description="Maximum cells scanned while building adjusted-goal candidates.",
            ),
            DeclareLaunchArgument(
                "adjusted_goal_attempts",
                default_value="40",
                description="Maximum reachable adjusted-goal candidates tried in priority order.",
            ),
            DeclareLaunchArgument(
                "adjusted_goal_iterations",
                default_value="8000",
                description="RRT sample budget shared by all adjusted-goal candidates.",
            ),
            DeclareLaunchArgument(
                "visualize_tree",
                default_value="true",
                description="Publish RRT tree edges as an RViz marker.",
            ),
            DeclareLaunchArgument(
                "visualize_sampling_process",
                default_value="true",
                description="Record planning events and replay the RRT sampling process in RViz.",
            ),
            DeclareLaunchArgument(
                "replay_interval",
                default_value="0.03",
                description="Seconds between sampling-process replay updates.",
            ),
            DeclareLaunchArgument(
                "replay_events_per_tick",
                default_value="20",
                description="Number of recorded events displayed per replay update.",
            ),
            DeclareLaunchArgument(
                "max_visualized_samples",
                default_value="10000",
                description="Maximum number of RRT events retained for replay.",
            ),
            DeclareLaunchArgument(
                "max_iterations",
                default_value="5000",
                description="Maximum number of RRT samples.",
            ),
            DeclareLaunchArgument(
                "step_size",
                default_value="0.30",
                description="Initial/maximum adaptive local sampling radius in meters.",
            ),
            DeclareLaunchArgument(
                "rewire_radius",
                default_value="1.00",
                description="RRT* neighborhood radius used for parent selection and rewiring.",
            ),
            DeclareLaunchArgument(
                "minimum_sampling_radius",
                default_value="0.075",
                description="Smallest local sampling radius tried after expansion failures.",
            ),
            DeclareLaunchArgument(
                "radius_decay",
                default_value="0.5",
                description="Exponential radius multiplier after no valid local sample is found.",
            ),
            DeclareLaunchArgument(
                "samples_per_radius",
                default_value="3",
                description="Local candidate attempts before shrinking the sampling radius.",
            ),
            DeclareLaunchArgument(
                "goal_sample_rate",
                default_value="0.15",
                description="Probability of sampling the goal directly.",
            ),
            DeclareLaunchArgument(
                "goal_tolerance",
                default_value="0.05",
                description="Distance in meters at which the RRT goal is considered reached.",
            ),
            DeclareLaunchArgument(
                "random_seed",
                default_value="0",
                description="Fixed RRT random seed; zero selects nondeterministic sampling.",
            ),
            DeclareLaunchArgument(
                "smooth_path",
                default_value="false",
                description="Shortcut-smooth the RRT path before publishing.",
            ),
            DeclareLaunchArgument(
                "smoothing_iterations",
                default_value="100",
                description="Number of random shortcut attempts used for path smoothing.",
            ),
            DeclareLaunchArgument(
                "min_waypoint_spacing",
                default_value="2.0",
                description="Minimum preferred spacing between published global waypoints in meters.",
            ),
            Node(
                package="p03_rviz_route_planner",
                executable="rrt_global_planner",
                name="rrt_global_planner",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"map_topic": LaunchConfiguration("map_topic")},
                    {"sampling_process_topic": LaunchConfiguration("sampling_process_topic")},
                    {"pose_topic": LaunchConfiguration("pose_topic")},
                    {"pose_frame_override": LaunchConfiguration("pose_frame_override")},
                    {"goal_pose_topic": LaunchConfiguration("goal_pose_topic")},
                    {"inflation_radius": LaunchConfiguration("inflation_radius")},
                    {"adjust_goal_if_blocked": LaunchConfiguration("adjust_goal_if_blocked")},
                    {"fallback_on_rrt_timeout": LaunchConfiguration("fallback_on_rrt_timeout")},
                    {"max_adjusted_goal_candidates": LaunchConfiguration("max_adjusted_goal_candidates")},
                    {"adjusted_goal_attempts": LaunchConfiguration("adjusted_goal_attempts")},
                    {"adjusted_goal_iterations": LaunchConfiguration("adjusted_goal_iterations")},
                    {"visualize_tree": LaunchConfiguration("visualize_tree")},
                    {"visualize_sampling_process": LaunchConfiguration("visualize_sampling_process")},
                    {"replay_interval": LaunchConfiguration("replay_interval")},
                    {"replay_events_per_tick": LaunchConfiguration("replay_events_per_tick")},
                    {"max_visualized_samples": LaunchConfiguration("max_visualized_samples")},
                    {"max_iterations": LaunchConfiguration("max_iterations")},
                    {"step_size": LaunchConfiguration("step_size")},
                    {"rewire_radius": LaunchConfiguration("rewire_radius")},
                    {"minimum_sampling_radius": LaunchConfiguration("minimum_sampling_radius")},
                    {"radius_decay": LaunchConfiguration("radius_decay")},
                    {"samples_per_radius": LaunchConfiguration("samples_per_radius")},
                    {"goal_sample_rate": LaunchConfiguration("goal_sample_rate")},
                    {"goal_tolerance": LaunchConfiguration("goal_tolerance")},
                    {"random_seed": LaunchConfiguration("random_seed")},
                    {"smooth_path": LaunchConfiguration("smooth_path")},
                    {"smoothing_iterations": LaunchConfiguration("smoothing_iterations")},
                    {"min_waypoint_spacing": LaunchConfiguration("min_waypoint_spacing")},
                ],
            ),
        ]
    )
