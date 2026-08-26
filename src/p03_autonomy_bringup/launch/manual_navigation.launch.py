from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#star  ipb_ros2_sim  p03_slam_ros p03_rviz_route_planner p03_nmpc_controller
def generate_launch_description():
    start_sim = LaunchConfiguration("start_sim")
    start_slam = LaunchConfiguration("start_slam")
    enable_obstacles = LaunchConfiguration("enable_obstacles")
    enable_frontier = LaunchConfiguration("enable_frontier")
    sim_config = LaunchConfiguration("sim_config")
    rviz = LaunchConfiguration("rviz")
    params_file = LaunchConfiguration("params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")

    default_params = PathJoinSubstitution(
        [FindPackageShare("p03_autonomy_bringup"), "config", "manual_navigation.yaml"]
    )
    default_slam_params = PathJoinSubstitution(
        [FindPackageShare("p03_slam_ros"), "config", "indoor.yaml"]
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ipb_ros2_sim"), "launch", "all.launch.py"])
        ),
        condition=IfCondition(start_sim),
        launch_arguments={"sim_config": sim_config, "rviz": rviz}.items(),
    )

    # Keep the SLAM algorithm/configuration in p03_slam_ros unchanged.  These
    # launch-level overrides only connect its relative topics and frames to the
    # namespaced simulator interfaces.
    slam_interface = {
        "use_sim_time": True,
        "joint_states_topic": "joint_states",
        "imu_topic": "imu",
        "odometry_topic": "wheel_imu_odom",
        "scan_topic": "lidar_2d",
        "map_topic": "map",
        "pose_topic": "slam_pose",
        "map_frame": "map",
        "odom_frame": "P1_robot0/odom",
        "base_frame": "P1_robot0/base_link",
        "publish_odom_tf": True,
        "left_wheel_joints": [
            "/P1_robot0/front_left_wheel",
            "/P1_robot0/rear_left_wheel",
        ],
        "right_wheel_joints": [
            "/P1_robot0/front_right_wheel",
            "/P1_robot0/rear_right_wheel",
        ],
    }

    wheel_odometry = Node(
        package="p03_slam_ros",
        executable="wheel_imu_odometry_node",
        namespace="P1_robot0",
        output="screen",
        parameters=[slam_params_file, slam_interface],
        condition=IfCondition(start_slam),
    )

    slam = Node(
        package="p03_slam_ros",
        executable="p03_slam_node",
        namespace="P1_robot0",
        output="screen",
        parameters=[slam_params_file, slam_interface],
        condition=IfCondition(start_slam),
    )

    planner = Node(
        package="p03_rviz_route_planner",
        executable="astar_global_planner",
        name="astar_global_planner",
        output="screen",
        parameters=[params_file],
    )

    controller = Node(
        package="p03_nmpc_controller",
        executable="mpc_controller",
        name="p03_nmpc_controller",
        output="screen",
        parameters=[params_file],
    )

    obstacle_detector = Node(
        package="p03_obstacle_detector",
        executable="lidar_in_world_frame",
        name="p03_obstacle_detector",
        output="screen",
        parameters=[params_file],
        condition=IfCondition(enable_obstacles),
    )

    frontier = Node(
        package="p03_frontier_exploration",
        executable="frontier_explorer",
        name="p03_frontier_explorer",
        output="screen",
        parameters=[params_file],
        condition=IfCondition(enable_frontier),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_sim", default_value="true"),
            DeclareLaunchArgument("start_slam", default_value="true"),
            DeclareLaunchArgument("enable_obstacles", default_value="true"),
            DeclareLaunchArgument("enable_frontier", default_value="false"),
            DeclareLaunchArgument("sim_config", default_value="indoor"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
            simulation,
            wheel_odometry,
            slam,
            planner,
            obstacle_detector,
            controller,
            frontier,
        ]
    )
