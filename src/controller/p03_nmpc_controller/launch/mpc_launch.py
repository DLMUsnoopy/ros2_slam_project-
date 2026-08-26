import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.descriptions import ParameterValue
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    pkg_share = get_package_share_directory("ipb_ros2_sim")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    # LaunchConfiguration handles resolved at launch time, declare once here
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_saved_map = LaunchConfiguration("publish_saved_map")
    start_live_mapping = LaunchConfiguration("start_live_mapping")
    map_yaml = LaunchConfiguration("map_yaml")

    paths = {
        "jackal": os.path.join(pkg_share, "description", "jackal", "urdf", "jackal.urdf.xacro"),
        "husky": os.path.join(pkg_share, "description", "husky", "urdf", "husky.urdf.xacro"),
    }

    # ------------------------------------------------------------------ #
    # map_server lifecycle node (only active when publish_saved_map=true)  #
    # ------------------------------------------------------------------ #
    map_server_node = LifecycleNode(package="nav2_map_server", executable="map_server", name="map_server", namespace="", output="screen", parameters=[{"use_sim_time": use_sim_time,"yaml_filename": map_yaml,}], condition=IfCondition(publish_saved_map))

    return LaunchDescription([
        # ---------------------------------------------------------------- #
        # Launch arguments — declared exactly once each                     #
        # ---------------------------------------------------------------- #
        DeclareLaunchArgument("use_sim_time",       default_value="true",          description="Flag to enable use_sim_time"),
        DeclareLaunchArgument("jackal",             default_value=paths["jackal"], description="Absolute path to jackal robot model file"),
        DeclareLaunchArgument("husky",              default_value=paths["husky"],  description="Absolute path to husky robot model file"),
        DeclareLaunchArgument("max_robot_count",    default_value="1",             description="Max number of robots to spawn defined in sim_config"),
        DeclareLaunchArgument("sim_config",         default_value="indoor",        description="Simulation config defining robots and world to be spawned"),
        DeclareLaunchArgument("rviz",               default_value="true",          description="Whether to launch RViz2"),
        DeclareLaunchArgument("namespace",          default_value="",              description="Optional robot namespace, e.g. P1_robot0."),
        DeclareLaunchArgument("default_frame_id",   default_value="map",           description="Frame used when clicked points do not contain a frame id."),
        DeclareLaunchArgument("publish_saved_map",  default_value="true",          description="Publish the saved occupancy map on /map."),
        DeclareLaunchArgument("start_live_mapping", default_value="false",         description="Start the live occupancy-grid creation nodes."),
        DeclareLaunchArgument("map_yaml",           default_value=PathJoinSubstitution([
                FindPackageShare("occu_map_creation"),
                "maps",
                "occupancy_map.yaml",
            ]),
            description="Path to the saved occupancy-map YAML file."),

        # ---------------------------------------------------------------- #
        # Simulation: Gazebo + robots + bridges + ArUco localization        #
        # ---------------------------------------------------------------- #
        OpaqueFunction(function=launch_setup_gazebo, kwargs={"pkg_share": pkg_share}),
        OpaqueFunction(function=launch_setup_robots, kwargs={"pkg_share": pkg_share, "ros_gz_sim_share": ros_gz_sim_share}),

        # ---------------------------------------------------------------- #
        # Saved-map server (lifecycle: configure @ 1 s, activate @ 2 s)    #
        # ---------------------------------------------------------------- #
        map_server_node,

        TimerAction(period=1.0, actions=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                    condition=IfCondition(publish_saved_map),
                ),
            ],
        ),
        TimerAction(period=2.0, actions=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                    condition=IfCondition(publish_saved_map),
                ),
            ],
        ),

        # ---------------------------------------------------------------- #
        # Live mapping (only when start_live_mapping=true)                  #
        # ArUco localization needs ~1 s to publish the TF tree first.       #
        # ---------------------------------------------------------------- #
        # lidar_map_frame: delayed 2 s so ArUco has published map->odom first.
        # free_ray_max_range caps free-space ray length — without it rays trace
        # to infinity, double-count free cells, and push everything toward grey.
        TimerAction(period=2.0,
            actions=[
                Node(
                    package="occu_map_creation",
                    executable="lidar_map_frame",
                    name="lidar_map_frame",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "free_ray_max_range": 25.0,
                    }],
                    condition=IfCondition(start_live_mapping),
                ),
            ],
        ),
        # grid_manager: delayed 3 s (after lidar_map_frame is up).
        # Wider threshold gap (0.6 / 0.35) + higher hit_weight makes cells
        # commit to black/white decisively instead of staying grey.
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="occu_map_creation",
                    executable="grid_manager_node",
                    name="grid_manager_node",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "map_width": 900,
                        "map_height": 900,
                        "resolution": 0.04,
                        "origin_x": -10.0,
                        "origin_y": -10.0,
                        "frame_id": "map",
                        "input_topic": "/P1_robot0/lidar_map_data",
                        "map_topic": "/P1_robot0/occupancy_grid",
                        "publish_frequency": 2.0,
                        "min_observations": 2,
                        "occupied_threshold": 0.6,
                        "free_threshold": 0.35,
                        "hit_weight": 4,
                        "miss_weight": 1,
                        "occupied_cell_radius": 1,
                        "save_directory": "/tmp",
                        "map_name": "occupancy_map",
                    }],
                    condition=IfCondition(start_live_mapping),
                ),
            ],
        ),

        # ---------------------------------------------------------------- #
        # Route planner (RViz click → waypoint publisher)                   #
        # ---------------------------------------------------------------- #
        Node(
            package="p03_rviz_route_planner",
            executable="p03_rviz_route_planner",
            namespace=LaunchConfiguration("namespace"),
            name="p03_rviz_route_planner",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"default_frame_id": LaunchConfiguration("default_frame_id")},
            ],
        ),

        # ---------------------------------------------------------------- #
        # PID controller — delayed 3 s so localization is ready            #
        # FIX: package='p03_nmpc_controller', executable='pid_controller'  #
        # ---------------------------------------------------------------- #
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="p03_nmpc_controller",
                    executable="mpc_controller",
                    name="base_mpc_controller",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
            ],
        ),
    ])


# --------------------------------------------------------------------------- #
# Gazebo: server + GUI + env vars                                              #
# --------------------------------------------------------------------------- #
def launch_setup_gazebo(context, pkg_share, *args, **kwargs):
    sim_config = load_sim_config(context, pkg_share)
    world_path = os.path.join(pkg_share, "world", sim_config["world"])
    world_name = sim_config["world_name"]

    gz_env = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=generate_gz_resource_path(pkg_share, sim_config["world"]),
    )
    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name="ros_gz_container",
        create_own_container=True,
        use_composition=True,
    )
    gz_gui = ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen")

    return [
        gz_env,
        gz_server,
        gz_gui,
        DeclareLaunchArgument("world_name", default_value=world_name),
    ]


# --------------------------------------------------------------------------- #
# Robots: RSP, spawn, RViz, bridges, ArUco localization                       #
# --------------------------------------------------------------------------- #
def launch_setup_robots(context, pkg_share, ros_gz_sim_share, *args, **kwargs):
    max_robot_count = int(LaunchConfiguration("max_robot_count").perform(context))
    sim_config = load_sim_config(context, pkg_share)

    config_paths = {
        "bridge_robot": os.path.join(
            pkg_share, "config/bridge", sim_config["bridge_robot"]
        ),
        "bridge_robot_camera": os.path.join(
            pkg_share, "config/bridge", sim_config["bridge_robot_camera"]
        ),
        "bridge_sim": os.path.join(
            pkg_share, "config/bridge", sim_config["bridge_sim"]
        ),
        "rviz": os.path.join(pkg_share, "config/rviz", sim_config["rviz"]),
    }

    marker_map_file = os.path.join(
        get_package_share_directory("aruco_localization"),
        "config",
        "marker_map.yaml",
    )

    spawn_launch = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")

    with open(config_paths["rviz"], "r") as f:
        rviz_template = f.read()
    with open(config_paths["bridge_robot"], "r") as f:
        bridge_robot_template = f.read()
    with open(config_paths["bridge_robot_camera"], "r") as f:
        bridge_robot_camera_template = f.read()

    namespace_prefix = sim_config["namespace"]
    robotspace_prefix = sim_config["robotspace"]

    if namespace_prefix != "":
        namespace_prefix += "_"
    if robotspace_prefix != "":
        namespace_prefix += robotspace_prefix
    if namespace_prefix == "" and max_robot_count != 1:
        namespace_prefix = "ID"

    nodes = []
    bridge_entries = []
    bridge_camera_entries = []
    all_robot_ids = set()

    for idx, robot_data in zip(range(max_robot_count), sim_config["robots"]):
        model_type = robot_data["model_type"]
        pose = robot_data["init_pose"]
        robot_id = robot_data["id"]

        if robot_id in all_robot_ids:
            raise IndexError(f"ID {robot_id} is used multiple times")
        all_robot_ids.add(robot_id)

        if namespace_prefix != "":
            ns = f"{namespace_prefix}{robot_id}"
            slash_ns_slash = f"/{ns}/"
            ns_slash = f"{ns}/"
        else:
            ns = ""
            slash_ns_slash = ""
            ns_slash = ""

        # RViz config per robot
        rviz_path = f"/tmp/rviz_config_{robot_id}.yaml"
        rviz_config = yaml.safe_load(
            rviz_template.replace("robot/base_link", f"{ns_slash}base_link")
        )
        save_yaml(rviz_path, rviz_config)

        bridge_entries.append(bridge_robot_template.replace("robot/", ns_slash))
        bridge_camera_entries.append(
            bridge_robot_camera_template.replace("robot/", ns_slash)
        )

        rsp_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=ns,
            name="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(
                    Command([
                        "xacro ",
                        LaunchConfiguration(model_type),
                        f" robot_namespace:={slash_ns_slash}",
                        f" robot_id:={robot_id}",
                    ]),
                    value_type=str,
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            remappings=[("robot_description", f"{slash_ns_slash}robot_description")],
        )

        spawn_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                "world": LaunchConfiguration("world_name"),
                "topic": f"{slash_ns_slash}robot_description",
                "entity_name": f"{model_type}_{idx}",
                "x": pose["x"],
                "y": pose["y"],
                "z": pose["z"],
                "Y": pose["theta"],
            }.items(),
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            namespace=ns,
            name="rviz",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            arguments=["-d", rviz_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )

        # ArUco localization — TF tree:
        #   map -> {ns}/odom   (ArUco fusion)
        #   {ns}/odom -> base  (wheel + gyro dead-reckoning)
        joint_states_topic = f"/{ns}/joint_states" if ns else "/joint_states"
        imu_topic = f"/{ns}/imu" if ns else "/imu"
        odom_topic = f"/{ns}/esmt_odom" if ns else "/esmt_odom"
        global_pose_topic = f"/{ns}/esmt_pose" if ns else "/esmt_pose"

        ns_prefix = f"/{ns}" if ns else ""
        left_wheel_joints = [
            f"{ns_prefix}/front_left_wheel",
            f"{ns_prefix}/rear_left_wheel",
        ]
        right_wheel_joints = [
            f"{ns_prefix}/front_right_wheel",
            f"{ns_prefix}/rear_right_wheel",
        ]

        aruco_detector_node = Node(
            package="aruco_localization",
            executable="aruco_detector",
            name="aruco_detector",
            namespace=ns,
            output="screen",
            parameters=[{
                "camera_frame_id": "",  # auto-detected from camera_info
                "use_sim_time": True,
            }],
        )

        localization_node = Node(
            package="aruco_localization",
            executable="localization",
            name="localization",
            namespace=ns,
            output="screen",
            parameters=[{
                "map_frame": "map",
                "marker_map_file": marker_map_file,
                "wheel_radius": 0.075,
                "wheel_separation": 0.40,
                "left_wheel_joints": left_wheel_joints,
                "right_wheel_joints": right_wheel_joints,
                "use_gyro_for_yaw": True,
                "joint_states_topic": joint_states_topic,
                "imu_topic": imu_topic,
                "odom_topic": odom_topic,
                "global_pose_topic": global_pose_topic,
                "use_sim_time": True,
            }],
        )

        nodes.extend([rsp_node, spawn_node, rviz_node, aruco_detector_node, localization_node])

    # Simulation bridge (clock etc.) — only one instance!
    simulation_bridge_node = RosGzBridge(
        bridge_name="simulation_bridge",
        config_file=config_paths["bridge_sim"],
        container_name="ros_gz_container",
        create_own_container=False,
        use_composition=True,
        namespace="ros_gz_bridge",
    )
    nodes.append(simulation_bridge_node)

    # Robot bridge (no camera)
    robot_bridge_list = yaml.safe_load("\n".join(bridge_entries)) or []
    bridge_path = "/tmp/combined_robot_bridge.yaml"
    save_yaml(bridge_path, robot_bridge_list)

    robot_bridge_node = RosGzBridge(
        bridge_name="robot_bridge",
        config_file=bridge_path,
        container_name="ros_gz_container",
        create_own_container=False,
        use_composition=True,
        namespace="ros_gz_bridge",
    )
    nodes.append(robot_bridge_node)

    # Camera bridge (separate node — one per sim)
    robot_camera_bridge_list = yaml.safe_load("\n".join(bridge_camera_entries)) or []
    bridge_camera_path = "/tmp/combined_bridge_camera.yaml"
    save_yaml(bridge_camera_path, robot_camera_bridge_list)

    bridge_camera_node = Node(
        package="ipb_ros2_sim",
        executable="camera_bridge",
        name="camera_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_camera_path},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        namespace="ros_gz_bridge",
    )
    nodes.append(bridge_camera_node)

    return nodes


# --------------------------------------------------------------------------- #
# Helpers                                                                      #
# --------------------------------------------------------------------------- #
def load_sim_config(context, pkg_share):
    sim_config_name = str(LaunchConfiguration("sim_config").perform(context))
    sim_config_path = (
        os.path.join(pkg_share, "config/simulation", sim_config_name) + ".yaml"
    )
    return load_yaml(sim_config_path)


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def save_yaml(path, data):
    with open(path, "w") as f:
        yaml.safe_dump(data, f)


def generate_gz_resource_path(pkg_share, world):
    base = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    added = os.pathsep.join([
        os.path.join(pkg_share, os.path.pardir),
        os.path.join(pkg_share, "world", os.path.dirname(world)),
        os.path.join(pkg_share, "world", os.path.dirname(world), "models"),
    ])
    return f"{base}{os.pathsep}{added}" if base else added
