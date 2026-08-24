from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_namespace = LaunchConfiguration("namespace")
    parameters_file = LaunchConfiguration("params_file")

    default_parameters_file = PathJoinSubstitution(
        [FindPackageShare("p03_slam_evaluation"), "config", "indoor.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="P1_robot0",
                description="ROS namespace of the evaluated robot.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_parameters_file,
                description="Absolute path to the evaluation parameter file.",
            ),
            Node(
                package="p03_slam_evaluation",
                executable="slam_evaluation_node",
                namespace=robot_namespace,
                parameters=[parameters_file],
                output="screen",
            ),
        ]
    )
