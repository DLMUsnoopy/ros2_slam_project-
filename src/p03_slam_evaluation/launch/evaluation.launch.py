from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_namespace = LaunchConfiguration("namespace")
    parameters_file = LaunchConfiguration("params_file")
    plot_errors = LaunchConfiguration("plot_errors")

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
            DeclareLaunchArgument(
                "plot_errors",
                default_value="true",
                description="Open live translation and yaw error line graphs with rqt_plot.",
            ),
            Node(
                package="p03_slam_evaluation",
                executable="slam_evaluation_node",
                namespace=robot_namespace,
                parameters=[parameters_file],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "rqt_plot",
                    "rqt_plot",
                    PathJoinSubstitution(
                        ["/", robot_namespace, "evaluation/errors/translation/current/data"]
                    ),
                    PathJoinSubstitution(
                        ["/", robot_namespace, "evaluation/errors/translation/rmse/data"]
                    ),
                    PathJoinSubstitution(
                        ["/", robot_namespace, "evaluation/errors/translation/max/data"]
                    ),
                ],
                condition=IfCondition(plot_errors),
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "rqt_plot",
                    "rqt_plot",
                    PathJoinSubstitution(
                        ["/", robot_namespace, "evaluation/errors/yaw/current_deg/data"]
                    ),
                    PathJoinSubstitution(
                        ["/", robot_namespace, "evaluation/errors/yaw/rmse_deg/data"]
                    ),
                    PathJoinSubstitution(
                        ["/", robot_namespace, "evaluation/errors/yaw/max_deg/data"]
                    ),
                ],
                condition=IfCondition(plot_errors),
                output="screen",
            ),
        ]
    )
