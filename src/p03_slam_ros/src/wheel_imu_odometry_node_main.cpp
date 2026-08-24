#include <memory>

#include "p03_slam_ros/wheel_imu_odometry_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<p03_slam_ros::WheelImuOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
