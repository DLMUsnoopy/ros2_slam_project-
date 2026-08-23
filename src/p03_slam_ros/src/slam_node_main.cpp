#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "p03_slam_ros/slam_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<p03_slam_ros::SlamNode>());
  rclcpp::shutdown();
  return 0;
}
