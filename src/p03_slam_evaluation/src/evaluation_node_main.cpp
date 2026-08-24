#include <memory>

#include "p03_slam_evaluation/evaluation_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<p03_slam_evaluation::EvaluationNode>());
  rclcpp::shutdown();
  return 0;
}
