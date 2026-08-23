#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace p03_slam_ros
{

// Connects ROS 2 sensor messages to the core SLAM pipeline.
class SlamNode : public rclcpp::Node
{
public:
  SlamNode();

private:
  // Handles each incoming planar laser scan.
  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

}  // namespace p03_slam_ros
