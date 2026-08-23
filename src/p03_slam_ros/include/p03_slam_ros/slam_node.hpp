#pragma once

#include <optional>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "p03_slam_core/odometry2d.hpp"
#include "p03_slam_core/slam_frontend2d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace p03_slam_ros
{

// Connects ROS 2 sensor messages to the core SLAM pipeline.
class SlamNode : public rclcpp::Node
{
public:
  SlamNode();

private:
  // Handles each incoming wheel-odometry message.
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message);

  // Handles each incoming planar laser scan.
  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message);

  // Stores the latest core odometry after the first message arrives.
  std::optional<p03_slam_core::Odometry2D> latest_odometry_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  const std::string base_frame_;
  p03_slam_core::SlamFrontend2D frontend_;

  // Stores timestamped transforms received from TF.
  tf2_ros::Buffer tf_buffer_;

  // Receives transforms and writes them into the buffer declared above.
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace p03_slam_ros
