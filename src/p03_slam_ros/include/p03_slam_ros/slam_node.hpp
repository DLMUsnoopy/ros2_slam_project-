#pragma once

#include <cstddef>
#include <deque>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "p03_slam_core/odometry_buffer2d.hpp"
#include "p03_slam_core/slam_frontend2d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
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

  // Processes queued scans once odometry brackets their timestamps.
  void processPendingScans();

  // Processes one time-synchronized laser scan.
  void processScan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & message,
    const p03_slam_core::Pose2D & odom_from_base);

  // Publishes the map at a bounded rate when new map data is available.
  void publishMap();

  std::deque<sensor_msgs::msg::LaserScan::ConstSharedPtr> pending_scans_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  const std::string base_frame_;
  const std::string map_frame_;
  const std::string odom_frame_;
  const double minimum_usable_range_;
  const double maximum_mapping_range_;
  const std::size_t max_pending_scans_;
  const bool publish_odom_tf_;
  p03_slam_core::OdometryBuffer2D odometry_buffer_;
  p03_slam_core::SlamFrontend2D frontend_;
  p03_slam_core::Pose2D map_from_odom_;
  bool map_dirty_{false};

  // Stores timestamped transforms received from TF.
  tf2_ros::Buffer tf_buffer_;

  // Receives transforms and writes them into the buffer declared above.
  tf2_ros::TransformListener tf_listener_;

  // Publishes map-to-odometry and optional odometry-to-base transforms.
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace p03_slam_ros
