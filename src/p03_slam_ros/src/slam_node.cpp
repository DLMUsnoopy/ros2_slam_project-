#include "p03_slam_ros/slam_node.hpp"

#include <functional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "p03_slam_core/pose2d.hpp"
#include "p03_slam_ros/laser_scan_conversion.hpp"
#include "p03_slam_ros/odometry_conversion.hpp"
#include "p03_slam_ros/transform_conversion.hpp"
#include "tf2/exceptions.hpp"

namespace p03_slam_ros
{

// Configures the node and creates its laser scan subscription.
SlamNode::SlamNode()
: rclcpp::Node("p03_slam"),
  base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_, this, false)
{
  const std::string odometry_topic = declare_parameter<std::string>(
    "odometry_topic", "wheel_odom");
  const std::string scan_topic = declare_parameter<std::string>(
    "scan_topic", "lidar_2d");

  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&SlamNode::odometryCallback, this, std::placeholders::_1));

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&SlamNode::scanCallback, this, std::placeholders::_1));
}

// Converts and stores the latest wheel-odometry pose.
void SlamNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  latest_odometry_ = toCoreOdometry(*message);

  // The arrow operator accesses the Odometry2D stored inside the optional.
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "Received odometry: x=%.3f, y=%.3f, yaw=%.3f.",
    latest_odometry_->odom_from_base.x,
    latest_odometry_->odom_from_base.y,
    latest_odometry_->odom_from_base.yaw);
}

// Builds the odometry-based initial pose for each incoming laser scan.
void SlamNode::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
  if (!latest_odometry_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Skipping laser scan because no odometry has arrived yet.");
    return;
  }

  const p03_slam_core::LaserScan2D scan = toCoreLaserScan(*message);

  // This transform converts points from the lidar frame into the base frame.
  geometry_msgs::msg::TransformStamped base_from_lidar_message;
  try {
    base_from_lidar_message = tf_buffer_.lookupTransform(
      base_frame_, message->header.frame_id, message->header.stamp);
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Cannot transform laser scan from '%s' to '%s': %s",
      message->header.frame_id.c_str(),
      base_frame_.c_str(),
      exception.what());
    return;
  }

  const p03_slam_core::Pose2D base_from_lidar = toCorePose2D(base_from_lidar_message);
  const p03_slam_core::Pose2D odom_from_lidar = p03_slam_core::compose(
    latest_odometry_->odom_from_base, base_from_lidar);

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "Received %zu laser beams; odom_from_lidar=(%.3f, %.3f, %.3f).",
    scan.ranges.size(),
    odom_from_lidar.x,
    odom_from_lidar.y,
    odom_from_lidar.yaw);
}

}  // namespace p03_slam_ros
