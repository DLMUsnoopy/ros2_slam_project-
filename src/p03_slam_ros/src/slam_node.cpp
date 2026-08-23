#include "p03_slam_ros/slam_node.hpp"

#include <functional>
#include <string>

#include "p03_slam_ros/laser_scan_conversion.hpp"

namespace p03_slam_ros
{

// Configures the node and creates its laser scan subscription.
SlamNode::SlamNode()
: rclcpp::Node("p03_slam")
{
  const std::string scan_topic = declare_parameter<std::string>(
    "scan_topic", "lidar_2d");

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&SlamNode::scanCallback, this, std::placeholders::_1));
}

// Converts one ROS scan and reports its beam count at a limited rate.
void SlamNode::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
  const p03_slam_core::LaserScan2D scan = toCoreLaserScan(*message);

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "Received laser scan with %zu beams.",
    scan.ranges.size());
}

}  // namespace p03_slam_ros
