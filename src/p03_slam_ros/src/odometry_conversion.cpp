#include "p03_slam_ros/odometry_conversion.hpp"

#include <cstdint>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace p03_slam_ros
{

// Copies the timestamp and converts the three-dimensional pose to Pose2D.
p03_slam_core::Odometry2D toCoreOdometry(const nav_msgs::msg::Odometry & message)
{
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

  p03_slam_core::Odometry2D odometry;
  odometry.timestamp_nanoseconds =
    static_cast<std::int64_t>(message.header.stamp.sec) * kNanosecondsPerSecond +
    static_cast<std::int64_t>(message.header.stamp.nanosec);

  const geometry_msgs::msg::Pose & pose = message.pose.pose;
  odometry.odom_from_base.x = pose.position.x;
  odometry.odom_from_base.y = pose.position.y;
  odometry.odom_from_base.yaw = tf2::getYaw(pose.orientation);   //use tf tool here to convert the quatation to yaw 
  return odometry;
}

}  // namespace p03_slam_ros
