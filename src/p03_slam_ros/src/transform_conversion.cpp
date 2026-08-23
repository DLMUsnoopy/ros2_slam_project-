#include "p03_slam_ros/transform_conversion.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace p03_slam_ros
{

// Copies planar translation and extracts yaw from the quaternion.
p03_slam_core::Pose2D toCorePose2D(
  const geometry_msgs::msg::TransformStamped & message)
{
  p03_slam_core::Pose2D pose;
  pose.x = message.transform.translation.x;
  pose.y = message.transform.translation.y;
  pose.yaw = tf2::getYaw(message.transform.rotation);
  return pose;
}

}  // namespace p03_slam_ros
