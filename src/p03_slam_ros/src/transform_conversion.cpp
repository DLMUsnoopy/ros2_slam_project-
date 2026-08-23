#include "p03_slam_ros/transform_conversion.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
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

// Copies planar translation and converts yaw into a quaternion.
geometry_msgs::msg::TransformStamped toRosTransformStamped(
  const p03_slam_core::Pose2D & pose,
  const std::string & parent_frame,
  const std::string & child_frame)
{
  geometry_msgs::msg::TransformStamped message;
  message.header.frame_id = parent_frame;
  message.child_frame_id = child_frame;
  message.transform.translation.x = pose.x;
  message.transform.translation.y = pose.y;

  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, pose.yaw);
  message.transform.rotation = tf2::toMsg(rotation);
  return message;
}

}  // namespace p03_slam_ros
