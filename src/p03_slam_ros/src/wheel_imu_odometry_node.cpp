#include "p03_slam_ros/wheel_imu_odometry_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iterator>

namespace p03_slam_ros
{
namespace
{

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

// Converts a ROS timestamp to nanoseconds.
std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(stamp.nanosec);
}

// Keeps an angle in the range [-pi, pi].
double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

WheelImuOdometryNode::WheelImuOdometryNode()
: rclcpp::Node("wheel_imu_odometry"),
  odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
  base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
  left_wheel_joints_(declare_parameter<std::vector<std::string>>(
      "left_wheel_joints", {"front_left_wheel", "rear_left_wheel"})),
  right_wheel_joints_(declare_parameter<std::vector<std::string>>(
      "right_wheel_joints", {"front_right_wheel", "rear_right_wheel"})),
  wheel_radius_(declare_parameter<double>("wheel_radius", 0.098)),
  wheel_separation_(declare_parameter<double>("wheel_separation", 0.37559)),
  imu_timeout_nanoseconds_(static_cast<std::int64_t>(
      declare_parameter<double>("imu_timeout", 0.15) * kNanosecondsPerSecond))
{
  const std::string joint_states_topic = declare_parameter<std::string>(
    "joint_states_topic", "joint_states");
  const std::string imu_topic = declare_parameter<std::string>("imu_topic", "imu");
  const std::string odometry_topic = declare_parameter<std::string>(
    "odometry_topic", "wheel_imu_odom");

  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(100);
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic,
    sensor_qos,
    std::bind(&WheelImuOdometryNode::imuCallback, this, std::placeholders::_1));
  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic,
    sensor_qos,
    std::bind(&WheelImuOdometryNode::jointStateCallback, this, std::placeholders::_1));
  odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic, 20);

  RCLCPP_INFO(
    get_logger(),
    "Wheel-IMU odometry: joints=%s, imu=%s, output=%s.",
    joint_states_topic.c_str(), imu_topic.c_str(), odometry_topic.c_str());
}

void WheelImuOdometryNode::imuCallback(
  const sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  if (!std::isfinite(message->angular_velocity.z)) {
    return;
  }

  imu_yaw_rate_ = message->angular_velocity.z;
  imu_timestamp_nanoseconds_ = stampNanoseconds(message->header.stamp);
}

bool WheelImuOdometryNode::meanJointPosition(
  const sensor_msgs::msg::JointState & message,
  const std::vector<std::string> & joint_names,
  double & mean_position) const
{
  if (joint_names.empty()) {
    return false;
  }

  double position_sum = 0.0;
  for (const std::string & joint_name : joint_names) {
    const auto joint = std::find(message.name.begin(), message.name.end(), joint_name);
    if (joint == message.name.end()) {
      return false;
    }

    const auto index = static_cast<std::size_t>(std::distance(message.name.begin(), joint));
    if (index >= message.position.size() || !std::isfinite(message.position[index])) {
      return false;
    }
    position_sum += message.position[index];
  }

  mean_position = position_sum / static_cast<double>(joint_names.size());
  return true;
}

void WheelImuOdometryNode::jointStateCallback(
  const sensor_msgs::msg::JointState::ConstSharedPtr message)
{
  const std::int64_t timestamp_nanoseconds = stampNanoseconds(message->header.stamp);
  if (timestamp_nanoseconds <= 0) {
    return;
  }

  double left_position = 0.0;
  double right_position = 0.0;
  if (!meanJointPosition(*message, left_wheel_joints_, left_position) ||
    !meanJointPosition(*message, right_wheel_joints_, right_position))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Configured wheel joints are missing from joint_states.");
    return;
  }

  if (!initialized_) {
    last_left_position_ = left_position;
    last_right_position_ = right_position;
    last_joint_timestamp_nanoseconds_ = timestamp_nanoseconds;
    initialized_ = true;
    publishOdometry(message->header.stamp);
    return;
  }

  const double delta_time = static_cast<double>(
    timestamp_nanoseconds - last_joint_timestamp_nanoseconds_) / kNanosecondsPerSecond;
  if (delta_time <= 0.0 || delta_time > 0.5) {
    last_left_position_ = left_position;
    last_right_position_ = right_position;
    last_joint_timestamp_nanoseconds_ = timestamp_nanoseconds;
    return;
  }

  const double left_travel = (left_position - last_left_position_) * wheel_radius_;
  const double right_travel = (right_position - last_right_position_) * wheel_radius_;
  const double linear_travel = 0.5 * (left_travel + right_travel);
  const double wheel_yaw_change = (right_travel - left_travel) / wheel_separation_;
  const bool imu_is_fresh = imu_timestamp_nanoseconds_ > 0 &&
    std::llabs(timestamp_nanoseconds - imu_timestamp_nanoseconds_) <=
    imu_timeout_nanoseconds_;
  const double yaw_change = imu_is_fresh ? imu_yaw_rate_ * delta_time : wheel_yaw_change;

  const double middle_yaw = yaw_ + 0.5 * yaw_change;
  x_ += linear_travel * std::cos(middle_yaw);
  y_ += linear_travel * std::sin(middle_yaw);
  yaw_ = normalizeAngle(yaw_ + yaw_change);
  linear_velocity_ = linear_travel / delta_time;
  angular_velocity_ = yaw_change / delta_time;

  last_left_position_ = left_position;
  last_right_position_ = right_position;
  last_joint_timestamp_nanoseconds_ = timestamp_nanoseconds;
  publishOdometry(message->header.stamp);
}

void WheelImuOdometryNode::publishOdometry(const builtin_interfaces::msg::Time & stamp)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = odom_frame_;
  message.child_frame_id = base_frame_;
  message.pose.pose.position.x = x_;
  message.pose.pose.position.y = y_;
  message.pose.pose.orientation.z = std::sin(0.5 * yaw_);
  message.pose.pose.orientation.w = std::cos(0.5 * yaw_);
  message.twist.twist.linear.x = linear_velocity_;
  message.twist.twist.angular.z = angular_velocity_;
  odometry_publisher_->publish(message);
}

}  // namespace p03_slam_ros
