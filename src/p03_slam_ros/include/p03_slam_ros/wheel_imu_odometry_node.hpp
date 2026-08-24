#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace p03_slam_ros
{

// Estimates planar odometry from wheel travel and IMU yaw rate.
class WheelImuOdometryNode : public rclcpp::Node
{
public:
  WheelImuOdometryNode();

private:
  // Stores the latest valid yaw rate reported by the IMU.
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr message);

  // Integrates wheel travel using the latest time-aligned IMU yaw rate.
  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr message);

  // Returns the mean position of the configured wheel joints.
  bool meanJointPosition(
    const sensor_msgs::msg::JointState & message,
    const std::vector<std::string> & joint_names,
    double & mean_position) const;

  // Publishes the current planar odometry estimate.
  void publishOdometry(const builtin_interfaces::msg::Time & stamp);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

  std::string odom_frame_;
  std::string base_frame_;
  std::vector<std::string> left_wheel_joints_;
  std::vector<std::string> right_wheel_joints_;
  double wheel_radius_{0.098};
  double wheel_separation_{0.37559};
  std::int64_t imu_timeout_nanoseconds_{150000000};

  std::int64_t imu_timestamp_nanoseconds_{0};
  std::int64_t last_joint_timestamp_nanoseconds_{0};
  double imu_yaw_rate_{0.0};
  double last_left_position_{0.0};
  double last_right_position_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double linear_velocity_{0.0};
  double angular_velocity_{0.0};
  bool initialized_{false};
};

}  // namespace p03_slam_ros
