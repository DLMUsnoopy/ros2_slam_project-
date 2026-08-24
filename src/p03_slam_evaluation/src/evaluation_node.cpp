#include "p03_slam_evaluation/evaluation_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "p03_slam_core/odometry2d.hpp"
#include "p03_slam_core/pose2d.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace p03_slam_evaluation
{
namespace
{

constexpr double kNanosecondsPerSecond = 1.0e9;
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

// Reads parameters used by the ground-truth interpolation buffer.
p03_slam_core::OdometryBufferConfig makeGroundTruthBufferConfig(rclcpp::Node & node)
{
  p03_slam_core::OdometryBufferConfig config;
  config.buffer_duration_nanoseconds = static_cast<std::int64_t>(
    node.declare_parameter<double>("ground_truth_buffer_duration", 5.0) *
    kNanosecondsPerSecond);
  config.max_interpolation_gap_nanoseconds = static_cast<std::int64_t>(
    node.declare_parameter<double>("max_ground_truth_interpolation_gap", 0.2) *
    kNanosecondsPerSecond);
  return config;
}

// Converts a ROS timestamp to nanoseconds.
std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

// Converts a ROS pose into its planar representation.
p03_slam_core::Pose2D toPose2D(const geometry_msgs::msg::Pose & pose)
{
  return {pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)};
}

// Creates a stamped ROS pose for path visualization.
geometry_msgs::msg::PoseStamped toPoseStamped(
  const p03_slam_core::Pose2D & pose,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = frame;
  message.pose.position.x = pose.x;
  message.pose.position.y = pose.y;

  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, pose.yaw);
  message.pose.orientation = tf2::toMsg(rotation);
  return message;
}

}  // namespace

EvaluationNode::EvaluationNode()
: rclcpp::Node("slam_evaluation"),
  max_pending_estimates_(static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("max_pending_estimates", 50)))),
  max_path_points_(static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("max_path_points", 2000)))),
  map_frame_(declare_parameter<std::string>("map_frame", "map")),
  ground_truth_buffer_(makeGroundTruthBufferConfig(*this))
{
  const std::string estimated_pose_topic = declare_parameter<std::string>(
    "estimated_pose_topic", "slam_pose");
  const std::string ground_truth_topic = declare_parameter<std::string>(
    "ground_truth_topic", "odom_gt");
  const std::string estimated_path_topic = declare_parameter<std::string>(
    "estimated_path_topic", "evaluation/estimated_path");
  const std::string ground_truth_path_topic = declare_parameter<std::string>(
    "ground_truth_path_topic", "evaluation/ground_truth_path");
  const double publish_rate = std::max(
    0.1, declare_parameter<double>("publish_rate", 1.0));

  rclcpp::QoS path_qos(rclcpp::KeepLast(1));
  path_qos.reliable();
  path_qos.transient_local();
  estimated_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
    estimated_path_topic, path_qos);
  ground_truth_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
    ground_truth_path_topic, path_qos);

  ground_truth_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    ground_truth_topic,
    rclcpp::SensorDataQoS().keep_last(100),
    std::bind(&EvaluationNode::groundTruthCallback, this, std::placeholders::_1));
  estimated_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    estimated_pose_topic,
    rclcpp::SensorDataQoS().keep_last(50),
    std::bind(&EvaluationNode::estimatedPoseCallback, this, std::placeholders::_1));

  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate),
    std::bind(&EvaluationNode::publishEvaluation, this));

  estimated_path_.header.frame_id = map_frame_;
  aligned_ground_truth_path_.header.frame_id = map_frame_;

  RCLCPP_INFO(
    get_logger(),
    "SLAM evaluation: estimate=%s, ground_truth=%s.",
    estimated_pose_topic.c_str(), ground_truth_topic.c_str());
}

void EvaluationNode::groundTruthCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  p03_slam_core::Odometry2D ground_truth;
  ground_truth.timestamp_nanoseconds = stampNanoseconds(message->header.stamp);
  ground_truth.odom_from_base = toPose2D(message->pose.pose);
  ground_truth_buffer_.add(ground_truth);
  processPendingEstimates();
}

void EvaluationNode::estimatedPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
{
  if (pending_estimates_.size() >= max_pending_estimates_) {
    pending_estimates_.pop_front();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Dropping the oldest estimated pose because the queue is full.");
  }
  pending_estimates_.push_back(message);
  processPendingEstimates();
}

void EvaluationNode::processPendingEstimates()
{
  while (!pending_estimates_.empty()) {
    const auto & estimate_message = pending_estimates_.front();
    const std::int64_t estimate_timestamp = stampNanoseconds(
      estimate_message->header.stamp);
    const std::optional<std::int64_t> oldest_ground_truth =
      ground_truth_buffer_.oldestTimestamp();
    const std::optional<std::int64_t> newest_ground_truth =
      ground_truth_buffer_.newestTimestamp();

    if (!oldest_ground_truth || !newest_ground_truth ||
      estimate_timestamp > *newest_ground_truth)
    {
      return;
    }

    if (estimate_timestamp < *oldest_ground_truth) {
      pending_estimates_.pop_front();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping an estimated pose older than the ground-truth buffer.");
      continue;
    }

    const std::optional<p03_slam_core::Pose2D> ground_truth_pose =
      ground_truth_buffer_.poseAt(estimate_timestamp);
    if (!ground_truth_pose) {
      pending_estimates_.pop_front();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping an estimated pose because ground truth cannot be interpolated.");
      continue;
    }

    const p03_slam_core::Pose2D estimated_pose = toPose2D(estimate_message->pose);
    const TrajectoryEvaluation2D evaluation = evaluator_.addSample(
      estimated_pose, *ground_truth_pose);
    appendPose(estimated_path_, *estimate_message);
    appendPose(
      aligned_ground_truth_path_,
      toPoseStamped(
        evaluation.aligned_ground_truth,
        estimate_message->header.stamp,
        map_frame_));
    pending_estimates_.pop_front();
  }
}

void EvaluationNode::appendPose(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (path.poses.size() >= max_path_points_) {
    path.poses.erase(path.poses.begin());
  }
  path.poses.push_back(pose);
}

void EvaluationNode::publishEvaluation()
{
  const TrajectoryMetrics2D & metrics = evaluator_.metrics();
  if (metrics.sample_count == 0) {
    return;
  }

  const builtin_interfaces::msg::Time stamp = now();
  estimated_path_.header.stamp = stamp;
  aligned_ground_truth_path_.header.stamp = stamp;
  estimated_path_publisher_->publish(estimated_path_);
  ground_truth_path_publisher_->publish(aligned_ground_truth_path_);

  RCLCPP_INFO(
    get_logger(),
    "Evaluation: samples=%zu | translation current=%.3f m, RMSE=%.3f m, max=%.3f m | "
    "yaw current=%.2f deg, RMSE=%.2f deg, max=%.2f deg.",
    metrics.sample_count,
    metrics.latest_translation_error,
    metrics.translation_rmse,
    metrics.maximum_translation_error,
    metrics.latest_yaw_error * kRadiansToDegrees,
    metrics.yaw_rmse * kRadiansToDegrees,
    metrics.maximum_yaw_error * kRadiansToDegrees);
}

}  // namespace p03_slam_evaluation
