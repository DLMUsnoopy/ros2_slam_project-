#pragma once

#include <cstddef>
#include <deque>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "p03_slam_core/odometry_buffer2d.hpp"
#include "p03_slam_evaluation/trajectory_evaluator2d.hpp"
#include "rclcpp/rclcpp.hpp"

namespace p03_slam_evaluation
{

// Compares time-synchronized SLAM poses with simulation ground truth.
class EvaluationNode : public rclcpp::Node
{
public:
  EvaluationNode();

private:
  // Stores each ground-truth sample for timestamp interpolation.
  void groundTruthCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message);

  // Queues each SLAM pose until matching ground truth is available.
  void estimatedPoseCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr message);

  // Evaluates all queued poses that are bracketed by ground truth.
  void processPendingEstimates();

  // Publishes bounded paths and prints the latest accumulated metrics.
  void publishEvaluation();

  // Adds a pose and removes the oldest point when the path is full.
  void appendPose(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & pose);

  std::deque<geometry_msgs::msg::PoseStamped::ConstSharedPtr> pending_estimates_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    estimated_pose_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr estimated_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ground_truth_path_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  const std::size_t max_pending_estimates_;
  const std::size_t max_path_points_;
  const std::string map_frame_;
  p03_slam_core::OdometryBuffer2D ground_truth_buffer_;
  TrajectoryEvaluator2D evaluator_;
  nav_msgs::msg::Path estimated_path_;
  nav_msgs::msg::Path aligned_ground_truth_path_;
};

}  // namespace p03_slam_evaluation
