#include "p03_slam_ros/slam_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "p03_slam_core/pose2d.hpp"
#include "p03_slam_ros/laser_scan_conversion.hpp"
#include "p03_slam_ros/occupancy_grid_conversion.hpp"
#include "p03_slam_ros/odometry_conversion.hpp"
#include "p03_slam_ros/transform_conversion.hpp"
#include "tf2/exceptions.hpp"

namespace p03_slam_ros
{
namespace
{

// Reads ROS parameters and creates the ROS-independent frontend configuration.
p03_slam_core::SlamFrontendConfig makeFrontendConfig(rclcpp::Node & node)
{
  constexpr double kPi = 3.14159265358979323846;
  p03_slam_core::SlamFrontendConfig config;
  config.occupancy_grid.width = static_cast<std::size_t>(
    node.declare_parameter<int>("map_width", 900));
  config.occupancy_grid.height = static_cast<std::size_t>(
    node.declare_parameter<int>("map_height", 900));
  config.occupancy_grid.resolution = node.declare_parameter<double>(
    "resolution", 0.04);
  config.occupancy_grid.origin.x = node.declare_parameter<double>(
    "origin_x", -10.0);
  config.occupancy_grid.origin.y = node.declare_parameter<double>(
    "origin_y", -10.0);
  config.occupancy_grid.min_log_odds = static_cast<float>(
    node.declare_parameter<double>("min_log_odds", -4.0));
  config.occupancy_grid.max_log_odds = static_cast<float>(
    node.declare_parameter<double>("max_log_odds", 4.0));

  config.inverse_sensor_model.free_space_evidence = static_cast<float>(
    node.declare_parameter<double>("free_log_odds", -0.4));
  config.inverse_sensor_model.occupied_space_evidence = static_cast<float>(
    node.declare_parameter<double>("occupied_log_odds", 0.85));
  config.inverse_sensor_model.occupied_radius = static_cast<int>(std::max<std::int64_t>(
      0, node.declare_parameter<std::int64_t>("occupied_radius", 1)));

  const double occupied_probability = node.declare_parameter<double>(
    "likelihood_occupied_probability", 0.65);
  config.likelihood_field.occupied_log_odds_threshold = static_cast<float>(
    std::log(occupied_probability / (1.0 - occupied_probability)));
  config.likelihood_field.max_obstacle_distance = static_cast<float>(
    node.declare_parameter<double>("likelihood_max_distance", 0.50));

  config.matcher.standard_deviation = node.declare_parameter<double>(
    "likelihood_sigma", 0.15);
  config.matcher.linear_search_window = node.declare_parameter<double>(
    "linear_search_window", 0.30);
  config.matcher.angular_search_window = node.declare_parameter<double>(
    "angular_search_window_deg", 10.0) * kPi / 180.0;
  config.matcher.linear_step = node.declare_parameter<double>(
    "linear_search_step", 0.05);
  config.matcher.angular_step = node.declare_parameter<double>(
    "angular_search_step_deg", 2.0) * kPi / 180.0;
  config.matcher.fine_linear_search_window = node.declare_parameter<double>(
    "fine_linear_search_window", 0.08);
  config.matcher.fine_angular_search_window = node.declare_parameter<double>(
    "fine_angular_search_window_deg", 3.0) * kPi / 180.0;
  config.matcher.fine_linear_step = node.declare_parameter<double>(
    "fine_linear_search_step", 0.02);
  config.matcher.fine_angular_step = node.declare_parameter<double>(
    "fine_angular_search_step_deg", 0.75) * kPi / 180.0;
  config.matcher.min_score = node.declare_parameter<double>("min_match_score", 0.12);
  config.matcher.min_score_improvement = node.declare_parameter<double>(
    "min_score_improvement", 0.03);
  config.matcher.maximum_range = node.declare_parameter<double>(
    "max_matching_range", 15.0);
  config.matcher.min_matched_points = static_cast<std::size_t>(std::max<std::int64_t>(
      1, node.declare_parameter<std::int64_t>("min_matched_points", 35)));
  config.matcher.beam_stride = static_cast<std::size_t>(std::max<std::int64_t>(
      1, node.declare_parameter<std::int64_t>("scan_downsample", 4)));

  config.correction_gain = node.declare_parameter<double>("correction_gain", 0.20);
  config.correction_deadband_translation = node.declare_parameter<double>(
    "correction_deadband_translation", 0.025);
  config.correction_deadband_rotation = node.declare_parameter<double>(
    "correction_deadband_rotation_deg", 1.0) * kPi / 180.0;
  config.bootstrap_scans = static_cast<std::size_t>(std::max<std::int64_t>(
      1, node.declare_parameter<std::int64_t>("bootstrap_scans", 20)));
  config.stationary_translation_threshold = node.declare_parameter<double>(
    "stationary_translation_threshold", 0.005);
  config.stationary_rotation_threshold = node.declare_parameter<double>(
    "stationary_rotation_threshold_deg", 0.30) * kPi / 180.0;
  config.stationary_map_update_interval = static_cast<std::size_t>(std::max<std::int64_t>(
      1, node.declare_parameter<std::int64_t>("stationary_map_update_interval", 10)));
  config.likelihood_update_interval = static_cast<std::size_t>(std::max<std::int64_t>(
      1, node.declare_parameter<std::int64_t>("likelihood_update_interval", 3)));
  config.maximum_angular_velocity_for_matching = node.declare_parameter<double>(
    "maximum_angular_velocity_for_matching", 0.25);
  return config;
}

// Reads the limits used to synchronize scans with timestamped odometry.
p03_slam_core::OdometryBufferConfig makeOdometryBufferConfig(rclcpp::Node & node)
{
  constexpr double nanoseconds_per_second = 1.0e9;
  p03_slam_core::OdometryBufferConfig config;
  config.buffer_duration_nanoseconds = static_cast<std::int64_t>(
    node.declare_parameter<double>("odometry_buffer_duration", 5.0) *
    nanoseconds_per_second);
  config.max_interpolation_gap_nanoseconds = static_cast<std::int64_t>(
    node.declare_parameter<double>("max_odometry_interpolation_gap", 0.2) *
    nanoseconds_per_second);
  return config;
}

}  // namespace

// Configures the node and creates its laser scan subscription.
SlamNode::SlamNode()
: rclcpp::Node("p03_slam"),
  base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
  map_frame_(declare_parameter<std::string>("map_frame", "map")),
  odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
  minimum_usable_range_(declare_parameter<double>("minimum_usable_range", 0.0)),
  maximum_mapping_range_(declare_parameter<double>("max_mapping_range", 20.0)),
  max_pending_scans_(static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("max_pending_scans", 20)))),
  publish_odom_tf_(declare_parameter<bool>("publish_odom_tf", false)),
  odometry_buffer_(makeOdometryBufferConfig(*this)),
  frontend_(makeFrontendConfig(*this)),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_, this, false),
  tf_broadcaster_(this)
{
  const std::string odometry_topic = declare_parameter<std::string>(
    "odometry_topic", "wheel_odom");
  const std::string map_topic = declare_parameter<std::string>(
    "map_topic", "map");
  const std::string pose_topic = declare_parameter<std::string>(
    "pose_topic", "slam_pose");
  const std::string scan_topic = declare_parameter<std::string>(
    "scan_topic", "lidar_2d");

  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.reliable();
  map_qos.transient_local();
  map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic, map_qos);
  pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

  const double map_publish_rate = std::max(
    0.1, declare_parameter<double>("map_publish_rate", 2.0));
  map_publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / map_publish_rate),
    std::bind(&SlamNode::publishMap, this));

  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic,
    rclcpp::SensorDataQoS().keep_last(100),
    std::bind(&SlamNode::odometryCallback, this, std::placeholders::_1));

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic,
    rclcpp::SensorDataQoS().keep_last(20),
    std::bind(&SlamNode::scanCallback, this, std::placeholders::_1));
}

// Adds odometry to the synchronization buffer and publishes current transforms.
void SlamNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  const p03_slam_core::Odometry2D odometry = toCoreOdometry(*message);
  odometry_buffer_.add(odometry);

  if (publish_odom_tf_) {
    geometry_msgs::msg::TransformStamped odom_from_base_message =
      toRosTransformStamped(odometry.odom_from_base, odom_frame_, base_frame_);
    odom_from_base_message.header.stamp = message->header.stamp;
    tf_broadcaster_.sendTransform(odom_from_base_message);
  }

  geometry_msgs::msg::TransformStamped map_from_odom_message =
    toRosTransformStamped(map_from_odom_, map_frame_, odom_frame_);
  map_from_odom_message.header.stamp = message->header.stamp;
  tf_broadcaster_.sendTransform(map_from_odom_message);

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "Received odometry: x=%.3f, y=%.3f, yaw=%.3f.",
    odometry.odom_from_base.x,
    odometry.odom_from_base.y,
    odometry.odom_from_base.yaw);

  processPendingScans();
}

// Queues a scan until odometry is available on both sides of its timestamp.
void SlamNode::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
  if (pending_scans_.size() >= max_pending_scans_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Dropping the oldest laser scan because the pending queue is full.");
    pending_scans_.pop_front();
  }
  pending_scans_.push_back(message);
  processPendingScans();
}

void SlamNode::processPendingScans()
{
  while (!pending_scans_.empty()) {
    const auto & message = pending_scans_.front();
    const std::int64_t scan_timestamp = rclcpp::Time(message->header.stamp).nanoseconds();
    const std::optional<std::int64_t> oldest_odometry = odometry_buffer_.oldestTimestamp();
    const std::optional<std::int64_t> newest_odometry = odometry_buffer_.newestTimestamp();

    if (!oldest_odometry || !newest_odometry || scan_timestamp > *newest_odometry) {
      return;
    }
    if (scan_timestamp < *oldest_odometry) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping a laser scan older than the odometry buffer.");
      pending_scans_.pop_front();
      continue;
    }

    const std::optional<p03_slam_core::Pose2D> odom_from_base =
      odometry_buffer_.poseAt(scan_timestamp);
    if (!odom_from_base) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping a laser scan because the surrounding odometry gap is too large.");
      pending_scans_.pop_front();
      continue;
    }

    processScan(message, *odom_from_base);
    pending_scans_.pop_front();
    return;
  }
}

// Converts and processes a scan with the odometry pose at the same timestamp.
void SlamNode::processScan(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & message,
  const p03_slam_core::Pose2D & odom_from_base)
{

  p03_slam_core::LaserScan2D scan = toCoreLaserScan(*message);
  scan.range_min = std::max(scan.range_min, minimum_usable_range_);
  scan.range_max = std::min(scan.range_max, maximum_mapping_range_);

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
    odom_from_base, base_from_lidar);
  const p03_slam_core::SlamUpdate2D update = frontend_.processScan(
    scan, odom_from_lidar);
  map_from_odom_ = update.map_from_odom;
  map_dirty_ = true;

  const p03_slam_core::Pose2D map_from_base = p03_slam_core::compose(
    map_from_odom_, odom_from_base);
  geometry_msgs::msg::PoseStamped pose_message = toRosPoseStamped(
    map_from_base, map_frame_);
  pose_message.header.stamp = message->header.stamp;
  pose_publisher_->publish(pose_message);

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "SLAM update: beams=%zu, attempted=%s, accepted=%s, corrected=%s, "
    "score=%.3f, prior=%.3f, points=%zu, "
    "map_from_lidar=(%.3f, %.3f, %.3f), map_from_odom=(%.3f, %.3f, %.3f).",
    scan.ranges.size(),
    update.scan_match_attempted ? "true" : "false",
    update.scan_match_accepted ? "true" : "false",
    update.scan_matched ? "true" : "false",
    update.score,
    update.prior_score,
    update.matched_point_count,
    update.map_from_lidar.x,
    update.map_from_lidar.y,
    update.map_from_lidar.yaw,
    update.map_from_odom.x,
    update.map_from_odom.y,
    update.map_from_odom.yaw);
}

void SlamNode::publishMap()
{
  if (!map_dirty_ || !frontend_.isInitialized()) {
    return;
  }

  nav_msgs::msg::OccupancyGrid map_message = toRosOccupancyGrid(
    frontend_.occupancyGrid());
  map_message.header.stamp = now();
  map_message.header.frame_id = map_frame_;
  map_publisher_->publish(map_message);
  map_dirty_ = false;
}

}  // namespace p03_slam_ros
