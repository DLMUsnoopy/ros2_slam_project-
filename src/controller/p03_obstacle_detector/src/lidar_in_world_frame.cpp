#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

using namespace std::chrono_literals;

namespace{
    double yawFromQuat(const geometry_msgs::msg::Quaternion &q) {
        const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
        const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(sin_yaw, cos_yaw);
    }

    double wrapAngle(const double a) {
        return std::atan2(std::sin(a), std::cos(a));
    }
}

// Planar pose sample with timestamp, used for interpolation at scan time.
struct StampedPose2D {
    double t = 0.0;    // seconds
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

class LidarMapFrame : public rclcpp::Node {
    public:
        LidarMapFrame() : Node("lidar_map_frame") {
            this->declare_parameter<std::string>("pose_source", "pose_topic");
            this->declare_parameter<std::string>("pose_topic", "/P1_robot0/esmt_pose");
            this->declare_parameter<std::string>("map_frame", "map");
            this->declare_parameter<std::string>("base_frame", "P1_robot0/base_link");
            this->declare_parameter<std::string>("lidar_topic", "/P1_robot0/lidar_2d");
            this->declare_parameter<std::string>("output_topic", "/P1_robot0/lidar_map_data");
            this->declare_parameter<double>("free_ray_max_range", 25.0);

            this->declare_parameter<double>("max_pose_gap", 0.30);
            this->declare_parameter<double>("max_pose_extrapolation", 0.15);

            this->get_parameter("pose_source", pose_source_);
            this->get_parameter("pose_topic", pose_topic_);
            this->get_parameter("map_frame", map_frame_);
            this->get_parameter("base_frame", base_frame_);
            this->get_parameter("lidar_topic", lidar_topic_);
            this->get_parameter("output_topic", output_topic_);
            this->get_parameter("free_ray_max_range", free_ray_max_range_);
            this->get_parameter("max_pose_gap", max_pose_gap_);
            this->get_parameter("max_pose_extrapolation", max_pose_extrapolation_);

            if (pose_source_ != "pose_topic" && pose_source_ != "tf") {
                throw std::runtime_error("pose_source must be 'pose_topic' or 'tf'");
            }

            publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, 10);

            sub_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidar_topic_, rclcpp::SensorDataQoS(), std::bind(&LidarMapFrame::lidar_callback, this, std::placeholders::_1));

            if (pose_source_ == "pose_topic") {
                sub_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(pose_topic_, 50, std::bind(&LidarMapFrame::pose_callback, this, std::placeholders::_1));
            }

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            this->set_parameter(rclcpp::Parameter("use_sim_time", true));

            RCLCPP_INFO(this->get_logger(), "lidar_map_frame started. pose_source=%s%s%s", pose_source_.c_str(), pose_source_ == "pose_topic" ? ", pose_topic=" : "", pose_source_ == "pose_topic" ? pose_topic_.c_str() : "");
        }

    private:
      // ---------------------------------------------------------------- pose buffer
      void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            StampedPose2D s;
            s.t = rclcpp::Time(msg->header.stamp).seconds();
            s.x = msg->pose.pose.position.x;
            s.y = msg->pose.pose.position.y;
            s.yaw = yawFromQuat(msg->pose.pose.orientation);

            // Keep the buffer time-ordered; drop out-of-order samples.
            if (!pose_buffer_.empty() && s.t <= pose_buffer_.back().t) {
                return;
            }

            pose_buffer_.push_back(s);

            while (pose_buffer_.size() > 500) {
                pose_buffer_.pop_front();
            }
      }

      // Interpolate the base pose at time t. Returns false when the buffer
      // cannot bracket t closely enough for a trustworthy pose.
      bool poseAt(const double t, StampedPose2D &out) const {
          if (pose_buffer_.empty()) {
              return false;
          }

          const StampedPose2D &newest = pose_buffer_.back();

          if (t >= newest.t) {
            if (t - newest.t > max_pose_extrapolation_) {
                return false;
            }
              out = newest;
              out.t = t;
              return true;
          }

          const StampedPose2D &oldest = pose_buffer_.front();

          if (t <= oldest.t) {
              if (oldest.t - t > max_pose_extrapolation_) {
                  return false;
              }
              out = oldest;
              out.t = t;
              return true;
          }

          for (std::size_t i = pose_buffer_.size() - 1; i > 0; --i) {
              const StampedPose2D &b = pose_buffer_[i];
              const StampedPose2D &a = pose_buffer_[i - 1];

              if (a.t <= t && t <= b.t) {
                  if (b.t - a.t > max_pose_gap_) {
                      return false;  // hole in the pose stream; do not bridge it
                  }

                  const double alpha = (b.t - a.t) > 1e-9 ? (t - a.t) / (b.t - a.t) : 0.0;

                  out.t = t;
                  out.x = (1.0 - alpha) * a.x + alpha * b.x;
                  out.y = (1.0 - alpha) * a.y + alpha * b.y;
                  out.yaw = wrapAngle(a.yaw + alpha * wrapAngle(b.yaw - a.yaw));

                  return true;
              }
          }

          return false;
      }

      // ---------------------------------------------------- static base->lidar
      bool ensureExtrinsic(const std::string &lidar_frame) {
          if (have_extrinsic_ && lidar_frame == extrinsic_lidar_frame_) {
              return true;
          }
          try {
              const auto tf = tf_buffer_->lookupTransform(
                base_frame_, lidar_frame, tf2::TimePointZero);
              ext_x_ = tf.transform.translation.x;
              ext_y_ = tf.transform.translation.y;
              ext_yaw_ = yawFromQuat(tf.transform.rotation);
              extrinsic_lidar_frame_ = lidar_frame;
              have_extrinsic_ = true;
              return true;
          } catch (const tf2::TransformException &ex) {
              RCLCPP_WARN_THROTTLE( this->get_logger(), *this->get_clock(), 2000, "Waiting for static TF %s -> %s: %s", base_frame_.c_str(), lidar_frame.c_str(), ex.what());
              return false;
          }
      }

        // ----------------------------------------------------------------- scan
      void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)  {
          angle_min_ = msg->angle_min;
          angle_increment_ = msg->angle_increment;
          range_min_ = msg->range_min;
          range_max_ = msg->range_max;

          std::string lidar_frame = msg->header.frame_id;
          if (!lidar_frame.empty() && lidar_frame.front() == '/') {
              lidar_frame.erase(0, 1);
          }
          if (lidar_frame.empty()) {
              lidar_frame = "P1_robot0/lidar2d_link";
          }

          double t_x = 0.0;
          double t_y = 0.0;
          double yaw = 0.0;

          if (pose_source_ == "pose_topic") {
              if (!ensureExtrinsic(lidar_frame)) {
                  return;
              }
              StampedPose2D base;
              const double t_scan = rclcpp::Time(msg->header.stamp).seconds();
              if (!poseAt(t_scan, base)) {
                  RCLCPP_WARN_THROTTLE(
                      this->get_logger(), *this->get_clock(), 2000,
                      "No pose from %s bracketing scan stamp %.3f (buffer: %zu samples); "
                      "dropping scan.",
                      pose_topic_.c_str(), t_scan, pose_buffer_.size());
                  return;
              }
              t_x = base.x + std::cos(base.yaw) * ext_x_ - std::sin(base.yaw) * ext_y_;
              t_y = base.y + std::sin(base.yaw) * ext_x_ + std::cos(base.yaw) * ext_y_;
              yaw = wrapAngle(base.yaw + ext_yaw_);
            } else {  // pose_source_ == "tf"
                try {
                    const auto tf = tf_buffer_->lookupTransform(
                        map_frame_, lidar_frame,
                        rclcpp::Time(msg->header.stamp),
                        rclcpp::Duration::from_seconds(0.2));
                    t_x = tf.transform.translation.x;
                    t_y = tf.transform.translation.y;
                    yaw = yawFromQuat(tf.transform.rotation);
                } catch (const tf2::TransformException &ex) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup at laser timestamp failed: %s", ex.what());
                    return;
                }
            }

            geometry_msgs::msg::PoseArray out_msg;
            out_msg.header.stamp = msg->header.stamp;
            out_msg.header.frame_id = map_frame_;

            // No robot_pose field anymore: the consumer (NMPC node) already has its
            // own pose from esmt_pose directly, so republishing it here would just be
            // a second, possibly-stale copy of the same number.
            out_msg.poses = get_map_frame_points(msg->ranges, t_x, t_y, yaw);
            publisher_->publish(out_msg);
        }

        std::vector<geometry_msgs::msg::Pose> get_map_frame_points(const std::vector<float> &ranges, const double t_x, const double t_y, const double yaw) {
            std::vector<geometry_msgs::msg::Pose> result;
            result.reserve(ranges.size());
            constexpr double kMaxRangeSentinel = 99999.0;

            const double cos_yaw = std::cos(yaw);
            const double sin_yaw = std::sin(yaw);

            for (int i = 0; i < static_cast<int>(ranges.size()); i++) {
                const float r = ranges[i];
                const float angle = angle_min_ + i * angle_increment_;

                geometry_msgs::msg::Pose p;
                p.orientation.w = 1.0;  // orientation unused downstream; identity

                if (std::isnan(r) || (std::isinf(r) && r < 0)) {
                  continue;
                }

                if (std::isinf(r) && r > 0) {
                    const float free_ray_range =
                      std::min(range_max_, static_cast<float>(free_ray_max_range_));
                    const float x_lidar = free_ray_range * std::cos(angle);
                    const float y_lidar = free_ray_range * std::sin(angle);

                    p.position.x = cos_yaw * x_lidar - sin_yaw * y_lidar + t_x;
                    p.position.y = sin_yaw * x_lidar + cos_yaw * y_lidar + t_y;
                    p.position.z = kMaxRangeSentinel;  // marks this as a free-space ray, not an obstacle hit
                    result.push_back(p);
                } else if (std::isfinite(r) && r >= range_min_ && r <= range_max_) {
                    const float x_lidar = r * std::cos(angle);
                    const float y_lidar = r * std::sin(angle);

                    p.position.x = cos_yaw * x_lidar - sin_yaw * y_lidar + t_x;
                    p.position.y = sin_yaw * x_lidar + cos_yaw * y_lidar + t_y;
                    p.position.z = 0.0;  // z=0 marks a real obstacle hit
                    result.push_back(p);
                }
            }
            return result;
        }

        // parameters
        std::string pose_source_ = "pose_topic";
        std::string pose_topic_ = "/P1_robot0/esmt_pose";
        std::string map_frame_ = "map";
        std::string base_frame_ = "P1_robot0/base_link";
        std::string lidar_topic_ = "/P1_robot0/lidar_2d";
        std::string output_topic_ = "/P1_robot0/lidar_map_data";
        double free_ray_max_range_ = 25.0;
        double max_pose_gap_ = 0.30;
        double max_pose_extrapolation_ = 0.15;

        // scan metadata
        float angle_min_ = 0.0f;
        float angle_increment_ = 0.0f;
        float range_min_ = 0.0f;
        float range_max_ = 0.0f;

        // pose buffer + cached extrinsic (single-threaded executor: no lock needed)
        std::deque<StampedPose2D> pose_buffer_;
        bool have_extrinsic_ = false;
        std::string extrinsic_lidar_frame_;
        double ext_x_ = 0.0;
        double ext_y_ = 0.0;
        double ext_yaw_ = 0.0;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMapFrame>());
    rclcpp::shutdown();
    return 0;
}