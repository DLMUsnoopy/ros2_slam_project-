#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

using Point = geometry_msgs::msg::Point;
using PoseArray = geometry_msgs::msg::PoseArray;
using TwistStamped = geometry_msgs::msg::TwistStamped;

// Forward declaration with correct signature
std::pair<float, float> PID(float x_robot, float y_robot, float z_robot, float x_waypoint, float y_waypoint);

class PIDController : public rclcpp::Node {
    public:
        PIDController() : Node("pid_controller") {
            sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/P1_robot0/odom_gt", 10, std::bind(&PIDController::odom_callback, this, std::placeholders::_1));
            
            waypoint_subscriber_ = this->create_subscription<PoseArray>("/planned_route_poses", 10, std::bind(&PIDController::listener_callback, this, std::placeholders::_1));

            publisher_ = this->create_publisher<TwistStamped>("/P1_robot0/cmd_vel", 10);

            timer_ = this->create_wall_timer(20ms, std::bind(&PIDController::timer_callback, this)); // 50Hz
        }

    private:
        float x_robot = 0.0f;
        float y_robot = 0.0f;
        double z_robot = 0.0;  // double because getRPY outputs double

        float x_waypoint = 0.0f;
        float y_waypoint = 0.0f;
        
        int w_idx = 0;
        PoseArray::SharedPtr waypoints_;

        void listener_callback(const PoseArray::SharedPtr msg) {
            this->waypoints_ = msg;
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            x_robot = msg->pose.pose.position.x;
            y_robot = msg->pose.pose.position.y;

            tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

            double roll, pitch;

            tf2::Matrix3x3(q).getRPY(roll, pitch, z_robot);
        }

        void timer_callback(){
            if (!waypoints_ || waypoints_->poses.empty()) return;
            auto msg = TwistStamped();
            
            float x_waypoint = this->waypoints_->poses[w_idx].position.x;
            float y_waypoint = this->waypoints_->poses[w_idx].position.y;

            float dx = x_waypoint - x_robot;
            float dy = y_waypoint - y_robot;

            if (std::sqrt(dx*dx + dy*dy) < 0.1f && w_idx < (int)waypoints_->poses.size() - 1) {
                w_idx++;
                
                x_waypoint = this->waypoints_->poses[w_idx].position.x;
                y_waypoint = this->waypoints_->poses[w_idx].position.y;
            }

            auto [lin, ang] = PID(x_robot, y_robot, (float)z_robot, x_waypoint, y_waypoint);

            // ── Logging ───────────────────────────────────────────────────────
            static std::ofstream log_file("/home/dev/p03_ws/tmp/pid_log.csv", std::ios::out);
            static bool header_written = false;
            if (!header_written) {
                log_file << "time_hhmmssms,d_dist_signed,d_yaw,ideal_yaw,v_cmd,w_cmd,waypoint_index\n";
                header_written = true;
            }

            float ideal_yaw_log = std::atan2(dy, dx);
            float d_yaw_log     = ideal_yaw_log - (float)z_robot;
            if      (d_yaw_log >  M_PI) d_yaw_log -= 2.0f * M_PI;
            else if (d_yaw_log < -M_PI) d_yaw_log += 2.0f * M_PI;
            float d_dist_signed = dx * std::cos((float)z_robot) + dy * std::sin((float)z_robot);

            auto    ros_now  = this->get_clock()->now();
            int64_t total_ms = ros_now.nanoseconds() / 1'000'000LL;
            int hh =  (total_ms / 3'600'000LL) % 24;
            int mm =  (total_ms /    60'000LL) % 60;
            int ss =  (total_ms /     1'000LL) % 60;
            int ms =   total_ms                % 1000;

            std::ostringstream t_fmt;
            t_fmt << std::setfill('0')
                  << std::setw(2) << hh << ":"
                  << std::setw(2) << mm << ":"
                  << std::setw(2) << ss << "."
                  << std::setw(3) << ms;

            log_file << t_fmt.str()   << ","
                     << d_dist_signed << ","
                     << d_yaw_log     << ","
                     << ideal_yaw_log << ","
                     << lin           << ","
                     << ang           << ","
                     << w_idx         << "\n";
            log_file.flush();
            // ─────────────────────────────────────────────────────────────────

            msg.header.stamp = this->get_clock()->now();
            msg.twist.linear.x  = lin;
            msg.twist.angular.z = ang;

            RCLCPP_INFO(this->get_logger(), "Publishing: lin=%.2f, ang=%.2f | waypoint[%d]: (%.2f, %.2f) | robot: (%.2f, %.2f)", lin, ang, w_idx, x_waypoint, y_waypoint, x_robot, y_robot);

            publisher_->publish(msg);
        }

        rclcpp::Publisher<TwistStamped>::SharedPtr publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Subscription<PoseArray>::SharedPtr waypoint_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
};


std::pair<float, float> PID(float x_robot, float y_robot, float z_robot, float x_waypoint, float y_waypoint){
    
    float dx = x_waypoint - x_robot;
    float dy = y_waypoint - y_robot;

    // z is yaw. Just to make it clear
    float d_dist = std::sqrt(dx * dx + dy * dy);
    float ideal_yaw = std::atan2(dy, dx);
    float d_yaw = ideal_yaw - z_robot;

    // Normalize heading error to -pi to pi
    if (d_yaw >  M_PI) d_yaw -= 2.0f * M_PI;
    else if (d_yaw < -M_PI) d_yaw += 2.0f * M_PI;

    // Will tune it once the whole thing can run
    float Kp_lin = 5.0f, Ki_lin = 0.0f, Kd_lin = 0.1f;
    float Kp_ang = 10.0f, Ki_ang = 0.0f, Kd_ang = 0.1f;

    // Integral needs to persist so that it can build up, so its static
    static float integral_lin = 0.0f, prev_error_lin = 0.0f;
    static float integral_ang = 0.0f, prev_err_ang   = 0.0f;
    static float dt = 0.02f; // 50 Hz

    // PID for direction (works constantly coz robot needs to always be perfectly straight)
    float angular_vel = 0.0f;
    if (std::abs(d_yaw) > 0.08f && std::sqrt(dx*dx + dy*dy) > 0.1){
        integral_ang += d_yaw * dt;

        float derivative_ang = (d_yaw - prev_err_ang) / dt;
        
        angular_vel  = Kp_ang * d_yaw + Ki_ang * integral_ang + Kd_ang * derivative_ang;
        
        prev_err_ang = d_yaw;
    }

    // PID for distance (only runs if direction is okayish (~17 degrees threshold))
    float linear_vel = 0.0f;
    if (std::abs(d_yaw) <= 0.08f && std::sqrt(dx*dx + dy*dy) > 0.1){
        integral_lin    += d_dist * dt;
        
        float derivative_lin = (d_dist - prev_error_lin) / dt;
        
        linear_vel       = Kp_lin * d_dist + Ki_lin * integral_lin + Kd_lin * derivative_lin;
        
        prev_error_lin   = d_dist;
    }

    return {linear_vel, angular_vel};
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDController>());
    rclcpp::shutdown();
    return 0;
}