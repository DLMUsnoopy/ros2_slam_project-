#include <chrono>
#include <algorithm>
#include <memory>
#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <casadi/casadi.hpp>

#include "p03_nmpc_controller/lidar_decomposer.hpp"

using namespace std::chrono_literals;
using namespace casadi;

using TwistStamped = geometry_msgs::msg::TwistStamped;
using PoseArray    = geometry_msgs::msg::PoseArray;
using Pose         = geometry_msgs::msg::Pose;


static constexpr int N  = 10;
static constexpr double DT = 0.2;

static constexpr double V_MAX = 1.0;
static constexpr double V_MIN = 0.0;
static constexpr double W_MAX = 2.5;
static constexpr double W_MIN = -2.5;

static constexpr double Q_x = 7.5;
static constexpr double Q_y = 7.5;
static constexpr double Q_th = 5.0;
static constexpr double R_v = 0.5;
static constexpr double R_w = 0.25;

static constexpr double ALPHA_YAW = 3.0;

static constexpr double WP_TOLERANCE = 0.2;

static constexpr double R_DV = 2.0;
static constexpr double R_DW = 0.4;

static constexpr double A_MAX     = 1.5;
static constexpr double ALPHA_MAX = 3.0;

static constexpr double W_OBS        = 50.0;
static constexpr double SIGMA_OBS    = 8.0;
static constexpr double OBS_SHARPNESS = 1.0;
static constexpr double SMAX_EPS     = 1e-4;


MX smooth_relu(const MX& x, double eps = SMAX_EPS){
    return 0.5 * (x + sqrt(x*x + eps));
}


MX robot_model(const MX& X_sym, const MX& U_sym){
    MX x  = X_sym(0);
    MX y  = X_sym(1);
    MX th = X_sym(2);

    MX v  = U_sym(0);
    MX w  = U_sym(1);

    MX x_dot  = v * cos(th);
    MX y_dot  = v * sin(th);
    MX th_dot = w;

    return vertcat(x_dot, y_dot, th_dot);
}

MX rk4_step(const MX& X_sym, const MX& U_sym){
    MX k1 = robot_model(X_sym,                 U_sym);
    MX k2 = robot_model(X_sym + (DT/2.0)*k1,  U_sym);
    MX k3 = robot_model(X_sym + (DT/2.0)*k2,  U_sym);
    MX k4 = robot_model(X_sym + DT*k3,         U_sym);

    return X_sym + (DT/6.0) * (k1 + 2*k2 + 2*k3 + k4);
}


Function build_nmpc_solver(){
    MX U_flat = MX::sym("U", 2*N, 1);

    static constexpr int kObsBlockSize = 4 * static_cast<int>(kMaxObstacles);
    static constexpr int kPSize = 6 + kObsBlockSize + 2;
    MX P = MX::sym("P", kPSize, 1);

    MX X_k = vertcat(P(0), P(1), P(2));

    MX x_ref  = P(3);
    MX y_ref  = P(4);
    MX th_ref = P(5);

    std::vector<MX> obs_cx(kMaxObstacles), obs_cy(kMaxObstacles), obs_hx(kMaxObstacles), obs_hy(kMaxObstacles);
    for (size_t i = 0; i < kMaxObstacles; ++i) {
        int base = 6 + 4 * static_cast<int>(i);
        obs_cx[i] = P(base);
        obs_cy[i] = P(base + 1);
        obs_hx[i] = P(base + 2);
        obs_hy[i] = P(base + 3);
    }

    MX v_prev = P(6 + kObsBlockSize);
    MX w_prev = P(6 + kObsBlockSize + 1);

    MX J = 0;
    MX g_vec;

    for (int k = 0; k < N; k++){
        MX U_k = vertcat(U_flat(2*k), U_flat(2*k + 1));

        MX X_next = rk4_step(X_k, U_k);

        MX e_x  = X_next(0) - x_ref;
        MX e_y  = X_next(1) - y_ref;
        MX e_th = 1 - cos(X_next(2) - th_ref);


        MX dist_sq = e_x*e_x + e_y*e_y;

        MX braking_multiplier = 1.0 + (20.0 / (1.0 + dist_sq));


        MX dv = U_k(0) - v_prev;
        MX dw = U_k(1) - w_prev;
        MX smooth_cost = R_DV * dv * dv + R_DW * dw * dw;
        g_vec = vertcat(g_vec, dv, dw);


        std::vector<MX> obs_terms(kMaxObstacles);
        for (size_t i = 0; i < kMaxObstacles; ++i) {
            MX ex_o = (X_next(0) - obs_cx[i]) / obs_hx[i];
            MX ey_o = (X_next(1) - obs_cy[i]) / obs_hy[i];
            MX rho = sqrt(ex_o*ex_o + ey_o*ey_o + 1e-9);      
            obs_terms[i] = W_OBS * exp(-SIGMA_OBS * (rho - 1.0));
        }
        
        MX obs_stack = vertcat(obs_terms);
        MX obs_m     = mmax(obs_stack);
        MX obs_cost  = obs_m + log(sum1(exp(OBS_SHARPNESS * (obs_stack - obs_m)))) / OBS_SHARPNESS;

        
        MX stage_cost = Q_x  * e_x  * e_x  + Q_y  * e_y  * e_y   + Q_th * e_th * e_th +
                        R_v * braking_multiplier * U_k(0) * U_k(0) * (1.0 + ALPHA_YAW * e_th*e_th) + R_w  * U_k(1) * U_k(1) + obs_cost;
        J = J + stage_cost;

        X_k = X_next;
        v_prev = U_k(0);
        w_prev = U_k(1);
    }

    MXDict nlp = {{"x", U_flat}, {"f", J}, {"p", P}, {"g", g_vec}};

    Dict ipopt_opts;
    ipopt_opts["print_level"] = 0;
    ipopt_opts["max_iter"] = 100;
    ipopt_opts["tol"] = 1e-4;
    ipopt_opts["warm_start_init_point"]= "yes";

    Dict solver_opts;
    solver_opts["ipopt"] = ipopt_opts;
    solver_opts["print_time"] = false;

    return nlpsol("nmpc_solver", "ipopt", nlp, solver_opts);
}

class NMPCController : public rclcpp::Node{
    public:
        NMPCController() : Node("p03_nmpc_controller") {
            map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
            base_frame_ = this->declare_parameter<std::string>("base_frame", "P1_robot0/base_link");
            waypoint_topic_ = this->declare_parameter<std::string>("waypoint_topic", "/planned_route_poses");
            lidar_topic_ = this->declare_parameter<std::string>("lidar_topic", "/P1_robot0/lidar_map_data");
            cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/P1_robot0/cmd_vel");
            goal_reached_topic_ = this->declare_parameter<std::string>("goal_reached_topic", "/P1_robot0/goal_reached");
            obstacle_marker_topic_ = this->declare_parameter<std::string>("obstacle_marker_topic", "/P1_robot0/obstacle_markers");
            control_rate_ = std::max(1.0, this->declare_parameter<double>("control_rate", 50.0));
            pose_timeout_ = std::max(0.0, this->declare_parameter<double>("pose_timeout", 0.5));
            route_timeout_ = std::max(0.0, this->declare_parameter<double>("route_timeout", 2.0));
            waypoint_tolerance_ = std::max(0.0, this->declare_parameter<double>("waypoint_tolerance", WP_TOLERANCE));
            goal_tolerance_ = std::max(0.0, this->declare_parameter<double>("goal_tolerance", 0.25));

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            sub_waypoints_ = this->create_subscription<PoseArray>(waypoint_topic_, 10, std::bind(&NMPCController::waypoint_callback, this, std::placeholders::_1));
            sub_dynamic_lidar_ = this->create_subscription<geometry_msgs::msg::PoseArray>(lidar_topic_, rclcpp::SensorDataQoS(), std::bind(&NMPCController::lidar_map_callback, this, std::placeholders::_1));

            pub_cmd_ = this->create_publisher<TwistStamped>(cmd_vel_topic_, 10);
            pub_goal_reached_ = this->create_publisher<std_msgs::msg::Bool>(goal_reached_topic_, 10);
            pub_obstacle_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(obstacle_marker_topic_, 10);

            solver_ = build_nmpc_solver();

            U_warm_ = std::vector<double>(2*N, 0.0);

            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / control_rate_),
                std::bind(&NMPCController::control_loop, this));

            RCLCPP_INFO(
                this->get_logger(),
                "NMPC interfaces: TF %s -> %s, route=%s, obstacles=%s, cmd=%s.",
                map_frame_.c_str(), base_frame_.c_str(), waypoint_topic_.c_str(),
                lidar_topic_.c_str(), cmd_vel_topic_.c_str());
        }

    private:
        void waypoint_callback(const PoseArray::SharedPtr msg){
            if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
                RCLCPP_ERROR(
                    this->get_logger(), "Rejecting route in frame '%s'; expected '%s'.",
                    msg->header.frame_id.c_str(), map_frame_.c_str());
                waypoints_.reset();
                publish_velocity(0.0, 0.0);
                return;
            }
            waypoints_ = msg;
            last_route_time_ = this->now();
            waypoint_index_ = 0;
            goal_reached_ = false;
            publish_goal_reached(false);

            if (waypoints_->poses.empty()) {
                publish_velocity(0.0, 0.0);
                return;
            }

            if (have_robot_pose_) {
                double best_distance = std::numeric_limits<double>::infinity();
                for (std::size_t index = 0; index < waypoints_->poses.size(); ++index) {
                    const auto &position = waypoints_->poses[index].position;
                    const double distance = std::hypot(position.x - x_robot_, position.y - y_robot_);
                    if (distance < best_distance) {
                        best_distance = distance;
                        waypoint_index_ = index;
                    }
                }
            }
        }

        void lidar_map_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            if (!have_robot_pose_) {
                return;
            }
            lidar_decomposer_.onLidarMap(msg, x_robot_, y_robot_);
        }

        bool update_robot_pose_from_tf() {
            try {
                const auto transform = tf_buffer_->lookupTransform(
                    map_frame_, base_frame_, tf2::TimePointZero);
                const auto transform_stamp = rclcpp::Time(
                    transform.header.stamp, this->get_clock()->get_clock_type());
                const double age = (this->now() - transform_stamp).seconds();
                if (pose_timeout_ > 0.0 && transform_stamp.nanoseconds() > 0 && age > pose_timeout_) {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "SLAM TF is stale by %.3f s; stopping.", age);
                    have_robot_pose_ = false;
                    return false;
                }

                x_robot_ = transform.transform.translation.x;
                y_robot_ = transform.transform.translation.y;
                tf2::Quaternion quaternion(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w);
                double roll = 0.0;
                double pitch = 0.0;
                tf2::Matrix3x3(quaternion).getRPY(roll, pitch, th_robot_);
                have_robot_pose_ = true;
                return true;
            } catch (const tf2::TransformException &exception) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Waiting for TF %s -> %s: %s", map_frame_.c_str(),
                    base_frame_.c_str(), exception.what());
                have_robot_pose_ = false;
                return false;
            }
        }


        void publish_obstacle_markers(const std::array<Obstacle, kMaxObstacles>& obstacles) {
            visualization_msgs::msg::MarkerArray marker_array;

            // Clearing previous code
            visualization_msgs::msg::Marker delete_all;
            delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(delete_all);


            int id = 0;

            for (const auto &obs : obstacles) {
                if (!obs.active) continue;

                visualization_msgs::msg::Marker m;
                m.header.frame_id = "map";
                m.header.stamp = this->get_clock()->now();
                m.ns = "nmpc_obstacles";
                m.id = id++;
                
                m.type = visualization_msgs::msg::Marker::CUBE;
                m.action = visualization_msgs::msg::Marker::ADD;

                m.pose.position.x = obs.cx;
                m.pose.position.y = obs.cy;
                m.pose.position.z = 0.5;
                
                m.pose.orientation.w = 1.0; 

                m.scale.x = obs.half_x * 2.0;
                m.scale.y = obs.half_y * 2.0;
                m.scale.z = 1.0;

                m.color.r = 1.0;
                m.color.g = 0.0;
                m.color.b = 0.0;
                m.color.a = 0.4; 

                marker_array.markers.push_back(m);
            }

            pub_obstacle_markers_->publish(marker_array);
        }

        // Safety-stop conditions.
        void control_loop() {
            if (!update_robot_pose_from_tf()) {
                publish_velocity(0.0, 0.0);
                return;
            }
            if (!waypoints_ || waypoints_->poses.empty()) {
                publish_velocity(0.0, 0.0);
                return;
            }
            if (route_timeout_ > 0.0 && (this->now() - last_route_time_).seconds() > route_timeout_) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Route is stale; stopping until A* publishes a fresh route.");
                publish_velocity(0.0, 0.0);
                return;
            }

            waypoint_index_ = std::min(waypoint_index_, waypoints_->poses.size() - 1);

            const auto &goal = waypoints_->poses.back().position;
            if (std::hypot(goal.x - x_robot_, goal.y - y_robot_) <= goal_tolerance_) {
                publish_velocity(0.0, 0.0);
                if (!goal_reached_) {
                    goal_reached_ = true;
                    publish_goal_reached(true);
                    RCLCPP_INFO(this->get_logger(), "Final goal reached; commanded zero velocity.");
                }
                return;
            }

            double x_ref = waypoints_->poses[waypoint_index_].position.x;
            double y_ref = waypoints_->poses[waypoint_index_].position.y;

            double dx = x_ref - x_robot_;
            double dy = y_ref - y_robot_;
            double dist_to_waypoint = std::sqrt(dx*dx + dy*dy);

            bool is_last_waypoint = (waypoint_index_ == waypoints_->poses.size() - 1);

            if (dist_to_waypoint < waypoint_tolerance_ && !is_last_waypoint){
                waypoint_index_++;
                x_ref = waypoints_->poses[waypoint_index_].position.x;
                y_ref = waypoints_->poses[waypoint_index_].position.y;
                dx = x_ref - x_robot_;
                dy = y_ref - y_robot_;
            }

            double th_ref = std::atan2(dy, dx);

            std::vector<double> P = {
                x_robot_,  y_robot_,  th_robot_,
                x_ref,     y_ref,     th_ref
            };


            P.reserve(P.size() + 4 * kMaxObstacles);
            
            const std::array<Obstacle, kMaxObstacles> obstacles = lidar_decomposer_.getObstacles();

            publish_obstacle_markers(obstacles);

            for (const auto &obs : obstacles) {
                if (obs.active) {
                    P.push_back(obs.cx);
                    P.push_back(obs.cy);
                    P.push_back(obs.half_x);
                    P.push_back(obs.half_y);
                } else {
                    P.push_back(1e6);
                    P.push_back(1e6);
                    P.push_back(1.0);
                    P.push_back(1.0);
                }
            }

            P.push_back(v_last_);
            P.push_back(w_last_);

            std::vector<double> lower_bounds(2*N), upper_bounds(2*N);
            for (int k = 0; k < N; k++){
                lower_bounds[2*k] = V_MIN;
                upper_bounds[2*k] = V_MAX;
                
                lower_bounds[2*k + 1] = W_MIN;
                upper_bounds[2*k + 1] = W_MAX;
            }

            std::vector<double> lbg(2*N), ubg(2*N);
            for (int k = 0; k < N; k++){
                lbg[2*k]     = -A_MAX * DT;
                ubg[2*k]     =  A_MAX * DT;
                lbg[2*k + 1] = -ALPHA_MAX * DT;
                ubg[2*k + 1] =  ALPHA_MAX * DT;
            }

            DMDict solver_input = {{"x0",  DM(U_warm_)}, {"lbx", DM(lower_bounds)}, {"ubx", DM(upper_bounds)}, {"lbg", DM(lbg)}, {"ubg", DM(ubg)}, {"p",   DM(P)}};

            DMDict solver_output;
            try {
                solver_output = solver_(solver_input);
            }
            catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Solver failed: %s — sending zero velocity.", e.what());
                publish_velocity(0.0, 0.0);
                return;
            }

            std::vector<double> U_star = std::vector<double>(solver_output.at("x"));

            double v_cmd = U_star[0];
            double w_cmd = U_star[1];

            publish_velocity(v_cmd, w_cmd);

            RCLCPP_INFO(this->get_logger(), "NMPC → v=%.3f m/s  ω=%.3f rad/s | wp[%zu]=(%.2f,%.2f) | robot=(%.2f,%.2f,%.1f°)", v_cmd, w_cmd, waypoint_index_, x_ref, y_ref, x_robot_, y_robot_, th_robot_ * 180.0 / M_PI);

            for (int k = 0; k < N - 1; k++){
                U_warm_[2*k] = U_star[2*k + 2];
                U_warm_[2*k + 1] = U_star[2*k + 3];
            }

            U_warm_[2*(N-1)] = 0.0;
            U_warm_[2*(N-1) + 1] = 0.0;
        }

        void publish_velocity(double v, double w) {
            TwistStamped msg;
            msg.header.stamp = this->get_clock()->now();
            msg.twist.linear.x = v;
            msg.twist.angular.z = w;
            pub_cmd_->publish(msg);

            v_last_ = v;
            w_last_ = w;
        }

        void publish_goal_reached(bool reached) {
            std_msgs::msg::Bool message;
            message.data = reached;
            pub_goal_reached_->publish(message);
        }

        LidarDecomposer lidar_decomposer_;

        double x_robot_ = 0.0;
        double y_robot_ = 0.0;
        double th_robot_ = 0.0;
        bool have_robot_pose_ = false;
        bool goal_reached_ = false;

        std::size_t waypoint_index_ = 0;
        PoseArray::SharedPtr waypoints_;

        double x_ref = 0.0;
        double y_ref = 0.0;
        double th_ref = 0.0;

        Function solver_;
        std::vector<double>  U_warm_;

        double v_last_ = 0.0;
        double w_last_ = 0.0;

        std::string map_frame_;
        std::string base_frame_;
        std::string waypoint_topic_;
        std::string lidar_topic_;
        std::string cmd_vel_topic_;
        std::string goal_reached_topic_;
        std::string obstacle_marker_topic_;
        double control_rate_ = 50.0;
        double pose_timeout_ = 0.5;
        double route_timeout_ = 2.0;
        double waypoint_tolerance_ = WP_TOLERANCE;
        double goal_tolerance_ = 0.25;
        rclcpp::Time last_route_time_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Publisher<TwistStamped>::SharedPtr pub_cmd_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_reached_;
        rclcpp::Subscription<PoseArray>::SharedPtr sub_waypoints_;
        rclcpp::Subscription<PoseArray>::SharedPtr sub_dynamic_lidar_;
        rclcpp::Subscription<Pose>::SharedPtr sub_dynamic_obstacle_poses_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacle_markers_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NMPCController>());
    rclcpp::shutdown();
    return 0;
}
