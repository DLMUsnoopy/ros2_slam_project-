#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <numeric>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

constexpr size_t kMaxObstacles = 20;
constexpr size_t kMaxPointsPerFrame = 1200; // the lidar only has 1024 points.

constexpr std::chrono::microseconds kCallbackTimeBudget{10000}; // 10 ms while testing dynamic obstacles

struct Obstacle {
    double cx = 0.0;
    double cy = 0.0;
    double half_x = 0.0;
    double half_y = 0.0;
    bool   active = false;
};


class SpatialHashGrid {
    public:
        void reserve(size_t expected_points) {
            size_t table_size = 1;
            while (table_size < expected_points * 2 + 8) table_size <<= 1;
            bucket_head_.assign(table_size, -1);
            mask_ = table_size - 1;
            next_.reserve(expected_points);
            cell_x_.reserve(expected_points);
            cell_y_.reserve(expected_points);
        }
        void build(const std::vector<std::pair<double, double>> &pts, double cell_size) {
            inv_cell_size_ = 1.0 / cell_size;
            const size_t n = pts.size();
            size_t min_table = 1;
            while (min_table < n * 2 + 8) min_table <<= 1;
            if (min_table > bucket_head_.size()) {
                bucket_head_.assign(min_table, -1);
                mask_ = min_table - 1;
            } else {
                std::fill(bucket_head_.begin(), bucket_head_.end(), -1);
            }
            next_.resize(n);
            cell_x_.resize(n);
            cell_y_.resize(n);
            for (size_t i = 0; i < n; ++i) {
                int64_t cx = static_cast<int64_t>(std::floor(pts[i].first * inv_cell_size_));
                int64_t cy = static_cast<int64_t>(std::floor(pts[i].second * inv_cell_size_));
                cell_x_[i] = cx;
                cell_y_[i] = cy;
                size_t h = hash(cx, cy) & mask_;
                next_[i] = bucket_head_[h];
                bucket_head_[h] = static_cast<int>(i);
            }
        }
        template <typename Fn>
        void forEachInNeighborhood(int64_t cx, int64_t cy, Fn &&fn) const {
            for (int64_t dx = -1; dx <= 1; ++dx) {
                for (int64_t dy = -1; dy <= 1; ++dy) {
                    size_t h = hash(cx + dx, cy + dy) & mask_;
                    for (int j = bucket_head_[h]; j != -1; j = next_[j]) {
                        if (cell_x_[j] == cx + dx && cell_y_[j] == cy + dy) {
                            if (!fn(j)) return;
                        }
                    }
                }
            }
        }
        int64_t cellX(size_t i) const { return cell_x_[i]; }
        int64_t cellY(size_t i) const { return cell_y_[i]; }
    private:
        static size_t hash(int64_t x, int64_t y) {
            uint64_t h = static_cast<uint64_t>(x) * 0x9E3779B97F4A7C15ull;
            h ^= static_cast<uint64_t>(y) * 0xC2B2AE3D27D4EB4Full;
            h ^= h >> 33;
            return static_cast<size_t>(h);
        }
        double inv_cell_size_ = 1.0;
        size_t mask_ = 0;
        std::vector<int> bucket_head_;
        std::vector<int> next_;
        std::vector<int64_t> cell_x_, cell_y_;
};


class RectangleDecomposer {
    public:
        struct Params {
            double noise_filter_radius;
            int    noise_filter_min_neighbors;
            double buffer_m;
            
            double cluster_gap_m;
            
            double max_obstacle_extent_m;
            Params() : noise_filter_radius(0.30), noise_filter_min_neighbors(3), buffer_m(0.5), cluster_gap_m(0.40), max_obstacle_extent_m(2.0) {}
        };

        explicit RectangleDecomposer(Params params = Params()) : params_(params) {
            reserve(512);
        }

        void reserve(size_t expected_points) {
            filtered_pts_.reserve(expected_points);
            noise_grid_.reserve(expected_points);
            cluster_grid_.reserve(expected_points);
            uf_parent_.reserve(expected_points);
            cluster_head_.reserve(expected_points);
            cluster_next_.reserve(expected_points);
            split_idx_.reserve(expected_points);
            candidates_.reserve(64);
        }
        
        void process(const std::vector<std::pair<double, double>> &raw_pts, std::array<Obstacle, kMaxObstacles> &out, double robot_x, double robot_y) {
            filterNoise(raw_pts);
            decompose(out, robot_x, robot_y);
        }
        
        const std::vector<std::pair<double, double>> &filteredPoints() const { return filtered_pts_; }
        
    private:
        void filterNoise(const std::vector<std::pair<double, double>> &raw_pts) {
            filtered_pts_.clear();
        
            if (raw_pts.empty()) return;
        
            noise_grid_.build(raw_pts, params_.noise_filter_radius);
            const double r2 = params_.noise_filter_radius * params_.noise_filter_radius;
            const int min_n = params_.noise_filter_min_neighbors;
        
            for (size_t i = 0; i < raw_pts.size(); ++i) {
                const double x = raw_pts[i].first;
                const double y = raw_pts[i].second;
                int neighbors = 0;
        
                noise_grid_.forEachInNeighborhood(noise_grid_.cellX(i), noise_grid_.cellY(i),
                    [&](int j) -> bool {
                        if (static_cast<size_t>(j) == i) return true;
                        double ddx = raw_pts[j].first - x;
                        double ddy = raw_pts[j].second - y;
                        if (ddx * ddx + ddy * ddy <= r2) {
                            if (++neighbors >= min_n) return false;
                        }
                        return true;
                    });

                if (neighbors >= min_n) filtered_pts_.push_back(raw_pts[i]);
            }
        }

        struct RectCandidate {
            double min_x, max_x, min_y, max_y;
            int point_count;
        };

        static double distSqToRobot(const RectCandidate &c, double rx, double ry) {
            const double dx = std::max({c.min_x - rx, 0.0, rx - c.max_x});
            const double dy = std::max({c.min_y - ry, 0.0, ry - c.max_y});
            return dx * dx + dy * dy;
        }

        int find(int x) {
            while (uf_parent_[x] != x) {
                uf_parent_[x] = uf_parent_[uf_parent_[x]];
                x = uf_parent_[x];
            }
            return x;
        }

        void unite(int a, int b) {
            a = find(a);
            b = find(b);
            if (a != b) uf_parent_[a] = b;
        }

        void splitCapped(int start, int end, std::vector<RectCandidate> &out_candidates) {
            double min_x = filtered_pts_[split_idx_[start]].first;
            double max_x = min_x;
            double min_y = filtered_pts_[split_idx_[start]].second;
            double max_y = min_y;

            for (int i = start + 1; i < end; ++i) {
                double x = filtered_pts_[split_idx_[i]].first;
                double y = filtered_pts_[split_idx_[i]].second;
                min_x = std::min(min_x, x); max_x = std::max(max_x, x);
                min_y = std::min(min_y, y); max_y = std::max(max_y, y);
            }

            const int count = end - start;
            const double w = max_x - min_x;
            const double h = max_y - min_y;

            if (count <= 1 || std::max(w, h) <= params_.max_obstacle_extent_m) {
                out_candidates.push_back({min_x, max_x, min_y, max_y, count});
                return;
            }

            const bool split_x = w >= h;
            const int mid = start + (end - start) / 2;

            std::nth_element(split_idx_.begin() + start, split_idx_.begin() + mid, split_idx_.begin() + end,
                [&](int a, int b) {
                    return split_x ? filtered_pts_[a].first < filtered_pts_[b].first
                                   : filtered_pts_[a].second < filtered_pts_[b].second;
                });

                splitCapped(start, mid, out_candidates);
            splitCapped(mid, end, out_candidates);
        }

        void decompose(std::array<Obstacle, kMaxObstacles> &out, double robot_x, double robot_y) {
            out = std::array<Obstacle, kMaxObstacles>{};
            const int n = static_cast<int>(filtered_pts_.size());
            if (n == 0) return;

            cluster_grid_.build(filtered_pts_, params_.cluster_gap_m);
            uf_parent_.resize(n);
            std::iota(uf_parent_.begin(), uf_parent_.end(), 0);

            const double gap2 = params_.cluster_gap_m * params_.cluster_gap_m;
            for (int i = 0; i < n; ++i) {
                const double xi = filtered_pts_[i].first;
                const double yi = filtered_pts_[i].second;
                cluster_grid_.forEachInNeighborhood(cluster_grid_.cellX(i), cluster_grid_.cellY(i),
                    [&](int j) -> bool {
                        if (j <= i) return true; // process each pair once (and skip self)
                        double ddx = filtered_pts_[j].first - xi;
                        double ddy = filtered_pts_[j].second - yi;
                        if (ddx * ddx + ddy * ddy <= gap2) unite(i, j);
                        return true;
                    });
            }

            cluster_head_.assign(n, -1);
            cluster_next_.resize(n);
            for (int i = 0; i < n; ++i) {
                const int r = find(i);
                cluster_next_[i] = cluster_head_[r];
                cluster_head_[r] = i;
            }

            candidates_.clear();
            split_idx_.clear();
            for (int r = 0; r < n; ++r) {
                if (cluster_head_[r] == -1) continue; // r wasnt a cluster root
                const int start = static_cast<int>(split_idx_.size());
                for (int i = cluster_head_[r]; i != -1; i = cluster_next_[i]) split_idx_.push_back(i);
                const int end = static_cast<int>(split_idx_.size());
                splitCapped(start, end, candidates_);
            }

            if (candidates_.size() > kMaxObstacles) {
                std::partial_sort(candidates_.begin(), candidates_.begin() + kMaxObstacles, candidates_.end(),
                    [&](const RectCandidate &a, const RectCandidate &b) {
                        return distSqToRobot(a, robot_x, robot_y) < distSqToRobot(b, robot_x, robot_y);
                    });
                candidates_.resize(kMaxObstacles);
            }

            for (size_t i = 0; i < candidates_.size(); ++i) {
                const RectCandidate &c = candidates_[i];
                Obstacle &o = out[i];
                o.cx = 0.5 * (c.min_x + c.max_x);
                o.cy = 0.5 * (c.min_y + c.max_y);
                o.half_x = 0.5 * (c.max_x - c.min_x) + params_.buffer_m;
                o.half_y = 0.5 * (c.max_y - c.min_y) + params_.buffer_m;
                o.active = true;
            }
        }

        Params params_;
        std::vector<std::pair<double, double>> filtered_pts_;
        SpatialHashGrid noise_grid_;
        SpatialHashGrid cluster_grid_;
        std::vector<int> uf_parent_;
        std::vector<int> cluster_head_;
        std::vector<int> cluster_next_;
        std::vector<int> split_idx_;
        std::vector<RectCandidate> candidates_;
};


class LidarDecomposer {
    public:
        explicit LidarDecomposer(RectangleDecomposer::Params params = RectangleDecomposer::Params(), double self_filter_radius_m = 0.5)
            : decomposer_(params), self_filter_radius_m_(self_filter_radius_m) {
            reserve(kMaxPointsPerFrame);
        }

        void reserve(size_t expected_points) {
            hits_scratch_.reserve(expected_points);
            decomposer_.reserve(expected_points);
        }

        void onLidarMap(const geometry_msgs::msg::PoseArray::SharedPtr &msg, double robot_x, double robot_y) {
            const auto t_start = std::chrono::steady_clock::now();
            hits_scratch_.clear();
            size_t accepted = 0;
            const double self_r2 = self_filter_radius_m_ * self_filter_radius_m_;
            for (const auto &p : msg->poses) {
                if (p.position.z > 1000.0) {
                    continue; // z is actually useless as its a 2D lidar. But we put 9999 when the data says inf (free space)
                }

                const double ddx = p.position.x - robot_x;
                const double ddy = p.position.y - robot_y;
                
                if (ddx*ddx + ddy*ddy <= self_r2) {
                    continue;
                }

                if (accepted >= kMaxPointsPerFrame) {
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("lidar_decomposer"), *clock_, 5000, "lidar_map_data exceeded kMaxPointsPerFrame (%zu); truncating.", kMaxPointsPerFrame);
                    break;
                }
                
                hits_scratch_.emplace_back(p.position.x, p.position.y);
                ++accepted;
            }
            
            std::array<Obstacle, kMaxObstacles> new_obstacles;
            decomposer_.process(hits_scratch_, new_obstacles, robot_x, robot_y);
            const auto elapsed = std::chrono::steady_clock::now() - t_start;
            
            if (elapsed > kCallbackTimeBudget) {
                ++stale_obstacle_frames_;
                RCLCPP_WARN_THROTTLE(
                    rclcpp::get_logger("lidar_decomposer"), *clock_, 1000,
                    "obstacle decomposition overran budget (%ld us); publishing fresh frame anyway.",
                    std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
            } else {
                stale_obstacle_frames_ = 0;
            }
            
            std::lock_guard<std::mutex> lock(obstacle_mutex_);
            obstacle_points_ = decomposer_.filteredPoints();
            obstacles_ = new_obstacles;
        }
        
        std::array<Obstacle, kMaxObstacles> getObstacles() const {
            std::lock_guard<std::mutex> lock(obstacle_mutex_);
            return obstacles_;
        }
        
        std::vector<std::pair<double, double>> getFilteredPoints() const {
            std::lock_guard<std::mutex> lock(obstacle_mutex_);
            return obstacle_points_;
        }

    private:
        RectangleDecomposer decomposer_;

        std::vector<std::pair<double, double>> hits_scratch_;
        
        mutable std::mutex obstacle_mutex_;
        
        std::array<Obstacle, kMaxObstacles> obstacles_{};
        
        std::vector<std::pair<double, double>> obstacle_points_;
        
        int stale_obstacle_frames_ = 0;
        
        double self_filter_radius_m_;
        
        rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
};