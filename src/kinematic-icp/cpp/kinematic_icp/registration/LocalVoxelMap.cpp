// MIT License

// Copyright (c) 2024 Tiziano Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill
// Stachniss.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "LocalVoxelMap.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <kiss_icp/core/VoxelUtils.hpp>
#include <limits>
#include <tuple>
#include <vector>

namespace {
using kiss_icp::Voxel;
static const std::array<Voxel, 27> voxel_shifts{
    {Voxel{0, 0, 0},   Voxel{1, 0, 0},   Voxel{-1, 0, 0},  Voxel{0, 1, 0},   Voxel{0, -1, 0},
     Voxel{0, 0, 1},   Voxel{0, 0, -1},  Voxel{1, 1, 0},   Voxel{1, -1, 0},  Voxel{-1, 1, 0},
     Voxel{-1, -1, 0}, Voxel{1, 0, 1},   Voxel{1, 0, -1},  Voxel{-1, 0, 1},  Voxel{-1, 0, -1},
     Voxel{0, 1, 1},   Voxel{0, 1, -1},  Voxel{0, -1, 1},  Voxel{0, -1, -1}, Voxel{1, 1, 1},
     Voxel{1, 1, -1},  Voxel{1, -1, 1},  Voxel{1, -1, -1}, Voxel{-1, 1, 1},  Voxel{-1, 1, -1},
     Voxel{-1, -1, 1}, Voxel{-1, -1, -1}}};
}  // namespace

namespace kinematic_icp {

std::tuple<Eigen::Vector3d, double> LocalVoxelMap::GetClosestNeighbor(
    const Eigen::Vector3d &query) const {
    const auto &voxel = kiss_icp::PointToVoxel(query, voxel_size_);
    Eigen::Vector3d closest_neighbor = Eigen::Vector3d::Zero();
    double closest_distance = std::numeric_limits<double>::max();
    std::for_each(voxel_shifts.cbegin(), voxel_shifts.cend(), [&](const auto &voxel_shift) {
        const auto &query_voxel = voxel + voxel_shift;
        auto search = map_.find(query_voxel);
        if (search != map_.end()) {
            const auto &points = search.value();
            const Eigen::Vector3d &neighbor = *std::min_element(
                points.cbegin(), points.cend(), [&](const auto &lhs, const auto &rhs) {
                    return (lhs - query).norm() < (rhs - query).norm();
                });
            double distance = (neighbor - query).norm();
            if (distance < closest_distance) {
                closest_neighbor = neighbor;
                closest_distance = distance;
            }
        }
    });
    return std::make_tuple(closest_neighbor, closest_distance);
}

std::vector<Eigen::Vector3d> LocalVoxelMap::Pointcloud() const {
    std::vector<Eigen::Vector3d> points;
    points.reserve(map_.size() * static_cast<size_t>(max_points_per_voxel_));
    std::for_each(map_.cbegin(), map_.cend(), [&](const auto &map_element) {
        const auto &voxel_points = map_element.second;
        points.insert(points.end(), voxel_points.cbegin(), voxel_points.cend());
    });
    points.shrink_to_fit();
    return points;
}

void LocalVoxelMap::Update(const std::vector<Eigen::Vector3d> &points,
                           const Eigen::Vector3d &origin) {
    frame_counter_++;
    AddPoints(points);
    RemovePointsFarFromLocation(origin);
}

void LocalVoxelMap::Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose) {
    std::vector<Eigen::Vector3d> points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const auto &point) { return pose * point; });
    const Eigen::Vector3d &origin = pose.translation();
    Update(points_transformed, origin);
}

void LocalVoxelMap::AddPoints(const std::vector<Eigen::Vector3d> &points) {
    const double map_resolution = std::sqrt(voxel_size_ * voxel_size_ / max_points_per_voxel_);
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        const auto voxel = kiss_icp::PointToVoxel(point, voxel_size_);
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            // Voxel already admitted: apply standard spacing check
            auto &voxel_points = search.value();
            if (voxel_points.size() == max_points_per_voxel_ ||
                std::any_of(voxel_points.cbegin(), voxel_points.cend(),
                            [&](const auto &voxel_point) {
                                return (voxel_point - point).norm() < map_resolution;
                            })) {
                return;
            }
            voxel_points.emplace_back(point);
        } else if (min_consecutive_observations_ <= 1 || frame_counter_ <= 1) {
            // No temporal filtering requested or first frame: admit immediately
            std::vector<Eigen::Vector3d> voxel_points;
            voxel_points.reserve(max_points_per_voxel_);
            voxel_points.emplace_back(point);
            map_.insert({voxel, std::move(voxel_points)});
        } else {
            // Temporal filter: require consecutive observations before admission
            auto cand = candidate_map_.find(voxel);
            if (cand != candidate_map_.end()) {
                auto &info = cand.value();
                if (info.last_seen_frame == frame_counter_ - 1) {
                    info.consecutive_count++;
                    info.last_seen_frame = frame_counter_;
                    info.point = point;
                    if (info.consecutive_count >= min_consecutive_observations_) {
                        std::vector<Eigen::Vector3d> voxel_points;
                        voxel_points.reserve(max_points_per_voxel_);
                        voxel_points.emplace_back(point);
                        map_.insert({voxel, std::move(voxel_points)});
                        candidate_map_.erase(cand);
                    }
                } else {
                    // Streak broken: restart count
                    info.consecutive_count = 1;
                    info.last_seen_frame = frame_counter_;
                    info.point = point;
                }
            } else {
                candidate_map_.insert({voxel, {point, frame_counter_, 1}});
            }
        }
    });
}

void LocalVoxelMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    const auto max_distance2 = max_distance_ * max_distance_;
    for (auto it = map_.begin(); it != map_.end();) {
        const auto &[voxel, voxel_points] = *it;
        const auto &pt = voxel_points.front();
        if ((pt - origin).squaredNorm() >= max_distance2) {
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
    if (min_consecutive_observations_ > 1) {
        for (auto it = candidate_map_.begin(); it != candidate_map_.end();) {
            if ((it->second.point - origin).squaredNorm() >= max_distance2) {
                it = candidate_map_.erase(it);
            } else {
                ++it;
            }
        }
    }
}

}  // namespace kinematic_icp
