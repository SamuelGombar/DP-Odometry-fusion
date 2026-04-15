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
#pragma once

#include <tsl/robin_map.h>

#include <Eigen/Core>
#include <kiss_icp/core/VoxelUtils.hpp>
#include <sophus/se3.hpp>
#include <tuple>
#include <vector>

namespace kinematic_icp {

struct LocalVoxelMap {
    using Voxel = kiss_icp::Voxel;

    // A candidate voxel that has not yet been observed consecutively enough times
    // to be admitted into the main map.
    struct CandidateVoxel {
        Eigen::Vector3d point;
        int last_seen_frame;
        int consecutive_count;
    };

    explicit LocalVoxelMap(double voxel_size,
                           double max_distance,
                           unsigned int max_points_per_voxel,
                           int min_consecutive_observations = 1)
        : voxel_size_(voxel_size),
          max_distance_(max_distance),
          max_points_per_voxel_(max_points_per_voxel),
          min_consecutive_observations_(min_consecutive_observations) {}

    inline void Clear() {
        map_.clear();
        candidate_map_.clear();
        frame_counter_ = 0;
    }
    inline bool Empty() const { return map_.empty(); }

    void Update(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &origin);
    void Update(const std::vector<Eigen::Vector3d> &points, const Sophus::SE3d &pose);
    void AddPoints(const std::vector<Eigen::Vector3d> &points);
    void RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    std::vector<Eigen::Vector3d> Pointcloud() const;
    std::tuple<Eigen::Vector3d, double> GetClosestNeighbor(const Eigen::Vector3d &query) const;

    double voxel_size_;
    double max_distance_;
    unsigned int max_points_per_voxel_;
    int min_consecutive_observations_;
    int frame_counter_ = 0;
    tsl::robin_map<Voxel, std::vector<Eigen::Vector3d>> map_;
    tsl::robin_map<Voxel, CandidateVoxel> candidate_map_;
};

}  // namespace kinematic_icp
