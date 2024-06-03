#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct DistanceResidual {
    DistanceResidual(double observed_distance)
        : observed_distance_(observed_distance) {}

    template <typename T>
    bool operator()(const T* const self_pose, const T* const other_pose, T* residual) const {
        T self_position[3] = {self_pose[0], self_pose[1], self_pose[2]};
        T other_position[3] = {other_pose[0], other_pose[1], other_pose[2]};
        residual[0] = ceres::sqrt(ceres::pow(self_position[0] - other_position[0], 2) +
                                  ceres::pow(self_position[1] - other_position[1], 2) +
                                  ceres::pow(self_position[2] - other_position[2], 2)) - T(observed_distance_);
        return true;
    }

    static ceres::CostFunction* Create(const double observed_distance) {
        return (new ceres::AutoDiffCostFunction<DistanceResidual, 1, 7, 7>(
            new DistanceResidual(observed_distance)));
    }

    double observed_distance_;
};
