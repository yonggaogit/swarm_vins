#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct DistanceResidual {
    double distance;
    double weight; // 权重

    DistanceResidual(double distance, double weight) : distance(distance), weight(weight) {}

    template <typename T>
    bool operator()(const T* const pose1, const T* const pose2, T* residual) const {
        T dx = pose1[0] - pose2[0];
        T dy = pose1[1] - pose2[1];
        T dz = pose1[2] - pose2[2];
        residual[0] = T(weight) * (ceres::sqrt(dx * dx + dy * dy + dz * dz) - T(distance));
        return true;
    }

    static ceres::CostFunction* Create(double distance, double weight) {
        return (new ceres::AutoDiffCostFunction<DistanceResidual, 1, 7, 7>(
            new DistanceResidual(distance, weight)));
    }
};

struct SmoothnessResidual {
    double weight; // 权重

    SmoothnessResidual(double weight) : weight(weight) {}

    template <typename T>
    bool operator()(const T* const pose1, const T* const pose2, T* residual) const {
        for (int i = 0; i < 3; ++i) {
            residual[i] = T(weight) * (pose1[i] - pose2[i]);
        }
        return true;
    }

    static ceres::CostFunction* Create(double weight) {
        return (new ceres::AutoDiffCostFunction<SmoothnessResidual, 3, 7, 7>(
            new SmoothnessResidual(weight)));
    }
};


struct VelocityResidual {
    double dt;
    double weight; // 权重

    VelocityResidual(double delta_t, double weight) : dt(delta_t), weight(weight) {}

    template <typename T>
    bool operator()(const T* const pose1, const T* const pose2, const T* const pose3, T* residual) const {
        for (int i = 0; i < 3; ++i) {
            residual[i] = T(weight) * ((pose3[i] - pose2[i]) / T(dt) - (pose2[i] - pose1[i]) / T(dt));
        }
        return true;
    }

    static ceres::CostFunction* Create(double dt, double weight) {
        return (new ceres::AutoDiffCostFunction<VelocityResidual, 3, 7, 7, 7>(
            new VelocityResidual(dt, weight)));
    }
};


struct AccelerationResidual {
    double dt;
    double weight; // 权重

    AccelerationResidual(double delta_t, double weight) : dt(delta_t), weight(weight) {}

    template <typename T>
    bool operator()(const T* const pose1, const T* const pose2, const T* const pose3, const T* const pose4, T* residual) const {
        for (int i = 0; i < 3; ++i) {
            T velocity1 = (pose2[i] - pose1[i]) / T(dt);
            T velocity2 = (pose3[i] - pose2[i]) / T(dt);
            T velocity3 = (pose4[i] - pose3[i]) / T(dt);
            residual[i] = T(weight) * ((velocity3 - velocity2) / T(dt) - (velocity2 - velocity1) / T(dt));
        }
        return true;
    }

    static ceres::CostFunction* Create(double dt, double weight) {
        return (new ceres::AutoDiffCostFunction<AccelerationResidual, 3, 7, 7, 7, 7>(
            new AccelerationResidual(dt, weight)));
    }
};