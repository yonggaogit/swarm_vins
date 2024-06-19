#include "globalOpt.h"
#include "Factors.h"
#include <ros/ros.h>

GlobalOptimization::GlobalOptimization(int window_size)
    : window_size_(window_size)
{
    newVIO = false;
    global_path.header.frame_id = "world";
    threadOpt = std::thread(&GlobalOptimization::optimize, this);
}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}

void GlobalOptimization::updateVIOPoseMap(const nav_msgs::Path &vio_path_msg){
    this->mPoseMap.lock();
    for(auto pose : vio_path_msg.poses) {
        double t = pose.header.stamp.toNSec()/1e9;
        if (selfPoseMap.find(t) == selfPoseMap.end()) continue;
        vector<double> vio_pos = {pose.pose.position.x,
                                  pose.pose.position.y,
                                  pose.pose.position.z,
                                  pose.pose.orientation.w,
                                  pose.pose.orientation.x,
                                  pose.pose.orientation.y,
                                  pose.pose.orientation.z};
        selfPoseMap[t] = vio_pos;
    }
    this->mPoseMap.unlock();
}

void GlobalOptimization::inputSelf(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ, Eigen::Vector3d OdomV)
{
    mPoseMap.lock();
    vector<double> pose{OdomP.x(), OdomP.y(), OdomP.z(),
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    vector<double> velocity{OdomV.x(), OdomV.y(), OdomV.z()};

    selfPoseMap[t] = pose;
    selfVelocityMap[t] = velocity;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = OdomP.x();
    pose_stamped.pose.position.y = OdomP.y();
    pose_stamped.pose.position.z = OdomP.z();
    pose_stamped.pose.orientation.x = OdomQ.x();
    pose_stamped.pose.orientation.y = OdomQ.y();
    pose_stamped.pose.orientation.z = OdomQ.z();
    pose_stamped.pose.orientation.w = OdomQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);

    lastT = t;
    lastP = OdomP;
    lastQ = OdomQ;
    lastV = OdomV;
    mPoseMap.unlock();
}
void GlobalOptimization::inputOther(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
    mPoseMap.lock();
    vector<double> pose{OdomP.x(), OdomP.y(), OdomP.z(),
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    otherPoseMap[t] = pose;
    mPoseMap.unlock();

    newVIO = true;
}
void GlobalOptimization::inputDis(double t, double dis)
{
    mPoseMap.lock();
    disMap[t] = dis;
    mPoseMap.unlock();
}


void GlobalOptimization::getGlobalOdom(double &T, Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ, Eigen::Vector3d &odomV)
{
    T = lastT;
    odomP = lastP;
    odomQ = lastQ;
    odomV = lastV;
}

void GlobalOptimization::optimize() {
    while (true) {
        if (newVIO) {
            newVIO = false;
            printf("#############global optimization begin#########\n");

            ceres::Problem problem;

            mPoseMap.lock();

            // 只优化窗口内的位姿
            if (disMap.size() > window_size_ && selfPoseMap.size() > window_size_ && otherPoseMap.size() > window_size_) {
                auto dis_iter = disMap.end();
                auto self_iter = selfPoseMap.end();
                auto other_iter = otherPoseMap.end();
                std::advance(dis_iter, -window_size_);
                std::advance(self_iter, -window_size_);
                std::advance(other_iter, -window_size_);

                double distance_weight = 0; // 设置距离残差的权重
                double smoothness_weight = 0.1; // 设置平滑性残差的权重
                double velocity_weight = 0.3; // 设置速度残差的权重
                double acceleration_weight = 0.3; // 设置加速度残差的权重

                while (dis_iter != disMap.end() && self_iter != selfPoseMap.end() && other_iter != otherPoseMap.end()) {
                    double distance = dis_iter->second;

                    problem.AddResidualBlock(
                        DistanceResidual::Create(distance, distance_weight),
                        nullptr,
                        self_iter->second.data(),
                        other_iter->second.data()
                    );

                    // 添加平滑约束
                    // if (self_iter != selfPoseMap.begin()) {
                    //     auto prev_self_iter = std::prev(self_iter);
                    //     problem.AddResidualBlock(
                    //         SmoothnessResidual::Create(smoothness_weight),
                    //         nullptr,
                    //         prev_self_iter->second.data(),
                    //         self_iter->second.data()
                    //     );

                    //     // 添加速度约束
                    //     if (std::distance(selfPoseMap.begin(), self_iter) > 1) {
                    //         auto prev_prev_self_iter = std::prev(prev_self_iter);
                    //         double dt = prev_self_iter->first - prev_prev_self_iter->first;
                    //         problem.AddResidualBlock(
                    //             VelocityResidual::Create(dt, velocity_weight),
                    //             nullptr,
                    //             prev_prev_self_iter->second.data(),
                    //             prev_self_iter->second.data(),
                    //             self_iter->second.data()
                    //         );
                    //     }

                    //     // 添加加速度约束
                    //     if (std::distance(selfPoseMap.begin(), self_iter) > 2) {
                    //         auto prev_prev_self_iter = std::prev(prev_self_iter);
                    //         auto prev_prev_prev_self_iter = std::prev(prev_prev_self_iter);
                    //         double dt = prev_prev_self_iter->first - prev_prev_prev_self_iter->first;
                    //         problem.AddResidualBlock(
                    //             AccelerationResidual::Create(dt, acceleration_weight),
                    //             nullptr,
                    //             prev_prev_prev_self_iter->second.data(),
                    //             prev_prev_self_iter->second.data(),
                    //             prev_self_iter->second.data(),
                    //             self_iter->second.data()
                    //         );
                    //     }
                    // }

                    ++dis_iter;
                    ++self_iter;
                    ++other_iter;
                }
            }

            // mPoseMap.unlock();

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = true;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            std::cout << summary.FullReport() << std::endl;
            auto lastElement = std::prev(selfPoseMap.end());
            auto lastVelocityElement = std::prev(selfVelocityMap.end());

            // mPoseMap.lock();
            for (auto it = selfPoseMap.begin(); it != selfPoseMap.end(); ++it) {
                globalPoseMap[it->first] = it->second;
                if (it == lastElement) {
                    lastT = it->first;
                    lastP = Eigen::Vector3d(it->second[0], it->second[1], it->second[2]);
                    lastQ = Eigen::Quaterniond(it->second[3], it->second[4], it->second[5], it->second[6]).toRotationMatrix();
                    lastV = Eigen::Vector3d(lastVelocityElement->second[0], lastVelocityElement->second[1], lastVelocityElement->second[2]);
                }
            }
            updateGlobalPath();
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
    return;
}


void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    for (auto iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);
    }
}