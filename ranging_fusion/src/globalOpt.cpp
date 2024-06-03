#include "globalOpt.h"
#include "Factors.h"
#include <ros/ros.h>

GlobalOptimization::GlobalOptimization()
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

void GlobalOptimization::inputSelf(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
    mPoseMap.lock();
    vector<double> pose{OdomP.x(), OdomP.y(), OdomP.z(),
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    selfPoseMap[t] = pose;

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

    lastP = OdomP;
    lastQ = OdomQ;
    mPoseMap.unlock();

    newVIO = true;
}
void GlobalOptimization::inputOther(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
    mPoseMap.lock();
    vector<double> pose{OdomP.x(), OdomP.y(), OdomP.z(),
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    otherPoseMap[t] = pose;
    mPoseMap.unlock();
}
void GlobalOptimization::inputDis(double t, double dis)
{
    mPoseMap.lock();
    disMap[t] = dis;
    mPoseMap.unlock();
}


void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

void GlobalOptimization::optimize()
{
    while(true)
    {
        if(newVIO)
        {
            newVIO = false;
            printf("#############global optimization begin#########\n");

            ceres::Problem problem;

            mPoseMap.lock();

            auto dis_iter = disMap.begin();
            auto self_iter = selfPoseMap.begin();
            auto other_iter = otherPoseMap.begin();

            while (dis_iter != disMap.end() && self_iter != selfPoseMap.end() && other_iter != otherPoseMap.end()) {
                double distance = dis_iter->second;

                problem.AddResidualBlock(
                    DistanceResidual::Create(distance),
                    nullptr,
                    self_iter->second.data(),
                    other_iter->second.data()
                );

                ++dis_iter;
                ++self_iter;
                ++other_iter;
            }

            mPoseMap.unlock();

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = true;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            std::cout << summary.FullReport() << std::endl;

            mPoseMap.lock();
            for (const auto& self_pose : selfPoseMap) {
                globalPoseMap[self_pose.first] = self_pose.second;
            }

            updateGlobalPath();
            mPoseMap.unlock();
        }
    }
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
