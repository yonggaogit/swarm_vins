#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "Factors.h"
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/SVD>
#include "parameters.h"
using namespace std;

class GlobalOptimization
{
public:
	GlobalOptimization(int window_size);
	~GlobalOptimization();
	void inputSelf(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void inputOther(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void inputDis(double t, double dis);


	void getGlobalOdom(double &T, Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
    void updateVIOPoseMap(const nav_msgs::Path &vio_path_msg);
	nav_msgs::Path global_path;

private:
	void optimize();
	void updateGlobalPath();
	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> globalPoseMap;

	map<double, vector<double>> selfPoseMap;
	map<double, vector<double>> otherPoseMap;
	map<double, double> disMap;
	bool newVIO;
	std::mutex mPoseMap;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	double lastT;
	std::thread threadOpt;

	int window_size_;
};