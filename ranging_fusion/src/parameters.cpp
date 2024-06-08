//
// Created by jacy on 2022/4/16.
//
#include "parameters.h"

int USE_LOOP_VINS;
int IMU_BUFFER_SIZE;
int WINDOW_SIZE;
double TIME_TOLERANCE;
double MOVE_DIS;
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "uwb_vio_config");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    USE_LOOP_VINS = fsSettings["use_loop_vins"];
    TIME_TOLERANCE = fsSettings["time_tolerance"];
    MOVE_DIS = fsSettings["move_dis"];
    IMU_BUFFER_SIZE = fsSettings["imu_buffer_size"];
    WINDOW_SIZE = fsSettings["window_size"];
    fsSettings.release();

    std::cout<<"use_loop_vins:"<< USE_LOOP_VINS <<std::endl;
    std::cout<<"time_tolerance:"<< TIME_TOLERANCE <<std::endl;
    std::cout<<"imu_buffer_size:"<< IMU_BUFFER_SIZE <<std::endl;
    std::cout<<"move_dis:"<< MOVE_DIS <<std::endl;
    std::cout<<"window_size:"<< WINDOW_SIZE <<std::endl;
}
