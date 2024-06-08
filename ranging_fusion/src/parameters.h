//
// Created by jacy on 2022/4/16.
//
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int USE_LOOP_VINS;
extern double TIME_TOLERANCE;
extern double MOVE_DIS;
extern int IMU_BUFFER_SIZE;
extern int WINDOW_SIZE;
void readParameters(ros::NodeHandle &n);