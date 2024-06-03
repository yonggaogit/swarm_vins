#include "ros/ros.h"
#include "globalOpt.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <queue>
#include <deque>
#include <mutex>
#include <vector>
#include <opencv2/core/core.hpp>
#include <unordered_map>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "parameters.h"
#include "nlink_parser/LinktrackNodeframe2.h"
#include "nlink_parser/LinktrackNode2.h"

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path;
nav_msgs::Path *global_path;
std::mutex m_buf;

std::queue<std::pair<nav_msgs::Odometry, nlink_parser::LinktrackNodeframe2>> odom_distane_queue;
std::queue<std::pair<int, nav_msgs::Odometry>> other_odom_queue;
std::deque<sensor_msgs::Imu> imu_buffer;

double last_vio_pos[3] = {-9,-9,-9};

void vio_path_callback(const nav_msgs::Path &vio_path_msg)
{
    if(USE_LOOP_VINS)
        globalEstimator.updateVIOPoseMap(vio_path_msg);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    m_buf.lock();
    imu_buffer.push_back(*imu_msg);
    while (imu_buffer.size() > IMU_BUFFER_SIZE)
    {
        imu_buffer.pop_front();
    }
    m_buf.unlock();
}

nav_msgs::Odometry preIntegrateImu(const nav_msgs::Odometry &odom_msg, const ros::Time &target_time)
{
    Eigen::Vector3d position = Eigen::Vector3d(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z);
    Eigen::Quaterniond orientation;
    orientation.w() = odom_msg.pose.pose.orientation.w;
    orientation.x() = odom_msg.pose.pose.orientation.x;
    orientation.y() = odom_msg.pose.pose.orientation.y;
    orientation.z() = odom_msg.pose.pose.orientation.z;
    Eigen::Vector3d velocity = Eigen::Vector3d(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z);

    ros::Time current_time = odom_msg.header.stamp;
    for (const auto &imu_msg : imu_buffer)
    {
        if (imu_msg.header.stamp <= current_time)
            continue;
        if (imu_msg.header.stamp > target_time)
            break;

        double dt = (imu_msg.header.stamp - current_time).toSec();
        Eigen::Vector3d acc(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
        Eigen::Vector3d gyro(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);

        Eigen::Quaterniond dq;
        dq = Eigen::AngleAxisd(gyro.norm() * dt, gyro.normalized());
        orientation = orientation * dq;

        velocity += orientation * acc * dt;
        position += velocity * dt;

        current_time = imu_msg.header.stamp;
    }

    nav_msgs::Odometry integrated_odom = odom_msg;
    integrated_odom.header.stamp = target_time;
    integrated_odom.pose.pose.position.x = position.x();
    integrated_odom.pose.pose.position.y = position.y();
    integrated_odom.pose.pose.position.z = position.z();
    integrated_odom.pose.pose.orientation.w = orientation.w();
    integrated_odom.pose.pose.orientation.x = orientation.x();
    integrated_odom.pose.pose.orientation.y = orientation.y();
    integrated_odom.pose.pose.orientation.z = orientation.z();
    integrated_odom.twist.twist.linear.x = velocity.x();
    integrated_odom.twist.twist.linear.y = velocity.y();
    integrated_odom.twist.twist.linear.z = velocity.z();

    return integrated_odom;
}


void odomDistanceCallback(const nav_msgs::Odometry::ConstPtr& self_odom_msg, const nlink_parser::LinktrackNodeframe2::ConstPtr& distance_msg) {
    if (fabs(self_odom_msg->header.stamp.toSec() - distance_msg->header.stamp.toSec()) > TIME_TOLERANCE)
    {
        m_buf.lock();
        nav_msgs::Odometry adjusted_odom = preIntegrateImu(*self_odom_msg, distance_msg->header.stamp);
        m_buf.unlock();
        odomDistanceCallback(boost::make_shared<nav_msgs::Odometry>(adjusted_odom), distance_msg);
        return;
    }

    double x_dis = self_odom_msg->pose.pose.position.x - last_vio_pos[0];
    double y_dis = self_odom_msg->pose.pose.position.y - last_vio_pos[1];
    double z_dis = self_odom_msg->pose.pose.position.z - last_vio_pos[2];
    double dis = sqrt(x_dis * x_dis + y_dis * y_dis + z_dis * z_dis);
    if (dis < MOVE_DIS)
        return;

    last_vio_pos[0] = self_odom_msg->pose.pose.position.x;
    last_vio_pos[1] = self_odom_msg->pose.pose.position.y;
    last_vio_pos[2] = self_odom_msg->pose.pose.position.z;

    m_buf.lock();
    std::pair<nav_msgs::Odometry, nlink_parser::LinktrackNodeframe2> tmpPair = std::make_pair(*self_odom_msg, *distance_msg);
    odom_distane_queue.push(tmpPair);
    m_buf.unlock();

    pub_global_path.publish(*global_path);
}

void processData() {
    m_buf.lock();
    while (! odom_distane_queue.empty() && ! other_odom_queue.empty()) {
        auto& [odom_msg, distance_msg] = odom_distane_queue.front();

        double self_time = odom_msg.header.stamp.toNSec()/1e9;

        auto& [drone_id, other_odom_msg] = other_odom_queue.front();
        double other_time = other_odom_msg.header.stamp.toNSec()/1e9;
        if( abs( self_time - other_time ) <= TIME_TOLERANCE )
        {
            double dis = -1.0;
            for( auto node : distance_msg.nodes ) {
                if( node.id == drone_id ) {
                    dis = node.dis;
                    break;
                }
            }
            if( dis >= 0 ) {
                Eigen::Vector3d self_t(odom_msg.pose.pose.position.x,
                                    odom_msg.pose.pose.position.y,
                                    odom_msg.pose.pose.position.z);
                Eigen::Quaterniond self_q;
                self_q.w() = odom_msg.pose.pose.orientation.w;
                self_q.x() = odom_msg.pose.pose.orientation.x;
                self_q.y() = odom_msg.pose.pose.orientation.y;
                self_q.z() = odom_msg.pose.pose.orientation.z;


                Eigen::Vector3d other_t(other_odom_msg.pose.pose.position.x,
                                    other_odom_msg.pose.pose.position.y,
                                    other_odom_msg.pose.pose.position.z);
                Eigen::Quaterniond other_q;
                other_q.w() = other_odom_msg.pose.pose.orientation.w;
                other_q.x() = other_odom_msg.pose.pose.orientation.x;
                other_q.y() = other_odom_msg.pose.pose.orientation.y;
                other_q.z() = other_odom_msg.pose.pose.orientation.z;

                double self_time = odom_msg.header.stamp.toNSec()/1e9;
                double other_time = other_odom_msg.header.stamp.toNSec()/1e9;
                double dis_time = distance_msg.header.stamp.toNSec()/1e9;

                globalEstimator.inputSelf(self_time, self_t, self_q);
                globalEstimator.inputOther(other_time, other_t, other_q);
                globalEstimator.inputDis(dis_time, dis);
                
                odom_distane_queue.pop();
                other_odom_queue.pop();

                break;
            }
        }
        else if(self_time < other_time) {
            odom_distane_queue.pop();
        }
        else {
            other_odom_queue.pop();
        }

    }
    m_buf.unlock();
}

void otherOdomCallback(const nav_msgs::Odometry::ConstPtr& msg, const int& drone_id) {
    m_buf.lock();
    other_odom_queue.push(make_pair(drone_id, *msg));
    m_buf.unlock();

    processData();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_fusion_node");
    ros::NodeHandle n("~");

    readParameters(n);

    global_path = &globalEstimator.global_path;
    ros::Subscriber sub_vio_path = n.subscribe("/pose_graph/path_1",1000, vio_path_callback);
    
    ros::Subscriber drone_1_odom_sub = n.subscribe<nav_msgs::Odometry>("/drone_1/vins_estimator/odometry", 1000, 
        boost::bind(otherOdomCallback, _1, 1 ));
    ros::Subscriber drone_2_odom_sub = n.subscribe<nav_msgs::Odometry>("/drone_2/vins_estimator/odometry", 1000, 
        boost::bind(otherOdomCallback, _1, 2 ));
    ros::Subscriber drone_3_odom_sub = n.subscribe<nav_msgs::Odometry>("/drone_3/vins_estimator/odometry", 1000, 
        boost::bind(otherOdomCallback, _1, 3 ));

    
    message_filters::Subscriber<nav_msgs::Odometry> self_odom_sub(n, "/vins_estimator/odometry", 1);
    message_filters::Subscriber<nlink_parser::LinktrackNodeframe2> distance_sub(n, "/nlink_linktrack_nodeframe2", 1);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nlink_parser::LinktrackNodeframe2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), self_odom_sub, distance_sub);
    sync.registerCallback(boost::bind(&odomDistanceCallback, _1, _2));

    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 1000);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 1000);

    ros::spin();

    return 0;
}
