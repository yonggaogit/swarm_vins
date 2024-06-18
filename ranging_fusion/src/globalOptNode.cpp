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
#include <boost/bind.hpp>

GlobalOptimization* globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path;
nav_msgs::Path* global_path;
std::mutex m_buf;

std::queue<std::pair<nav_msgs::Odometry, nlink_parser::LinktrackNodeframe2>> odom_distane_queue;
std::queue<std::pair<int, nav_msgs::Odometry>> other_odom_queue;
std::deque<sensor_msgs::Imu> imu_buffer;

double last_vio_pos[3] = {-9,-9,-9};

void vio_path_callback(const nav_msgs::Path &vio_path_msg)
{
    if(USE_LOOP_VINS)
        globalEstimator->updateVIOPoseMap(vio_path_msg);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    // std::cout << "=======================================imu callback=================================" << std::endl;
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

    // 重力加速度
    Eigen::Vector3d gravity(0, 0, -9.81);

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

        // 加上重力加速度
        velocity += (orientation * acc + gravity) * dt;
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

void processData() {
    // std::cout << "===================================start process data============================" << std::endl;
    m_buf.lock();
    while (! odom_distane_queue.empty() && ! other_odom_queue.empty()) {
        // std::cout << odom_distane_queue.size() << "," << other_odom_queue.size() << std::endl;
        // std::cout << "===================================into queue 1============================" << std::endl;
        auto& [odom_msg, distance_msg] = odom_distane_queue.front();

        double self_time = odom_msg.header.stamp.toNSec()/1e9;

        auto& [drone_id, other_odom_msg] = other_odom_queue.front();
        double other_time = other_odom_msg.header.stamp.toNSec()/1e9;
        // std::cout << "drone_id: " << drone_id << std::endl;
        // std::cout << std::scientific << std::setprecision(30);
        // std::cout << "self_time:" << self_time << std::endl;
        // std::cout << "other_time:" << other_time << std::endl;
        // std::cout << "time_diff:" << fabs( self_time - other_time ) << std::endl;
        // std::cout << "time_tolerance:" << TIME_TOLERANCE << std::endl;
        // std::cout << "===================================into queue 2============================" << std::endl;
        if( fabs( self_time - other_time ) <= TIME_TOLERANCE )
        {
            // std::cout << "===============go to if1===============" << std::endl;
            double dis = -1.0;
            for( auto node : distance_msg.nodes ) {
                // std::cout << "node.id" << node.id << std::endl;
                if( node.id == drone_id - 1 ) {
                    dis = node.dis;
                    break;
                }
            }
            // std::cout << "dis:" << dis << std::endl;
            if( dis >= 0 ) {
                Eigen::Vector3d self_t(odom_msg.pose.pose.position.x,
                                    odom_msg.pose.pose.position.y,
                                    odom_msg.pose.pose.position.z);
                Eigen::Quaterniond self_q;
                self_q.w() = odom_msg.pose.pose.orientation.w;
                self_q.x() = odom_msg.pose.pose.orientation.x;
                self_q.y() = odom_msg.pose.pose.orientation.y;
                self_q.z() = odom_msg.pose.pose.orientation.z;

                Eigen::Vector3d self_velocity = Eigen::Vector3d(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z);


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

                // std::cout << "===================================same timestamp data============================" << std::endl;

                globalEstimator->inputSelf(self_time, self_t, self_q, self_velocity);
                globalEstimator->inputOther(other_time, other_t, other_q);
                globalEstimator->inputDis(dis_time, dis);
                
                odom_distane_queue.pop();
                other_odom_queue.pop();

                break;
            } else {
                odom_distane_queue.pop();
            }
        }
        else if(self_time < other_time) {
            // std::cout << "===============go to if2===============" << std::endl;
            odom_distane_queue.pop();
        }
        else {
            // std::cout << "===============go to if3===============" << std::endl;
            other_odom_queue.pop();
        }
        // std::cout << odom_distane_queue.size() << "," << other_odom_queue.size() << std::endl;
    }
    m_buf.unlock();
    // std::cout << "=============================youhua============================" << std::endl;
    Eigen::Vector3d global_p;
    Eigen:: Quaterniond global_q;
    Eigen::Vector3d global_v;
    double global_t = 0.0;
    globalEstimator->getGlobalOdom(global_t, global_p, global_q, global_v);
    // // 输出 Vector3d
    // std::cout << "global_p: " << global_p.transpose() << std::endl;

    // // 输出 Quaterniond
    // std::cout << "global_q: " 
    //           << global_q.w() << " " 
    //           << global_q.x() << " " 
    //           << global_q.y() << " " 
    //           << global_q.z() << std::endl;

    // // 输出 double
    // std::cout << "global_t: " << global_t << std::endl;
    nav_msgs::Odometry odometry;
    // if ( global_t != 0.0 ) {
    //     odometry.header.stamp = ros::Time(global_t);
    // } else odometry.header.stamp = ros::Time::now();
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "";
    odometry.pose.pose.position.x = global_p.x();
    odometry.pose.pose.position.y = global_p.y();
    odometry.pose.pose.position.z = global_p.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    odometry.twist.twist.linear.x = global_v.x();
    odometry.twist.twist.linear.y = global_v.y();
    odometry.twist.twist.linear.z = global_v.z();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
}


void odomDistanceCallback(const nav_msgs::Odometry::ConstPtr& self_odom_msg, const nlink_parser::LinktrackNodeframe2::ConstPtr& distance_msg) { 
    // std::cout << "===================================odom distance callback============================" << std::endl;   
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
    // std::cout << "=====================size==========================" << distance_msg->nodes[0].id << std::endl;
    m_buf.lock();
    std::pair<nav_msgs::Odometry, nlink_parser::LinktrackNodeframe2> tmpPair = std::make_pair(*self_odom_msg, *distance_msg);
    odom_distane_queue.push(tmpPair);
    m_buf.unlock();

    processData();
}

void otherOdomCallback(const nav_msgs::Odometry::ConstPtr& msg, const int& drone_id) {
    // std::cout << "=========================other odom start=============================" << std::endl;
    m_buf.lock();
    other_odom_queue.push(make_pair(drone_id, *msg));
    m_buf.unlock();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_fusion_node");
    ros::NodeHandle n("~");

    readParameters(n);
    globalEstimator = new GlobalOptimization(WINDOW_SIZE);
    global_path = &globalEstimator->global_path;
    std::string pose_graph_path_topic, self_odom_topic, linktrack_nodeframe_topic, imu_topic;
    n.param<std::string>("pose_graph_path_topic", pose_graph_path_topic, "/pose_graph/path_1");
    n.param<std::string>("self_odom_topic", self_odom_topic, "/vins_fusion/imu_propagate");
    n.param<std::string>("linktrack_nodeframe_topic", linktrack_nodeframe_topic, "/nlink_linktrack_nodeframe2");
    n.param<std::string>("imu_topic", imu_topic, "/mavros/imu/data_raw");

    ros::Subscriber sub_vio_path = n.subscribe(pose_graph_path_topic, 1000, vio_path_callback);
    ros::Subscriber sub_imu = n.subscribe(imu_topic, 100, imu_callback);

    int drone_id, num_drones;
    n.param("drone_id", drone_id, 1);
    n.param("num_drones", num_drones, 2);
    ros::Subscriber sub_other_odom_1 = n.subscribe<nav_msgs::Odometry>("/drone_1/vins_fusion/imu_propagate", 10, boost::bind(otherOdomCallback, _1, 1));
    ros::Subscriber sub_other_odom_2 = n.subscribe<nav_msgs::Odometry>("/drone_2/vins_fusion/imu_propagate", 10, boost::bind(otherOdomCallback, _1, 2));
    ros::Subscriber sub_other_odom_3 = n.subscribe<nav_msgs::Odometry>("/drone_3/vins_fusion/imu_propagate", 10, boost::bind(otherOdomCallback, _1, 3));
    ros::Subscriber sub_other_odom_4 = n.subscribe<nav_msgs::Odometry>("/drone_4/vins_fusion/imu_propagate", 10, boost::bind(otherOdomCallback, _1, 4));
    ros::Subscriber sub_other_odom_5 = n.subscribe<nav_msgs::Odometry>("/drone_5/vins_fusion/imu_propagate", 10, boost::bind(otherOdomCallback, _1, 5));

    
    message_filters::Subscriber<nav_msgs::Odometry> self_odom_sub(n, self_odom_topic, 1);
    message_filters::Subscriber<nlink_parser::LinktrackNodeframe2> distance_sub(n, linktrack_nodeframe_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nlink_parser::LinktrackNodeframe2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), self_odom_sub, distance_sub);
    // sync.setMaxIntervalDuration(ros::Duration(TIME_TOLERANCE));
    sync.registerCallback(boost::bind(&odomDistanceCallback, _1, _2));

    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 1000);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 1000);

    ros::spin();

    delete globalEstimator;
    return 0;
}