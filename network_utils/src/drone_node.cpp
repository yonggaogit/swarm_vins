#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <iostream>
#include <sstream>
#include "messages.pb.h"  // Include the generated protobuf header

using boost::asio::ip::tcp;

class DroneNode {
public:
    DroneNode(const std::string& ip, int port, int drone_id)
        : socket(io_service), endpoint(boost::asio::ip::address::from_string(ip), port), drone_id(drone_id) {
        socket.connect(endpoint);
        ROS_INFO("Connected to server at %s:%d", ip.c_str(), port);

        imu_sub = nh.subscribe("/vins_fusion/imu_propagate", 10, &DroneNode::imuCallback, this);
        path_sub = nh.subscribe("/vins_fusion/path", 10, &DroneNode::pathCallback, this);
        global_odom_sub = nh.subscribe("/ranging_fusion/global_odometry", 10, &DroneNode::globalOdomCallback, this);
        global_path_sub = nh.subscribe("/ranging_fusion/global_path", 10, &DroneNode::globalPathCallback, this);
    }

    void run() {
        ros::spin();
    }

private:
    void imuCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        drone::OdometryData odometry_data;
        odometry_data.set_type(drone::OdometryData::IMU_PROPAGATE);
        odometry_data.set_drone_id(drone_id);
        odometry_data.set_timestamp_sec(msg->header.stamp.sec);
        odometry_data.set_timestamp_nsec(msg->header.stamp.nsec);
        setPose(odometry_data.mutable_pose(), msg->pose.pose);

        sendProtobufData(odometry_data);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        drone::PathData path_data;
        path_data.set_type(drone::PathData::VINS_PATH);
        path_data.set_drone_id(drone_id);
        path_data.set_timestamp_sec(msg->header.stamp.sec);
        path_data.set_timestamp_nsec(msg->header.stamp.nsec);

        for (const auto& pose_stamped : msg->poses) {
            setPose(path_data.add_poses(), pose_stamped.pose);
        }
        std::cout << "=================================Send VINS Path===========================" << std::endl;
        sendProtobufData(path_data);
    }

    void globalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        drone::OdometryData odometry_data;
        odometry_data.set_type(drone::OdometryData::GLOBAL_ODOMETRY);
        odometry_data.set_drone_id(drone_id);
        odometry_data.set_timestamp_sec(msg->header.stamp.sec);
        odometry_data.set_timestamp_nsec(msg->header.stamp.nsec);
        setPose(odometry_data.mutable_pose(), msg->pose.pose);

        sendProtobufData(odometry_data);
    }

    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        drone::PathData path_data;
        path_data.set_type(drone::PathData::GLOBAL_PATH);
        path_data.set_drone_id(drone_id);
        path_data.set_timestamp_sec(msg->header.stamp.sec);
        path_data.set_timestamp_nsec(msg->header.stamp.nsec);

        for (const auto& pose_stamped : msg->poses) {
            setPose(path_data.add_poses(), pose_stamped.pose);
        }

        sendProtobufData(path_data);
    }

    void setPose(drone::Pose* proto_pose, const geometry_msgs::Pose& ros_pose) {
        proto_pose->set_x(ros_pose.position.x);
        proto_pose->set_y(ros_pose.position.y);
        proto_pose->set_z(ros_pose.position.z);
        proto_pose->set_qx(ros_pose.orientation.x);
        proto_pose->set_qy(ros_pose.orientation.y);
        proto_pose->set_qz(ros_pose.orientation.z);
        proto_pose->set_qw(ros_pose.orientation.w);
    }

    template <typename T>
    void sendProtobufData(const T& protobuf_data) {
        std::string serialized_data;
        protobuf_data.SerializeToString(&serialized_data);

        uint32_t data_length = serialized_data.size();
        std::vector<boost::asio::const_buffer> buffers;
        buffers.push_back(boost::asio::buffer(&data_length, sizeof(data_length)));
        buffers.push_back(boost::asio::buffer(serialized_data));

        boost::asio::write(socket, buffers);
    }

    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Subscriber path_sub;
    ros::Subscriber global_odom_sub;
    ros::Subscriber global_path_sub;

    boost::asio::io_service io_service;
    tcp::socket socket;
    tcp::endpoint endpoint;
    int drone_id;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_node");
    ros::NodeHandle nh("~");

    std::string ip;
    int port;
    int drone_id;

    nh.param("server_ip", ip, std::string("127.0.0.1"));
    nh.param("server_port", port, 9000);
    nh.param("drone_id", drone_id, 1);

    DroneNode drone_node(ip, port, drone_id);
    drone_node.run();

    return 0;
}