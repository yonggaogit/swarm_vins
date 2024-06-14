#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <google/protobuf/util/time_util.h>
#include "messages.pb.h"

using boost::asio::ip::tcp;

enum OdometryMessageType {
    IMU_PROPAGATE,
    GLOBAL_ODOMETRY
};

enum PathMessageType {
    VINS_PATH,
    GLOBAL_PATH
};

class DroneNode {
public:
    DroneNode(int id, const std::string& server_ip, int server_port, double offset_multiplier)
        : drone_id(id), io_service(), socket(io_service), offset_multiplier(offset_multiplier) {
        try {
            tcp::resolver resolver(io_service);
            tcp::resolver::query query(server_ip, std::to_string(server_port));
            tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
            boost::asio::connect(socket, endpoint_iterator);
            ROS_INFO("Connected to server %s:%d", server_ip.c_str(), server_port);
        } catch (boost::system::system_error& e) {
            ROS_ERROR("Failed to connect to server: %s", e.what());
            ros::shutdown();
        }

        imu_sub = nh.subscribe("/vins_fusion/imu_propagate", 10, &DroneNode::imuCallback, this);
        path_sub = nh.subscribe("/vins_fusion/path", 10, &DroneNode::pathCallback, this);
        global_odometry_sub = nh.subscribe("/ranging_fusion/global_odometry", 10, &DroneNode::globalOdometryCallback, this);
        global_path_sub = nh.subscribe("/ranging_fusion/global_path", 10, &DroneNode::globalPathCallback, this);
    }

    void imuCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        sendOdometryData(OdometryMessageType::IMU_PROPAGATE, msg->header.stamp, msg->pose.pose.position, msg->pose.pose.orientation);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (!msg->poses.empty()) {
            nav_msgs::Path path_msg = *msg;
            sendPathData(PathMessageType::VINS_PATH, path_msg);
        } else {
            ROS_WARN("Received empty Path message");
        }
    }

    void globalOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::Point position = msg->pose.pose.position;
        position.y += modifyYCoordinate();
        sendOdometryData(OdometryMessageType::GLOBAL_ODOMETRY, msg->header.stamp, position, msg->pose.pose.orientation);
    }

    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        std::cout << "==========================global path======================" << std::endl;
        if (!msg->poses.empty()) {
            nav_msgs::Path path_msg = *msg;
            for (auto& pose : path_msg.poses) {
                pose.pose.position.y += modifyYCoordinate();
            }
            sendPathData(PathMessageType::GLOBAL_PATH, path_msg);
        } else {
            ROS_WARN("Received empty Global Path message");
        }
    }

private:
    double modifyYCoordinate() {
        return (drone_id - 1) * offset_multiplier;
    }

    void sendOdometryData(OdometryMessageType type, const ros::Time& timestamp, const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation) {
        try {
            drone::OdometryData odom_data;
            odom_data.set_type(static_cast<drone::OdometryData::Type>(type));
            odom_data.set_drone_id(drone_id);
            odom_data.set_timestamp_sec(timestamp.sec);
            odom_data.set_timestamp_nsec(timestamp.nsec);

            auto pose = odom_data.mutable_pose();
            pose->set_x(position.x);
            pose->set_y(position.y);
            pose->set_z(position.z);
            pose->set_qx(orientation.x);
            pose->set_qy(orientation.y);
            pose->set_qz(orientation.z);
            pose->set_qw(orientation.w);

            std::string outbound_data;
            odom_data.SerializeToString(&outbound_data);

            uint32_t data_length = outbound_data.size();
            boost::asio::write(socket, boost::asio::buffer(&data_length, sizeof(data_length)));
            boost::asio::write(socket, boost::asio::buffer(outbound_data));

            // ROS_INFO("Sent OdometryData of type %d", type);
        } catch (boost::system::system_error& e) {
            ROS_ERROR("Failed to send data: %s", e.what());
        }
    }

    void sendPathData(PathMessageType type, const nav_msgs::Path& path_msg) {
        try {
            drone::PathData path_data;
            path_data.set_type(static_cast<drone::PathData::Type>(type));
            path_data.set_drone_id(drone_id);
            path_data.set_seq(path_msg.header.seq);
            path_data.set_timestamp_sec(path_msg.header.stamp.sec);
            path_data.set_timestamp_nsec(path_msg.header.stamp.nsec);

            for (const auto& pose_stamped : path_msg.poses) {
                auto pose = path_data.add_poses();
                pose->set_timestamp_sec(pose_stamped.header.stamp.sec);
                pose->set_timestamp_nsec(pose_stamped.header.stamp.nsec);
                pose->set_x(pose_stamped.pose.position.x);
                pose->set_y(pose_stamped.pose.position.y);
                pose->set_z(pose_stamped.pose.position.z);
                pose->set_qx(pose_stamped.pose.orientation.x);
                pose->set_qy(pose_stamped.pose.orientation.y);
                pose->set_qz(pose_stamped.pose.orientation.z);
                pose->set_qw(pose_stamped.pose.orientation.w);
            }

            std::string outbound_data;
            if (!path_data.SerializeToString(&outbound_data)) {
                ROS_ERROR("Failed to serialize PathData");
                return;
            }

            uint32_t data_length = outbound_data.size();
            boost::asio::write(socket, boost::asio::buffer(&data_length, sizeof(data_length)));
            boost::asio::write(socket, boost::asio::buffer(outbound_data));

            // ROS_INFO("Sent PathData of type %d", type);
        } catch (boost::system::system_error& e) {
            ROS_ERROR("Failed to send path data: %s", e.what());
        }
    }

    ros::NodeHandle nh;
    int drone_id;
    double offset_multiplier;
    ros::Subscriber imu_sub, path_sub, global_odometry_sub, global_path_sub;
    boost::asio::io_service io_service;
    tcp::socket socket;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_node");
    ros::NodeHandle nh("~");

    int drone_id;
    std::string server_ip;
    int server_port;
    double offset_multiplier;

    nh.param("drone_id", drone_id, 1);
    nh.param("server_ip", server_ip, std::string("127.0.0.1"));
    nh.param("server_port", server_port, 9000);
    nh.param("offset_multiplier", offset_multiplier, 1.0);

    DroneNode drone_node(drone_id, server_ip, server_port, offset_multiplier);

    ros::spin();

    return 0;
}