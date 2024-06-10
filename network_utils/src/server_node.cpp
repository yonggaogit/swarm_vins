#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <zlib.h>
#include "messages.pb.h"

using boost::asio::ip::tcp;

enum MessageType {
    IMU_PROPAGATE,
    VINS_PATH,
    GLOBAL_ODOMETRY,
    GLOBAL_PATH
};

class ServerNode {
public:
    ServerNode(const std::string& ip, int port) 
        : io_service(), acceptor(io_service, tcp::endpoint(boost::asio::ip::address::from_string(ip), port)) {
        startAccept();
        ROS_INFO("Server started on %s:%d", ip.c_str(), port);
    }

    void run() {
        boost::thread_group threadpool;
        for (std::size_t i = 0; i < boost::thread::hardware_concurrency(); ++i) {
            threadpool.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
        }
        threadpool.join_all();
    }

private:
    void startAccept() {
        tcp::socket* new_socket = new tcp::socket(io_service);
        acceptor.async_accept(*new_socket, boost::bind(&ServerNode::handleAccept, this, new_socket, boost::asio::placeholders::error));
    }

    void handleAccept(tcp::socket* new_socket, const boost::system::error_code& error) {
        if (!error) {
            ROS_INFO("New client connected");
            boost::thread(boost::bind(&ServerNode::handleClient, this, new_socket)).detach();
        } else {
            ROS_ERROR("Accept error: %s", error.message().c_str());
            delete new_socket;
        }
        startAccept();
    }

    void handleClient(tcp::socket* client_socket) {
        try {
            while (true) {
                uint32_t data_length;
                boost::asio::read(*client_socket, boost::asio::buffer(&data_length, sizeof(data_length)));

                std::vector<char> inbound_data(data_length);
                boost::asio::read(*client_socket, boost::asio::buffer(inbound_data.data(), inbound_data.size()));

                drone::OdometryData odom_data;
                drone::PathData path_data;

                if (odom_data.ParseFromArray(inbound_data.data(), data_length)) {
                    handleOdometryData(odom_data);
                } else if (path_data.ParseFromArray(inbound_data.data(), data_length)) {
                    handlePathData(path_data);
                } else {
                    ROS_ERROR("Failed to parse received data");
                }
            }
        } catch (boost::system::system_error& e) {
            ROS_ERROR("Client connection error: %s", e.what());
            delete client_socket;
        }
    }

    void handleOdometryData(const drone::OdometryData& odom_data) {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time(odom_data.timestamp_sec(), odom_data.timestamp_nsec());
        odom_msg.pose.pose.position.x = odom_data.pose().x();
        odom_msg.pose.pose.position.y = odom_data.pose().y();
        odom_msg.pose.pose.position.z = odom_data.pose().z();
        odom_msg.pose.pose.orientation.x = odom_data.pose().qx();
        odom_msg.pose.pose.orientation.y = odom_data.pose().qy();
        odom_msg.pose.pose.orientation.z = odom_data.pose().qz();
        odom_msg.pose.pose.orientation.w = odom_data.pose().qw();

        std::string topic_name;
        switch (odom_data.type()) {
            case drone::OdometryData::IMU_PROPAGATE:
                topic_name = "/drone_" + std::to_string(odom_data.drone_id()) + "/vins_fusion/imu_propagate";
                break;
            case drone::OdometryData::GLOBAL_ODOMETRY:
                topic_name = "/drone_" + std::to_string(odom_data.drone_id()) + "/ranging_fusion/global_odometry";
                break;
            default:
                ROS_ERROR("Unknown OdometryData type: %d", odom_data.type());
                return;
        }

        ros::Publisher odom_pub = getPublisher<nav_msgs::Odometry>(topic_name);
        odom_pub.publish(odom_msg);
        ROS_INFO("Published OdometryData to %s", topic_name.c_str());
    }

    void handlePathData(const drone::PathData& path_data) {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time(path_data.timestamp_sec(), path_data.timestamp_nsec());
        path_msg.header.frame_id = "world"; // 设置适当的frame_id

        for (int i = 0; i < path_data.poses_size(); ++i) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "world"; // 设置适当的frame_id
            pose_stamped.pose.position.x = path_data.poses(i).x();
            pose_stamped.pose.position.y = path_data.poses(i).y();
            pose_stamped.pose.position.z = path_data.poses(i).z();
            pose_stamped.pose.orientation.x = path_data.poses(i).qx();
            pose_stamped.pose.orientation.y = path_data.poses(i).qy();
            pose_stamped.pose.orientation.z = path_data.poses(i).qz();
            pose_stamped.pose.orientation.w = path_data.poses(i).qw();
            path_msg.poses.push_back(pose_stamped);
        }

        std::string topic_name;
        switch (path_data.type()) {
            case drone::PathData::VINS_PATH:
                topic_name = "/drone_" + std::to_string(path_data.drone_id()) + "/vins_fusion/path";
                break;
            case drone::PathData::GLOBAL_PATH:
                topic_name = "/drone_" + std::to_string(path_data.drone_id()) + "/ranging_fusion/global_path";
                break;
            default:
                ROS_ERROR("Unknown PathData type: %d", path_data.type());
                return;
        }

        ros::Publisher path_pub = getPublisher<nav_msgs::Path>(topic_name);
        path_pub.publish(path_msg);
        ROS_INFO("Published PathData to %s", topic_name.c_str());
    }

    template <typename T>
    ros::Publisher getPublisher(const std::string& topic_name) {
        if (publishers.find(topic_name) == publishers.end()) {
            publishers[topic_name] = nh.advertise<T>(topic_name, 10);
        }
        return publishers[topic_name];
    }

    ros::NodeHandle nh;
    boost::asio::io_service io_service;
    tcp::acceptor acceptor;
    std::map<std::string, ros::Publisher> publishers;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "server_node");
    ros::NodeHandle nh("~");

    std::string server_ip;
    int server_port;

    nh.param("server_ip", server_ip, std::string("127.0.0.1"));
    nh.param("server_port", server_port, 9000);

    ServerNode server_node(server_ip, server_port);
    server_node.run();

    return 0;
}