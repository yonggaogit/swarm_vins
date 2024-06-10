#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include "messages.pb.h"  // Include the generated protobuf header

using boost::asio::ip::tcp;

class ServerNode {
public:
    ServerNode(const std::string& ip, int port) 
        : io_service(), acceptor(io_service, tcp::endpoint(boost::asio::ip::address::from_string(ip), port)) {
        startAccept();
        ROS_INFO("Server started on %s:%d", ip.c_str(), port);
    }

    void run() {
        io_service.run();
    }

private:
    void startAccept() {
        tcp::socket* socket = new tcp::socket(io_service);
        acceptor.async_accept(*socket, boost::bind(&ServerNode::handleAccept, this, socket, boost::asio::placeholders::error));
    }

    void handleAccept(tcp::socket* socket, const boost::system::error_code& error) {
        if (!error) {
            boost::thread(boost::bind(&ServerNode::handleClient, this, socket));
        }
        startAccept();
    }

    void handleClient(tcp::socket* socket) {
        try {
            for (;;) {
                uint32_t data_length = 0;
                boost::asio::read(*socket, boost::asio::buffer(&data_length, sizeof(data_length)));
                std::vector<char> data(data_length);
                boost::asio::read(*socket, boost::asio::buffer(data.data(), data_length));

                std::string serialized_data(data.begin(), data.end());

                if (handleProtobufData(serialized_data)) {
                    ROS_INFO("Processed protobuf data successfully");
                } else {
                    ROS_ERROR("Failed to process protobuf data");
                }
            }
        } catch (std::exception& e) {
            ROS_ERROR("Client handling error: %s", e.what());
        } catch (...) {
            ROS_ERROR("Unknown error occurred while handling client");
        }
        delete socket;
    }

    bool handleProtobufData(const std::string& serialized_data) {
        if (serialized_data.size() < 4) {
            ROS_ERROR("Invalid data format");
            return false;
        }

        drone::OdometryData odometry_data;
        drone::PathData path_data;

        if (odometry_data.ParseFromString(serialized_data)) {
            return handleOdometryData(odometry_data);
        } else if (path_data.ParseFromString(serialized_data)) {
            return handlePathData(path_data);
        } else {
            ROS_ERROR("Failed to parse protobuf data");
            return false;
        }
    }

    bool handleOdometryData(const drone::OdometryData& odometry_data) {
        nav_msgs::Odometry msg;
        msg.header.stamp.sec = odometry_data.timestamp_sec();
        msg.header.stamp.nsec = odometry_data.timestamp_nsec();
        msg.header.frame_id = "world";
        
        msg.pose.pose.position.x = odometry_data.pose().x();
        msg.pose.pose.position.y = odometry_data.pose().y();
        msg.pose.pose.position.z = odometry_data.pose().z();
        msg.pose.pose.orientation.x = odometry_data.pose().qx();
        msg.pose.pose.orientation.y = odometry_data.pose().qy();
        msg.pose.pose.orientation.z = odometry_data.pose().qz();
        msg.pose.pose.orientation.w = odometry_data.pose().qw();

        std::string base_topic;
        if (odometry_data.type() == drone::OdometryData::IMU_PROPAGATE) {
            base_topic = "/vins_fusion/imu_propagate";
        } else if (odometry_data.type() == drone::OdometryData::GLOBAL_ODOMETRY) {
            base_topic = "/ranging_fusion/global_odometry";
        } else {
            ROS_ERROR("Unknown odometry type: %d", odometry_data.type());
            return false;
        }

        std::string topic = "/drone_" + std::to_string(odometry_data.drone_id()) + base_topic;
        getPublisher<nav_msgs::Odometry>(topic).publish(msg);

        ROS_INFO("Published odometry data to %s", topic.c_str());
        return true;
    }

    bool handlePathData(const drone::PathData& path_data) {
        nav_msgs::Path msg;
        msg.header.stamp.sec = path_data.timestamp_sec();
        msg.header.stamp.nsec = path_data.timestamp_nsec();
        msg.header.frame_id = "world";

        for (const auto& pose : path_data.poses()) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose.position.x = pose.x();
            pose_stamped.pose.position.y = pose.y();
            pose_stamped.pose.position.z = pose.z();
            pose_stamped.pose.orientation.x = pose.qx();
            pose_stamped.pose.orientation.y = pose.qy();
            pose_stamped.pose.orientation.z = pose.qz();
            pose_stamped.pose.orientation.w = pose.qw();
            msg.poses.push_back(pose_stamped);
        }

        std::string base_topic;
        if (path_data.type() == drone::PathData::VINS_PATH) {
            base_topic = "/vins_fusion/path";
        } else if (path_data.type() == drone::PathData::GLOBAL_PATH) {
            base_topic = "/ranging_fusion/global_path";
        } else {
            ROS_ERROR("Unknown path type: %d", path_data.type());
            return false;
        }

        std::string topic = "/drone_" + std::to_string(path_data.drone_id()) + base_topic;
        getPublisher<nav_msgs::Path>(topic).publish(msg);

        ROS_INFO("Published path data to %s", topic.c_str());
        return true;
    }

    template<typename T>
    ros::Publisher& getPublisher(const std::string& topic) {
        auto it = publishers.find(topic);
        if (it == publishers.end()) {
            ros::Publisher pub = nh.advertise<T>(topic, 10);
            it = publishers.emplace(topic, pub).first;
        }
        return it->second;
    }

    ros::NodeHandle nh;
    boost::asio::io_service io_service;
    tcp::acceptor acceptor;
    std::unordered_map<std::string, ros::Publisher> publishers;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "server_node");
    ros::NodeHandle nh("~");

    std::string ip;
    int port;

    nh.param("server_ip", ip, std::string("127.0.0.1"));
    nh.param("server_port", port, 9000);

    ServerNode server_node(ip, port);

    boost::thread server_thread(boost::bind(&ServerNode::run, &server_node));

    ros::spin();

    server_thread.join();

    return 0;
}