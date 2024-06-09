#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <thread>
#include <unordered_map>

using boost::asio::ip::tcp;

ros::Publisher createPublisher(ros::NodeHandle &nh, const std::string &topic_name, const std::string &msg_type) {
    if (msg_type == "Odometry") {
        return nh.advertise<nav_msgs::Odometry>(topic_name, 10);
    } else if (msg_type == "Path") {
        return nh.advertise<nav_msgs::Path>(topic_name, 10);
    }
    return ros::Publisher();
}

void handleClient(tcp::socket socket, ros::NodeHandle &nh, std::unordered_map<std::string, ros::Publisher> &publishers) {
    try {
        boost::array<char, 8192> buf;
        boost::system::error_code error;
        while (true) {
            size_t len = socket.read_some(boost::asio::buffer(buf), error);
            if (error == boost::asio::error::eof)
                break; // Connection closed cleanly by peer.
            else if (error)
                throw boost::system::system_error(error); // Some other error.

            std::istringstream iss(std::string(buf.data(), len));
            boost::property_tree::ptree pt;
            boost::property_tree::read_json(iss, pt);

            int drone_id = pt.get<int>("drone_id");
            std::string topic_name = pt.get<std::string>("topic_name");
            std::string full_topic_name = "/drone_" + std::to_string(drone_id) + "/" + topic_name;
            std::string msg_type = pt.get<std::string>("msg_type");

            if (publishers.find(full_topic_name) == publishers.end()) {
                publishers[full_topic_name] = createPublisher(nh, full_topic_name, msg_type);
            }

            if (msg_type == "Odometry") {
                nav_msgs::Odometry odom_msg;
                odom_msg.header.stamp = ros::Time(pt.get<uint64_t>("msg.header.stamp"));
                odom_msg.pose.pose.position.x = pt.get<double>("msg.pose.pose.position.x");
                odom_msg.pose.pose.position.y = pt.get<double>("msg.pose.pose.position.y");
                odom_msg.pose.pose.position.z = pt.get<double>("msg.pose.pose.position.z");
                odom_msg.pose.pose.orientation.x = pt.get<double>("msg.pose.pose.orientation.x");
                odom_msg.pose.pose.orientation.y = pt.get<double>("msg.pose.pose.orientation.y");
                odom_msg.pose.pose.orientation.z = pt.get<double>("msg.pose.pose.orientation.z");
                odom_msg.pose.pose.orientation.w = pt.get<double>("msg.pose.pose.orientation.w");
                publishers[full_topic_name].publish(odom_msg);
            } else if (msg_type == "Path") {
                nav_msgs::Path path_msg;
                path_msg.header.stamp = ros::Time(pt.get<uint64_t>("msg.header.stamp"));
                
                for (const auto &pose_pt : pt.get_child("msg.poses")) {
                    geometry_msgs::PoseStamped pose;
                    pose.pose.position.x = pose_pt.second.get<double>("position.x");
                    pose.pose.position.y = pose_pt.second.get<double>("position.y");
                    pose.pose.position.z = pose_pt.second.get<double>("position.z");
                    pose.pose.orientation.x = pose_pt.second.get<double>("orientation.x");
                    pose.pose.orientation.y = pose_pt.second.get<double>("orientation.y");
                    pose.pose.orientation.z = pose_pt.second.get<double>("orientation.z");
                    pose.pose.orientation.w = pose_pt.second.get<double>("orientation.w");
                    path_msg.poses.push_back(pose);
                }
                publishers[full_topic_name].publish(path_msg);
            }
        }
    }
    catch (std::exception &e) {
        ROS_ERROR("Exception in thread: %s", e.what());
    }
}

void server(ros::NodeHandle &nh, const std::string &server_ip, int server_port) {
    try {
        boost::asio::io_service io_service;
        tcp::acceptor acceptor(io_service, tcp::endpoint(boost::asio::ip::address::from_string(server_ip), server_port));
        std::unordered_map<std::string, ros::Publisher> publishers;

        while (ros::ok()) {
            tcp::socket socket(io_service);
            acceptor.accept(socket);
            std::thread(handleClient, std::move(socket), std::ref(nh), std::ref(publishers)).detach();
        }
    }
    catch (std::exception &e) {
        ROS_ERROR("Exception: %s", e.what());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "server_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string server_ip;
    int server_port;

    private_nh.getParam("server_ip", server_ip);
    private_nh.getParam("server_port", server_port);

    std::thread(server, std::ref(nh), server_ip, server_port).detach();

    ros::spin();
    return 0;
}
