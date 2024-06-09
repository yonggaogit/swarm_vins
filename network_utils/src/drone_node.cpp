#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <string>
#include <sstream>

using boost::asio::ip::tcp;

int drone_id;
std::string server_ip;
int server_port;
double offset_multiplier;

void sendToServer(const std::string &data) {
    try {
        boost::asio::io_service io_service;
        tcp::socket socket(io_service);
        tcp::resolver resolver(io_service);
        tcp::resolver::query query(server_ip, std::to_string(server_port));
        boost::asio::connect(socket, resolver.resolve(query));

        boost::system::error_code ignored_error;
        boost::asio::write(socket, boost::asio::buffer(data), ignored_error);
    }
    catch (std::exception &e) {
        ROS_ERROR("Exception: %s", e.what());
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, const std::string &topic_name) {
    boost::property_tree::ptree pt;
    pt.put("drone_id", drone_id);
    pt.put("topic_name", topic_name);
    pt.put("current_time", ros::Time::now().toNSec());
    pt.put("msg.header.stamp", msg->header.stamp.toNSec());
    pt.put("msg.pose.pose.position.x", msg->pose.pose.position.x);
    pt.put("msg.pose.pose.position.y", msg->pose.pose.position.y + (drone_id - 1) * offset_multiplier);
    pt.put("msg.pose.pose.position.z", msg->pose.pose.position.z);
    pt.put("msg.pose.pose.orientation.x", msg->pose.pose.orientation.x);
    pt.put("msg.pose.pose.orientation.y", msg->pose.pose.orientation.y);
    pt.put("msg.pose.pose.orientation.z", msg->pose.pose.orientation.z);
    pt.put("msg.pose.pose.orientation.w", msg->pose.pose.orientation.w);

    std::ostringstream oss;
    boost::property_tree::write_json(oss, pt);
    sendToServer(oss.str());
}

void pathCallback(const nav_msgs::Path::ConstPtr &msg, const std::string &topic_name) {
    boost::property_tree::ptree pt;
    pt.put("drone_id", drone_id);
    pt.put("topic_name", topic_name);
    pt.put("current_time", ros::Time::now().toNSec());
    pt.put("msg.header.stamp", msg->header.stamp.toNSec());

    boost::property_tree::ptree poses;
    for (const auto &pose : msg->poses) {
        boost::property_tree::ptree pose_pt;
        pose_pt.put("position.x", pose.pose.position.x);
        pose_pt.put("position.y", pose.pose.position.y + (drone_id - 1) * offset_multiplier);
        pose_pt.put("position.z", pose.pose.position.z);
        pose_pt.put("orientation.x", pose.pose.orientation.x);
        pose_pt.put("orientation.y", pose.pose.orientation.y);
        pose_pt.put("orientation.z", pose.pose.orientation.z);
        pose_pt.put("orientation.w", pose.pose.orientation.w);
        poses.push_back(std::make_pair("", pose_pt));
    }
    pt.add_child("msg.poses", poses);

    std::ostringstream oss;
    boost::property_tree::write_json(oss, pt);
    sendToServer(oss.str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("drone_id", drone_id);
    private_nh.getParam("server_ip", server_ip);
    private_nh.getParam("server_port", server_port);
    private_nh.getParam("offset_multiplier", offset_multiplier);

    ros::Subscriber sub1 = nh.subscribe<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 10, boost::bind(odomCallback, _1, "imu_propagate"));
    ros::Subscriber sub2 = nh.subscribe<nav_msgs::Odometry>("/ranging_fusion/global_odometry", 10, boost::bind(odomCallback, _1, "global_odometry"));
    ros::Subscriber sub3 = nh.subscribe<nav_msgs::Path>("/ranging_fusion/global_path", 10, boost::bind(pathCallback, _1, "global_path"));

    ros::spin();
    return 0;
}
