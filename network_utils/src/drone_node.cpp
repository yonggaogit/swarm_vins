#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>

using boost::asio::ip::tcp;

enum MessageType {
    IMU_PROPAGATE,
    VINS_PATH,
    GLOBAL_ODOMETRY,
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
        sendData(IMU_PROPAGATE, msg->header.stamp, msg->pose.pose.position, msg->pose.pose.orientation);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        for (const auto& pose : msg->poses) {
            sendData(VINS_PATH, pose.header.stamp, pose.pose.position, pose.pose.orientation);
        }
    }

    void globalOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::Point position = msg->pose.pose.position;
        modifyYCoordinate(position.y);
        sendData(GLOBAL_ODOMETRY, msg->header.stamp, position, msg->pose.pose.orientation);
    }

    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        for (auto& pose : msg->poses) {
            geometry_msgs::Point position = pose.pose.position;
            modifyYCoordinate(position.y);
            sendData(GLOBAL_PATH, pose.header.stamp, position, pose.pose.orientation);
        }
    }

private:
    void modifyYCoordinate(double& y) {
        if (drone_id > 1) {
            y += (drone_id - 1) * offset_multiplier;
        }
    }

    void sendData(MessageType type, const ros::Time& timestamp, const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation) {
        try {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(9)
                << drone_id << "|" << timestamp.sec << "|" << timestamp.nsec << "|" << type << "|"
                << position.x << "|" << position.y << "|" << position.z << "|"
                << orientation.x << "|" << orientation.y << "|" << orientation.z << "|" << orientation.w << "\n";
            std::string outbound_data = oss.str();
            ROS_INFO("Sending data: %s", outbound_data.c_str());  // Add log for debugging
            boost::asio::write(socket, boost::asio::buffer(outbound_data));
            ROS_INFO("Sent data of type %d", type);
        } catch (boost::system::system_error& e) {
            ROS_ERROR("Failed to send data: %s", e.what());
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