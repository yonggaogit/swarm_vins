#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include "serialization_helpers.hpp"

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
        sendData(IMU_PROPAGATE, *msg);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        sendData(VINS_PATH, *msg);
    }

    void globalOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        nav_msgs::Odometry modified_msg = *msg;
        modifyYCoordinate(modified_msg.pose.pose.position.y);
        sendData(GLOBAL_ODOMETRY, modified_msg);
    }

    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        nav_msgs::Path modified_msg = *msg;
        for (auto& pose : modified_msg.poses) {
            modifyYCoordinate(pose.pose.position.y);
        }
        sendData(GLOBAL_PATH, modified_msg);
    }

private:
    void modifyYCoordinate(double& y) {
        if (drone_id > 1) {
            y += (drone_id - 1) * offset_multiplier;
        }
    }

    void sendData(MessageType type, const nav_msgs::Odometry& msg) {
        try {
            std::ostringstream archive_stream;
            boost::archive::binary_oarchive archive(archive_stream);
            archive << drone_id << ros::Time::now() << type << msg;
            std::string outbound_data = archive_stream.str();
            boost::asio::write(socket, boost::asio::buffer(outbound_data));
            ROS_INFO("Sent Odometry data of type %d", type);
        } catch (boost::system::system_error& e) {
            ROS_ERROR("Failed to send data: %s", e.what());
        }
    }

    void sendData(MessageType type, const nav_msgs::Path& msg) {
        try {
            std::ostringstream archive_stream;
            boost::archive::binary_oarchive archive(archive_stream);
            archive << drone_id << ros::Time::now() << type << msg;
            std::string outbound_data = archive_stream.str();
            boost::asio::write(socket, boost::asio::buffer(outbound_data));
            ROS_INFO("Sent Path data of type %d", type);
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