#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <zlib.h>

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
        sendPathData(VINS_PATH, *msg);
    }

    void globalOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::Point position = msg->pose.pose.position;
        modifyYCoordinate(position.y);
        sendData(GLOBAL_ODOMETRY, msg->header.stamp, position, msg->pose.pose.orientation);
    }

    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        nav_msgs::Path modified_msg = *msg;
        for (auto& pose : modified_msg.poses) {
            modifyYCoordinate(pose.pose.position.y);
        }
        sendPathData(GLOBAL_PATH, modified_msg);
    }

private:
    void modifyYCoordinate(double& y) {
        if (drone_id > 1) {
            y += (drone_id - 1) * offset_multiplier;
        }
    }

    std::string compressString(const std::string& str) {
        z_stream zs;
        memset(&zs, 0, sizeof(zs));

        if (deflateInit(&zs, Z_BEST_COMPRESSION) != Z_OK) {
            throw std::runtime_error("deflateInit failed while compressing.");
        }

        zs.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(str.data()));
        zs.avail_in = str.size();

        int ret;
        char outbuffer[32768];
        std::string outstring;

        do {
            zs.next_out = reinterpret_cast<Bytef*>(outbuffer);
            zs.avail_out = sizeof(outbuffer);

            ret = deflate(&zs, Z_FINISH);

            if (outstring.size() < zs.total_out) {
                outstring.append(outbuffer, zs.total_out - outstring.size());
            }
        } while (ret == Z_OK);

        deflateEnd(&zs);

        if (ret != Z_STREAM_END) {
            throw std::runtime_error("deflate failed while compressing.");
        }

        return outstring;
    }

    void sendData(MessageType type, const ros::Time& timestamp, const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation) {
        try {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(9)
                << drone_id << "|" << timestamp.sec << "|" << timestamp.nsec << "|" << type << "|"
                << position.x << "|" << position.y << "|" << position.z << "|"
                << orientation.x << "|" << orientation.y << "|" << orientation.z << "|" << orientation.w << "\n";
            std::string outbound_data = compressString(oss.str());
            
            uint32_t data_length = outbound_data.size();
            std::ostringstream length_stream;
            length_stream.write(reinterpret_cast<const char*>(&data_length), sizeof(data_length));
            std::string length_data = length_stream.str();

            std::vector<boost::asio::const_buffer> buffers;
            buffers.push_back(boost::asio::buffer(length_data));
            buffers.push_back(boost::asio::buffer(outbound_data));

            ROS_INFO("Sending compressed data");
            boost::asio::write(socket, buffers);
            ROS_INFO("Sent compressed data of type %d", type);
        } catch (boost::system::system_error& e) {
            ROS_ERROR("Failed to send data: %s", e.what());
        }
    }

    void sendPathData(MessageType type, const nav_msgs::Path& path_msg) {
        try {
            std::ostringstream oss;
            oss << drone_id << "|" << path_msg.header.stamp.sec << "|" << path_msg.header.stamp.nsec << "|" << type;

            for (const auto& pose : path_msg.poses) {
                oss << "|" << pose.header.stamp.sec << "|" << pose.header.stamp.nsec
                    << "|" << pose.pose.position.x << "|" << pose.pose.position.y << "|" << pose.pose.position.z
                    << "|" << pose.pose.orientation.x << "|" << pose.pose.orientation.y << "|" << pose.pose.orientation.z << "|" << pose.pose.orientation.w;
            }
            oss << "\n";

            std::string outbound_data = compressString(oss.str());
            
            uint32_t data_length = outbound_data.size();
            std::ostringstream length_stream;
            length_stream.write(reinterpret_cast<const char*>(&data_length), sizeof(data_length));
            std::string length_data = length_stream.str();

            std::vector<boost::asio::const_buffer> buffers;
            buffers.push_back(boost::asio::buffer(length_data));
            buffers.push_back(boost::asio::buffer(outbound_data));

            ROS_INFO("Sending compressed path data");
            boost::asio::write(socket, buffers);
            ROS_INFO("Sent compressed path data of type %d", type);
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