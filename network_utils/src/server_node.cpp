#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <sstream>
#include <vector>

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

    void handleClient(tcp::socket* socket) {
        try {
            for (;;) {
                boost::asio::streambuf buf;
                boost::asio::read_until(*socket, buf, '\n');
                std::istream is(&buf);
                std::string line;
                std::getline(is, line);

                std::istringstream iss(line);
                std::string token;
                std::vector<std::string> tokens;
                while (std::getline(iss, token, '|')) {
                    tokens.push_back(token);
                }

                if (tokens.size() != 12) {
                    ROS_ERROR("Invalid data format");
                    continue;
                }

                int drone_id = std::stoi(tokens[0]);
                unsigned int sec = std::stoul(tokens[1]);
                unsigned int nsec = std::stoul(tokens[2]);
                int type = std::stoi(tokens[3]);
                geometry_msgs::Point position;
                position.x = std::stod(tokens[4]);
                position.y = std::stod(tokens[5]);
                position.z = std::stod(tokens[6]);
                geometry_msgs::Quaternion orientation;
                orientation.x = std::stod(tokens[7]);
                orientation.y = std::stod(tokens[8]);
                orientation.z = std::stod(tokens[9]);
                orientation.w = std::stod(tokens[10]);

                ros::Time timestamp(sec, nsec);

                ROS_INFO("Received data from drone %d, timestamp: %u.%u, message type: %d", drone_id, sec, nsec, type);

                switch (type) {
                    case IMU_PROPAGATE: {
                        nav_msgs::Odometry msg;
                        msg.header.stamp = timestamp;
                        msg.pose.pose.position = position;
                        msg.pose.pose.orientation = orientation;
                        processAndPublish(drone_id, "/vins_fusion/imu_propagate", msg);
                        break;
                    }
                    case VINS_PATH: {
                        nav_msgs::Path msg;
                        geometry_msgs::PoseStamped pose_stamped;
                        pose_stamped.header.stamp = timestamp;
                        pose_stamped.pose.position = position;
                        pose_stamped.pose.orientation = orientation;
                        msg.poses.push_back(pose_stamped);
                        processAndPublish(drone_id, "/vins_fusion/path", msg);
                        break;
                    }
                    case GLOBAL_ODOMETRY: {
                        nav_msgs::Odometry msg;
                        msg.header.stamp = timestamp;
                        msg.pose.pose.position = position;
                        msg.pose.pose.orientation = orientation;
                        processAndPublish(drone_id, "/ranging_fusion/global_odometry", msg);
                        break;
                    }
                    case GLOBAL_PATH: {
                        nav_msgs::Path msg;
                        geometry_msgs::PoseStamped pose_stamped;
                        pose_stamped.header.stamp = timestamp;
                        pose_stamped.pose.position = position;
                        pose_stamped.pose.orientation = orientation;
                        msg.poses.push_back(pose_stamped);
                        processAndPublish(drone_id, "/ranging_fusion/global_path", msg);
                        break;
                    }
                    default:
                        ROS_ERROR("Unknown message type: %d", type);
                        break;
                }
            }
        } catch (std::exception& e) {
            ROS_ERROR("Client handling error: %s", e.what());
        } catch (...) {
            ROS_ERROR("Unknown error occurred while handling client");
        }
        delete socket;
    }

    template <typename T>
    void processAndPublish(int drone_id, const std::string& topic, const T& msg) {
        std::string prefix = "/drone_" + std::to_string(drone_id);
        ros::Publisher pub = nh.advertise<T>(prefix + topic, 10);
        pub.publish(msg);
    }

    ros::NodeHandle nh;
    boost::asio::io_service io_service;
    tcp::acceptor acceptor;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "server_node");
    ros::NodeHandle nh("~");

    std::string ip;
    int port;

    nh.param("server_ip", ip, std::string("127.0.0.1"));
    nh.param("server_port", port, 9000);

    ServerNode server_node(ip, port);
    server_node.run();

    return 0;
}