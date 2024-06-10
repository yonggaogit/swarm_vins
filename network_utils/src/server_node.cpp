#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <zlib.h>

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
                uint32_t data_length = 0;
                boost::asio::read(*socket, boost::asio::buffer(&data_length, sizeof(data_length)));

                std::vector<char> data(data_length);
                boost::asio::read(*socket, boost::asio::buffer(data.data(), data_length));

                std::string compressed_data(data.begin(), data.end());
                std::string line = decompressString(compressed_data);

                std::istringstream iss(line);
                std::string token;
                std::vector<std::string> tokens;
                while (std::getline(iss, token, '|')) {
                    tokens.push_back(token);
                }

                int type = std::stoi(tokens[3]);

                if (type == VINS_PATH || type == GLOBAL_PATH) {
                    handlePathData(tokens);
                } else {
                    handleData(tokens);
                }
            }
        } catch (std::exception& e) {
            ROS_ERROR("Client handling error: %s", e.what());
        } catch (...) {
            ROS_ERROR("Unknown error occurred while handling client");
        }
        delete socket;
    }

    void handleData(const std::vector<std::string>& tokens) {
        if (tokens.size() != 11) {
            ROS_ERROR("Invalid data format: %s", joinTokens(tokens).c_str());
            return;
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
            case GLOBAL_ODOMETRY: {
                nav_msgs::Odometry msg;
                msg.header.stamp = timestamp;
                msg.pose.pose.position = position;
                msg.pose.pose.orientation = orientation;
                processAndPublish(drone_id, "/ranging_fusion/global_odometry", msg);
                break;
            }
            default:
                ROS_ERROR("Unknown message type: %d", type);
                break;
        }
    }

    void handlePathData(const std::vector<std::string>& tokens) {
        int drone_id = std::stoi(tokens[0]);
        unsigned int sec = std::stoul(tokens[1]);
        unsigned int nsec = std::stoul(tokens[2]);
        int type = std::stoi(tokens[3]);

        ros::Time timestamp(sec, nsec);

        nav_msgs::Path msg;
        msg.header.stamp = timestamp;

        for (size_t i = 4; i < tokens.size(); i += 11) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp.sec = std::stoul(tokens[i]);
            pose_stamped.header.stamp.nsec = std::stoul(tokens[i + 1]);
            pose_stamped.pose.position.x = std::stod(tokens[i + 2]);
            pose_stamped.pose.position.y = std::stod(tokens[i + 3]);
            pose_stamped.pose.position.z = std::stod(tokens[i + 4]);
            pose_stamped.pose.orientation.x = std::stod(tokens[i + 5]);
            pose_stamped.pose.orientation.y = std::stod(tokens[i + 6]);
            pose_stamped.pose.orientation.z = std::stod(tokens[i + 7]);
            pose_stamped.pose.orientation.w = std::stod(tokens[i + 8]);
            msg.poses.push_back(pose_stamped);
        }

        ROS_INFO("Received path data from drone %d, message type: %d", drone_id, type);

        switch (type) {
            case VINS_PATH:
                processAndPublish(drone_id, "/vins_fusion/path", msg);
                break;
            case GLOBAL_PATH:
                processAndPublish(drone_id, "/ranging_fusion/global_path", msg);
                break;
            default:
                ROS_ERROR("Unknown message type: %d", type);
                break;
        }
    }

    std::string joinTokens(const std::vector<std::string>& tokens) {
        std::ostringstream oss;
        for (const auto& token : tokens) {
            oss << token << "|";
        }
        std::string result = oss.str();
        if (!result.empty()) {
            result.pop_back();  // Remove the last '|'
        }
        return result;
    }

    std::string decompressString(const std::string& str) {
        z_stream zs;
        memset(&zs, 0, sizeof(zs));

        if (inflateInit(&zs) != Z_OK) {
            throw std::runtime_error("inflateInit failed while decompressing.");
        }

        zs.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(str.data()));
        zs.avail_in = str.size();

        int ret;
        char outbuffer[32768];
        std::string outstring;

        do {
            zs.next_out = reinterpret_cast<Bytef*>(outbuffer);
            zs.avail_out = sizeof(outbuffer);

            ret = inflate(&zs, 0);

            if (outstring.size() < zs.total_out) {
                outstring.append(outbuffer, zs.total_out - outstring.size());
            }
        } while (ret == Z_OK);

        inflateEnd(&zs);

        if (ret != Z_STREAM_END) {
            throw std::runtime_error("inflate failed while decompressing.");
        }

        return outstring;
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