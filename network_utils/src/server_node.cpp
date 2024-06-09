#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include "serialization_helpers.hpp"

using boost::asio::ip::tcp;

enum MessageType {
    IMU_PROPAGATE,
    VINS_PATH,
    GLOBAL_ODOMETRY,
    GLOBAL_PATH
};

struct DroneData {
    int drone_id;
    ros::Time timestamp;
    MessageType type;
    nav_msgs::Odometry odometry;
    nav_msgs::Path path;
};

class ServerNode {
public:
    ServerNode(const std::string& ip, int port) : io_service(), acceptor(io_service, tcp::endpoint(boost::asio::ip::address::from_string(ip), port)) {
        startAccept();
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
            boost::thread(boost::bind(&ServerNode::handleClient, this, new_socket)).detach();
        } else {
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
                boost::archive::binary_iarchive archive(is);
                int drone_id;
                ros::Time timestamp;
                MessageType type;
                archive >> drone_id >> timestamp >> type;

                switch (type) {
                    case IMU_PROPAGATE: {
                        nav_msgs::Odometry msg;
                        archive >> msg;
                        processAndPublish(drone_id, "/vins_fusion/imu_propagate", msg);
                        break;
                    }
                    case VINS_PATH: {
                        nav_msgs::Path msg;
                        archive >> msg;
                        processAndPublish(drone_id, "/vins_fusion/path", msg);
                        break;
                    }
                    case GLOBAL_ODOMETRY: {
                        nav_msgs::Odometry msg;
                        archive >> msg;
                        processAndPublish(drone_id, "/ranging_fusion/global_odometry", msg);
                        break;
                    }
                    case GLOBAL_PATH: {
                        nav_msgs::Path msg;
                        archive >> msg;
                        processAndPublish(drone_id, "/ranging_fusion/global_path", msg);
                        break;
                    }
                }
            }
        } catch (std::exception& e) {
            ROS_ERROR("Exception: %s", e.what());
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
