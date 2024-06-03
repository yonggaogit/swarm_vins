// 文件路径: src/pose_pub_sub.cpp

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <zmq.hpp>
#include <sstream>
#include <thread>
#include <vector>
#include <chrono>
#include <unordered_map>

class PosePubSub {
public:
    PosePubSub(const std::string& drone_id, const std::string& pub_endpoint, const std::vector<std::string>& sub_endpoints)
        : drone_id_(drone_id), context_(1), publisher_(context_, ZMQ_PUB), subscriber_(context_, ZMQ_SUB) {
        ros::NodeHandle nh;
        odom_sub_ = nh.subscribe("odom", 10, &PosePubSub::odomCallback, this);

        // 设置发布端点
        try {
            publisher_.bind(pub_endpoint);
            ROS_INFO("Bound publisher to %s", pub_endpoint.c_str());
        } catch (const zmq::error_t& e) {
            ROS_ERROR("Failed to bind publisher to %s: %s", pub_endpoint.c_str(), e.what());
            throw;
        }

        // 设置订阅端点
        for (const auto& endpoint : sub_endpoints) {
            try {
                subscriber_.connect(endpoint);
                ROS_INFO("Connected subscriber to %s", endpoint.c_str());
            } catch (const zmq::error_t& e) {
                ROS_ERROR("Failed to connect subscriber to %s: %s", endpoint.c_str(), e.what());
                throw;
            }
        }

        // 设置订阅选项
        subscriber_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    }

    void start() {
        std::thread([this]() {
            while (ros::ok()) {
                try {
                    zmq::message_t message;
                    subscriber_.recv(&message);
                    std::string data(static_cast<char*>(message.data()), message.size());

                    std::istringstream iss(data);
                    std::string drone_id;
                    double timestamp, px, py, pz, ox, oy, oz, ow;
                    iss >> drone_id >> timestamp >> px >> py >> pz >> ox >> oy >> oz >> ow;

                    ROS_INFO("Received pose from %s at time %f: [%f, %f, %f, %f, %f, %f, %f]",
                             drone_id.c_str(), timestamp, px, py, pz, ox, oy, oz, ow);

                    // 动态生成话题名并发布消息
                    if (drone_id != drone_id_) {
                        std::string topic_name = "/drone_" + drone_id + "/vins_estimator/odometry";
                        nav_msgs::Odometry odom_msg;
                        odom_msg.header.stamp = ros::Time(timestamp);
                        odom_msg.header.frame_id = "odom";
                        odom_msg.child_frame_id = "base_link";
                        odom_msg.pose.pose.position.x = px;
                        odom_msg.pose.pose.position.y = py;
                        odom_msg.pose.pose.position.z = pz;
                        odom_msg.pose.pose.orientation.x = ox;
                        odom_msg.pose.pose.orientation.y = oy;
                        odom_msg.pose.pose.orientation.z = oz;
                        odom_msg.pose.pose.orientation.w = ow;

                        if (pose_publishers_.find(drone_id) == pose_publishers_.end()) {
                            ros::NodeHandle nh;
                            pose_publishers_[drone_id] = nh.advertise<nav_msgs::Odometry>(topic_name, 10);
                        }
                        pose_publishers_[drone_id].publish(odom_msg);
                    }
                } catch (const zmq::error_t& e) {
                    ROS_ERROR("Error receiving message: %s", e.what());
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }).detach();
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::ostringstream oss;
        oss << drone_id_ << " "
            << msg->header.stamp.toSec() << " "
            << msg->pose.pose.position.x << " "
            << msg->pose.pose.position.y << " "
            << msg->pose.pose.position.z << " "
            << msg->pose.pose.orientation.x << " "
            << msg->pose.pose.orientation.y << " "
            << msg->pose.pose.orientation.z << " "
            << msg->pose.pose.orientation.w;

        std::string data = oss.str();
        zmq::message_t message(data.size());
        memcpy(message.data(), data.c_str(), data.size());

        try {
            publisher_.send(message, zmq::send_flags::none);
        } catch (const zmq::error_t& e) {
            ROS_ERROR("Error sending message: %s", e.what());
        }
    }

    ros::Subscriber odom_sub_;
    zmq::context_t context_;
    zmq::socket_t publisher_;
    zmq::socket_t subscriber_;
    std::string drone_id_;
    std::unordered_map<std::string, ros::Publisher> pose_publishers_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_pub_sub");
    ros::NodeHandle nh;

    std::string drone_id, pub_endpoint, sub_endpoints_str;
    std::vector<std::string> sub_endpoints;

    if (!nh.getParam("drone_id", drone_id) ||
        !nh.getParam("pub_endpoint", pub_endpoint) ||
        !nh.getParam("sub_endpoints", sub_endpoints_str)) {
        ROS_ERROR("Missing parameters: drone_id, pub_endpoint, or sub_endpoints");
        return -1;
    }

    std::istringstream iss(sub_endpoints_str);
    std::string endpoint;
    while (std::getline(iss, endpoint, ',')) {
        sub_endpoints.push_back(endpoint);
    }

    try {
        PosePubSub pose_pub_sub(drone_id, pub_endpoint, sub_endpoints);
        pose_pub_sub.start();
        ros::spin();
    } catch (const zmq::error_t& e) {
        ROS_ERROR("ZeroMQ error: %s", e.what());
        return -1;
    } catch (const std::exception& e) {
        ROS_ERROR("Standard exception: %s", e.what());
        return -1;
    } catch (...) {
        ROS_ERROR("Unknown exception");
        return -1;
    }

    return 0;
}
