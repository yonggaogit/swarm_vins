#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <zmq.hpp>
#include <sstream>
#include <thread>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <string>
#include <stdexcept>

class PosePubSub {
public:
    PosePubSub(int drone_id, const std::string& pub_endpoint, const std::vector<std::string>& sub_endpoints,
               const std::unordered_map<std::string, std::vector<double>>& offsets, const std::string& odom_topic, const std::string& vins_topic)
        : drone_id_(drone_id), context_(1), publisher_(context_, ZMQ_PUB), subscriber_(context_, ZMQ_SUB), offsets_(offsets), vins_topic_(vins_topic) {
        ros::NodeHandle nh;
        odom_sub_ = nh.subscribe(odom_topic, 10, &PosePubSub::odomCallback, this);

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
                    std::cout << "origin posistion:" << px << ", " << py << ", " << pz << std::endl;
                    // 调整位姿数据
                    if (std::stoi(drone_id) != drone_id_ && offsets_.find(drone_id) != offsets_.end()) {
                        std::vector<double> offset = offsets_.at(drone_id);
                        px += offset[0];
                        py += offset[1];
                        pz += offset[2];
                    }
                    std::cout << "after adjust posistion:" << px << ", " << py << ", " << pz << std::endl;
                    // 动态生成话题名并发布消息
                    if (std::stoi(drone_id) != drone_id_) {
                        std::string topic_name = "/drone_" + drone_id + vins_topic_;
                        nav_msgs::Odometry odom_msg;
                        odom_msg.header.stamp = ros::Time(timestamp);
                        odom_msg.header.frame_id = "world";
                        odom_msg.child_frame_id = "world";
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
    int drone_id_;
    std::unordered_map<std::string, ros::Publisher> pose_publishers_;
    std::unordered_map<std::string, std::vector<double>> offsets_;
    std::string vins_topic_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_pub_sub");
    ros::NodeHandle nh;

    int drone_id, num_drones, base_port;
    double offset_multiplier;
    std::string pub_endpoint, odom_topic, vins_topic;
    std::string drone1_ip, drone2_ip, drone3_ip, drone4_ip, drone5_ip;
    std::vector<std::string> sub_endpoints;
    std::unordered_map<std::string, std::vector<double>> offsets;

    // 检查并获取参数
    if (!nh.getParam("drone_id", drone_id)) {
        ROS_ERROR("Missing parameter: drone_id");
        return -1;
    }
    if (!nh.getParam("num_drones", num_drones)) {
        ROS_ERROR("Missing parameter: num_drones");
        return -1;
    }
    if (!nh.getParam("drone1_ip", drone1_ip)) {
        ROS_ERROR("Missing parameter: drone1_ip");
        return -1;
    }
    if (!nh.getParam("drone2_ip", drone2_ip)) {
        ROS_ERROR("Missing parameter: drone2_ip");
        return -1;
    }
    if (!nh.getParam("drone3_ip", drone3_ip)) {
        ROS_ERROR("Missing parameter: drone3_ip");
        return -1;
    }
    if (!nh.getParam("drone4_ip", drone4_ip)) {
        ROS_ERROR("Missing parameter: drone4_ip");
        return -1;
    }
    if (!nh.getParam("drone5_ip", drone5_ip)) {
        ROS_ERROR("Missing parameter: drone5_ip");
        return -1;
    }
    if (!nh.getParam("base_port", base_port)) {
        ROS_ERROR("Missing parameter: base_port");
        return -1;
    }
    if (!nh.getParam("odom_topic", odom_topic)) {
        ROS_ERROR("Missing parameter: odom_topic");
        return -1;
    }
    if (!nh.getParam("vins_topic", vins_topic)) {
        ROS_ERROR("Missing parameter: vins_topic");
        return -1;
    }
    if (!nh.getParam("offset_multiplier", offset_multiplier)) {
        ROS_ERROR("Missing parameter: offset_multiplier");
        return -1;
    }

    // 打印所有参数用于调试
    ROS_INFO("Parameters:");
    ROS_INFO("  drone_id: %d", drone_id);
    ROS_INFO("  num_drones: %d", num_drones);
    ROS_INFO("  drone1_ip: %s", drone1_ip.c_str());
    ROS_INFO("  drone2_ip: %s", drone2_ip.c_str());
    ROS_INFO("  drone3_ip: %s", drone3_ip.c_str());
    ROS_INFO("  drone4_ip: %s", drone4_ip.c_str());
    ROS_INFO("  drone5_ip: %s", drone5_ip.c_str());
    ROS_INFO("  base_port: %d", base_port);
    ROS_INFO("  odom_topic: %s", odom_topic.c_str());
    ROS_INFO("  vins_topic: %s", vins_topic.c_str());
    ROS_INFO("  offset_multiplier: %f", offset_multiplier);

    // 处理IP地址和端点
    std::vector<std::string> drone_ips = {drone1_ip, drone2_ip, drone3_ip, drone4_ip, drone5_ip};

    for (int i = 0; i < num_drones; ++i) {
        if (i + 1 != drone_id && !drone_ips[i].empty()) {
            std::string endpoint = "tcp://" + drone_ips[i] + ":" + std::to_string(base_port);
            sub_endpoints.push_back(endpoint);
            std::vector<double> offset = {0.0, (i + 1 - drone_id) * offset_multiplier, 0.0};
            offsets[endpoint] = offset;
        }
    }

    pub_endpoint = "tcp://" + drone_ips[drone_id - 1] + ":" + std::to_string(base_port);

    try {
        PosePubSub pose_pub_sub(drone_id, pub_endpoint, sub_endpoints, offsets, odom_topic, vins_topic);
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
