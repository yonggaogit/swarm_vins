#ifndef SERIALIZATION_HELPERS_HPP
#define SERIALIZATION_HELPERS_HPP

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive& ar, geometry_msgs::Point& point, const unsigned int version) {
    ar & point.x;
    ar & point.y;
    ar & point.z;
}

template<class Archive>
void serialize(Archive& ar, geometry_msgs::Quaternion& quat, const unsigned int version) {
    ar & quat.x;
    ar & quat.y;
    ar & quat.z;
    ar & quat.w;
}

template<class Archive>
void serialize(Archive& ar, geometry_msgs::Pose& pose, const unsigned int version) {
    ar & pose.position;
    ar & pose.orientation;
}

template<class Archive>
void serialize(Archive& ar, geometry_msgs::PoseStamped& pose_stamped, const unsigned int version) {
    ar & pose_stamped.header.stamp;
    ar & pose_stamped.header.frame_id;
    ar & pose_stamped.pose;
}

template<class Archive>
void serialize(Archive& ar, geometry_msgs::Twist& twist, const unsigned int version) {
    ar & twist.linear.x;
    ar & twist.linear.y;
    ar & twist.linear.z;
    ar & twist.angular.x;
    ar & twist.angular.y;
    ar & twist.angular.z;
}

template<class Archive>
void serialize(Archive& ar, nav_msgs::Odometry& odom, const unsigned int version) {
    ar & odom.header.stamp;
    ar & odom.header.frame_id;
    ar & odom.child_frame_id;
    ar & odom.pose.pose;
    ar & odom.twist.twist;
}

template<class Archive>
void serialize(Archive& ar, nav_msgs::Path& path, const unsigned int version) {
    ar & path.header.stamp;
    ar & path.header.frame_id;
    ar & path.poses;
}

template<class Archive>
void serialize(Archive& ar, ros::Time& time, const unsigned int version) {
    ar & time.sec;
    ar & time.nsec;
}

} // namespace serialization
} // namespace boost

#endif // SERIALIZATION_HELPERS_HPP
