#ifndef PLANNER_COMMON_ROS2_COMPAT_H_
#define PLANNER_COMMON_ROS2_COMPAT_H_

// The ROS 1 calls GBPlanner's planning code makes, on ROS 2. Keeping their names keeps this
// fork's planning code close to upstream, so upstream changes still merge.

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

namespace ros2_compat {

/// The planner's node. Set once in main, before anything plans or reads parameters.
void setNode(const rclcpp::Node::SharedPtr& node);
rclcpp::Node::SharedPtr node();
rclcpp::Logger logger();
/// The node's clock: ROS time, or simulation time with use_sim_time.
rclcpp::Time now();
/// "/<node name>", what ros::this_node::getName() returned.
std::string nodeName();

/// ros::param::get. Names are ROS 1 paths such as "/gbplanner_node/PlanningParams/type"; the
/// node's own name is dropped and "/" becomes ".", so this reads PlanningParams.type. Like ROS 1,
/// it returns false and leaves the value alone when the parameter is missing or the wrong type.
bool getParam(const std::string& name, std::string& value);
bool getParam(const std::string& name, double& value);
bool getParam(const std::string& name, float& value);
bool getParam(const std::string& name, int& value);
bool getParam(const std::string& name, bool& value);
bool getParam(const std::string& name, std::vector<double>& value);
bool getParam(const std::string& name, std::vector<float>& value);
bool getParam(const std::string& name, std::vector<int>& value);
bool getParam(const std::string& name, std::vector<std::string>& value);
bool getParam(const std::string& name, std::vector<bool>& value);
bool hasParam(const std::string& name);

inline double toSec(const rclcpp::Time& time) { return time.seconds(); }
inline double toSec(const rclcpp::Duration& duration) { return duration.seconds(); }
inline rclcpp::Duration duration(double seconds) { return rclcpp::Duration::from_seconds(seconds); }
inline void sleep(double seconds) {
  rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(seconds * 1e9)));
}

}  // namespace ros2_compat

/// The tf (ROS 1) math GBPlanner uses, on tf2's LinearMath types.
namespace tf_compat {

using Quaternion = tf2::Quaternion;
using Vector3 = tf2::Vector3;
using Point = tf2::Vector3;
using Matrix3x3 = tf2::Matrix3x3;
using Pose = tf2::Transform;
using Transform = tf2::Transform;

inline double getYaw(const tf2::Quaternion& q) { return tf2::getYaw(q); }
inline double getYaw(const geometry_msgs::msg::Quaternion& q) {
  return tf2::getYaw(tf2::Quaternion(q.x, q.y, q.z, q.w));
}
inline void quaternionTFToMsg(const tf2::Quaternion& q, geometry_msgs::msg::Quaternion& msg) {
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
}
inline void quaternionMsgToTF(const geometry_msgs::msg::Quaternion& msg, tf2::Quaternion& q) {
  q = tf2::Quaternion(msg.x, msg.y, msg.z, msg.w);
}
inline void pointTFToMsg(const tf2::Vector3& v, geometry_msgs::msg::Point& msg) {
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
}
inline void pointMsgToTF(const geometry_msgs::msg::Point& msg, tf2::Vector3& v) {
  v = tf2::Vector3(msg.x, msg.y, msg.z);
}
inline void vector3TFToMsg(const tf2::Vector3& v, geometry_msgs::msg::Vector3& msg) {
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
}
inline void poseTFToMsg(const tf2::Transform& pose, geometry_msgs::msg::Pose& msg) {
  pointTFToMsg(pose.getOrigin(), msg.position);
  quaternionTFToMsg(pose.getRotation(), msg.orientation);
}
inline void poseMsgToTF(const geometry_msgs::msg::Pose& msg, tf2::Transform& pose) {
  tf2::Vector3 origin;
  tf2::Quaternion rotation;
  pointMsgToTF(msg.position, origin);
  quaternionMsgToTF(msg.orientation, rotation);
  pose = tf2::Transform(rotation, origin);
}
inline geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion msg;
  quaternionTFToMsg(q, msg);
  return msg;
}
inline tf2::Quaternion createQuaternionFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return q;
}
inline tf2::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return q;
}

}  // namespace tf_compat

#define ROS_DEBUG(...) RCLCPP_DEBUG(ros2_compat::logger(), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(ros2_compat::logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(ros2_compat::logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(ros2_compat::logger(), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(ros2_compat::logger(), __VA_ARGS__)
#define ROS_DEBUG_COND(cond, ...) RCLCPP_DEBUG_EXPRESSION(ros2_compat::logger(), (cond), __VA_ARGS__)
#define ROS_INFO_COND(cond, ...) RCLCPP_INFO_EXPRESSION(ros2_compat::logger(), (cond), __VA_ARGS__)
#define ROS_WARN_COND(cond, ...) RCLCPP_WARN_EXPRESSION(ros2_compat::logger(), (cond), __VA_ARGS__)
#define ROS_ERROR_COND(cond, ...) RCLCPP_ERROR_EXPRESSION(ros2_compat::logger(), (cond), __VA_ARGS__)
#define ROS_INFO_STREAM(stream) RCLCPP_INFO_STREAM(ros2_compat::logger(), stream)
#define ROS_WARN_STREAM(stream) RCLCPP_WARN_STREAM(ros2_compat::logger(), stream)
#define ROS_ERROR_STREAM(stream) RCLCPP_ERROR_STREAM(ros2_compat::logger(), stream)
#define ROS_INFO_THROTTLE(period, ...) \
  RCLCPP_INFO_THROTTLE(ros2_compat::logger(), *ros2_compat::node()->get_clock(), (period) * 1000, __VA_ARGS__)
#define ROS_WARN_THROTTLE(period, ...) \
  RCLCPP_WARN_THROTTLE(ros2_compat::logger(), *ros2_compat::node()->get_clock(), (period) * 1000, __VA_ARGS__)
#define ROS_ERROR_THROTTLE(period, ...) \
  RCLCPP_ERROR_THROTTLE(ros2_compat::logger(), *ros2_compat::node()->get_clock(), (period) * 1000, __VA_ARGS__)
#define ROS_INFO_ONCE(...) RCLCPP_INFO_ONCE(ros2_compat::logger(), __VA_ARGS__)
#define ROS_WARN_ONCE(...) RCLCPP_WARN_ONCE(ros2_compat::logger(), __VA_ARGS__)

#endif  // PLANNER_COMMON_ROS2_COMPAT_H_
