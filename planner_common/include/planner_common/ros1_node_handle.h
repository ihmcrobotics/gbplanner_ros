#ifndef PLANNER_COMMON_ROS1_NODE_HANDLE_H_
#define PLANNER_COMMON_ROS1_NODE_HANDLE_H_

// The parts of ROS 1's NodeHandle GBPlanner uses (advertise, subscribe, advertiseService,
// serviceClient, createTimer), on an rclcpp node. Keeps the planner's node code close to upstream.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ros2_compat {

/// A ros::Publisher: one handle for any message type, picked at advertise().
class Publisher {
 public:
  Publisher() = default;
  explicit Publisher(std::shared_ptr<rclcpp::PublisherBase> publisher) : publisher_(std::move(publisher)) {}

  template <typename MessageType>
  void publish(const MessageType& message) const {
    auto typed = std::dynamic_pointer_cast<rclcpp::Publisher<MessageType>>(publisher_);
    if (!typed) {
      RCLCPP_ERROR(rclcpp::get_logger("gbplanner"), "Publishing the wrong message type on %s",
                   publisher_ ? publisher_->get_topic_name() : "an unadvertised publisher");
      return;
    }
    typed->publish(message);
  }

  /// pcl_ros let ROS 1 publish PCL clouds directly
  template <typename PointType>
  void publish(const pcl::PointCloud<PointType>& cloud) const {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    publish(message);
  }

  template <typename PointType>
  void publish(const typename pcl::PointCloud<PointType>::Ptr& cloud) const {
    publish(*cloud);
  }

  uint32_t getNumSubscribers() const {
    return publisher_ ? static_cast<uint32_t>(publisher_->get_subscription_count()) : 0;
  }

  std::string getTopic() const { return publisher_ ? publisher_->get_topic_name() : ""; }

 private:
  std::shared_ptr<rclcpp::PublisherBase> publisher_;
};

using Subscriber = std::shared_ptr<rclcpp::SubscriptionBase>;
using ServiceServer = std::shared_ptr<rclcpp::ServiceBase>;

/// ros::TimerEvent. Nothing in GBPlanner reads its fields.
struct TimerEvent {};

/// A ros::Timer that can be stopped and started again.
class Timer {
 public:
  Timer() = default;
  explicit Timer(rclcpp::TimerBase::SharedPtr timer) : timer_(std::move(timer)) {}
  void stop() {
    if (timer_) timer_->cancel();
  }
  void start() {
    if (timer_) timer_->reset();
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
};

/// A ros::ServiceClient for service type ServiceType.
template <typename ServiceType>
class ServiceClient {
 public:
  ServiceClient() = default;
  explicit ServiceClient(typename rclcpp::Client<ServiceType>::SharedPtr client) : client_(std::move(client)) {}

  /// Blocks up to timeout for the response. From a callback, this needs a multi-threaded executor.
  bool call(const typename ServiceType::Request& request, typename ServiceType::Response& response,
            std::chrono::duration<double> timeout = std::chrono::seconds(5)) {
    if (!client_ || !client_->wait_for_service(std::chrono::milliseconds(100))) return false;
    auto future = client_->async_send_request(std::make_shared<typename ServiceType::Request>(request));
    if (future.wait_for(timeout) != std::future_status::ready) return false;
    response = *future.get();
    return true;
  }

  bool exists() const { return client_ && client_->service_is_ready(); }

 private:
  typename rclcpp::Client<ServiceType>::SharedPtr client_;
};

/// A ros::NodeHandle: topic and service names resolve in the node's namespace, or in the node's
/// private namespace for a handle made with private_namespace = true ("~").
class NodeHandle {
 public:
  NodeHandle() = default;
  explicit NodeHandle(rclcpp::Node::SharedPtr node, bool private_namespace = false)
      : node_(std::move(node)), private_namespace_(private_namespace) {
    // Subscriptions, services and timers share the node's default group, so they run one at a time
    // as under ros::spin(). Clients get their own group so a callback can wait for a response on a
    // multi-threaded executor.
    if (node_) client_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }

  rclcpp::Node::SharedPtr node() const { return node_; }

  template <typename MessageType>
  Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false) {
    rclcpp::QoS qos(std::max<uint32_t>(queue_size, 1));
    if (latch) qos.transient_local();
    return Publisher(node_->create_publisher<MessageType>(name(topic), qos));
  }

  template <typename Class, typename MessageType>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                       void (Class::*callback)(const MessageType&), Class* object) {
    return node_->create_subscription<MessageType>(
        name(topic), rclcpp::QoS(std::max<uint32_t>(queue_size, 1)),
        [object, callback](const typename MessageType::ConstSharedPtr message) { (object->*callback)(*message); });
  }

  template <typename Class, typename MessageType>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                       void (Class::*callback)(const std::shared_ptr<const MessageType>&), Class* object) {
    return node_->create_subscription<MessageType>(
        name(topic), rclcpp::QoS(std::max<uint32_t>(queue_size, 1)),
        [object, callback](const typename MessageType::ConstSharedPtr message) { (object->*callback)(message); });
  }

  template <typename ServiceType, typename Class>
  ServiceServer advertiseService(const std::string& service,
                                 bool (Class::*callback)(typename ServiceType::Request&,
                                                         typename ServiceType::Response&),
                                 Class* object) {
    return node_->create_service<ServiceType>(
        name(service),
        [object, callback](const std::shared_ptr<typename ServiceType::Request> request,
                           std::shared_ptr<typename ServiceType::Response> response) {
          (object->*callback)(*request, *response);
        });
  }

  template <typename ServiceType>
  ServiceClient<ServiceType> serviceClient(const std::string& service) {
    return ServiceClient<ServiceType>(
        node_->create_client<ServiceType>(name(service), rclcpp::ServicesQoS(), client_group_));
  }

  template <typename Class>
  Timer createTimer(const rclcpp::Duration& period, void (Class::*callback)(const TimerEvent&), Class* object,
                    bool oneshot = false, bool autostart = true) {
    auto timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
    *timer = node_->create_timer(
        std::chrono::nanoseconds(period.nanoseconds()),
        [object, callback, oneshot, timer]() {
          if (oneshot && *timer) (*timer)->cancel();
          (object->*callback)(TimerEvent());
        });
    if (!autostart) (*timer)->cancel();
    return Timer(*timer);
  }

 private:
  std::string name(const std::string& topic) const {
    if (!private_namespace_ || topic.empty() || topic.front() == '/' || topic.front() == '~') return topic;
    return "~/" + topic;
  }

  rclcpp::Node::SharedPtr node_;
  bool private_namespace_ = false;
  rclcpp::CallbackGroup::SharedPtr client_group_;
};

}  // namespace ros2_compat

#endif  // PLANNER_COMMON_ROS1_NODE_HANDLE_H_
