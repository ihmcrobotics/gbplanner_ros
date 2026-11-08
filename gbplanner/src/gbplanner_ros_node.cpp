#include <gflags/gflags.h>
#include <glog/logging.h>

#include "planner_common/ros2_compat.h"
#include "gbplanner/gbplanner_ros.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  rclcpp::init(argc, argv);
  const auto ros_arguments = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> remaining;
  for (const auto& argument : ros_arguments) remaining.push_back(const_cast<char*>(argument.c_str()));
  int remaining_count = static_cast<int>(remaining.size());
  char** remaining_arguments = remaining.data();
  google::ParseCommandLineFlags(&remaining_count, &remaining_arguments, false);

  // The planner reads its nested YAML (PlanningParams.type, ...) without declaring each parameter,
  // as ROS 1 did
  auto node = std::make_shared<rclcpp::Node>(
      "gbplanner_node", rclcpp::NodeOptions()
                            .allow_undeclared_parameters(true)
                            .automatically_declare_parameters_from_overrides(true));
  ros2_compat::setNode(node);

  ros2_compat::NodeHandle nh(node);
  ros2_compat::NodeHandle nh_private(node, true);
  GbplannerRos planner(nh, nh_private);

  // A second thread answers service clients while a planning callback waits on them
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  ros2_compat::setNode(nullptr);
  rclcpp::shutdown();
  return 0;
}
