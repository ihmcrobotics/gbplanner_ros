#include "planner_common/ros2_compat.h"

#include <sstream>

namespace ros2_compat {

namespace {

rclcpp::Node::SharedPtr& storedNode() {
  static rclcpp::Node::SharedPtr node;
  return node;
}

/// "/gbplanner_node/PlanningParams/type" or "~PlanningParams/type" -> "PlanningParams.type".
std::string parameterName(const std::string& name) {
  std::vector<std::string> parts;
  std::stringstream stream(name);
  std::string part;
  while (std::getline(stream, part, '/')) {
    if (!part.empty()) parts.push_back(part);
  }
  if (!parts.empty() && !parts.front().empty() && parts.front().front() == '~')
    parts.front().erase(0, 1);
  auto current = storedNode();
  if (current && parts.size() > 1 && parts.front() == current->get_name())
    parts.erase(parts.begin());
  std::string result;
  for (const auto& p : parts) result += (result.empty() ? "" : ".") + p;
  return result;
}

bool lookup(const std::string& name, rclcpp::Parameter& parameter) {
  auto current = storedNode();
  if (!current) return false;
  return current->get_parameter(parameterName(name), parameter) &&
         parameter.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
}

}  // namespace

void setNode(const rclcpp::Node::SharedPtr& node) { storedNode() = node; }

rclcpp::Node::SharedPtr node() { return storedNode(); }

rclcpp::Logger logger() {
  auto current = storedNode();
  return current ? current->get_logger() : rclcpp::get_logger("gbplanner");
}

rclcpp::Time now() {
  auto current = storedNode();
  return current ? current->now() : rclcpp::Clock(RCL_ROS_TIME).now();
}

std::string nodeName() {
  auto current = storedNode();
  return current ? "/" + std::string(current->get_name()) : "/gbplanner_node";
}

bool hasParam(const std::string& name) {
  rclcpp::Parameter parameter;
  return lookup(name, parameter);
}

bool getParam(const std::string& name, std::string& value) {
  rclcpp::Parameter parameter;
  if (!lookup(name, parameter) || parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    return false;
  value = parameter.as_string();
  return true;
}

bool getParam(const std::string& name, double& value) {
  rclcpp::Parameter parameter;
  if (!lookup(name, parameter)) return false;
  // ROS 1 read an integer into a double, and YAML writes 1.0 as 1 often enough
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    value = parameter.as_double();
  else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    value = static_cast<double>(parameter.as_int());
  else
    return false;
  return true;
}

bool getParam(const std::string& name, float& value) {
  double d;
  if (!getParam(name, d)) return false;
  value = static_cast<float>(d);
  return true;
}

bool getParam(const std::string& name, int& value) {
  rclcpp::Parameter parameter;
  if (!lookup(name, parameter) || parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    return false;
  value = static_cast<int>(parameter.as_int());
  return true;
}

bool getParam(const std::string& name, bool& value) {
  rclcpp::Parameter parameter;
  if (!lookup(name, parameter) || parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
    return false;
  value = parameter.as_bool();
  return true;
}

bool getParam(const std::string& name, std::vector<double>& value) {
  rclcpp::Parameter parameter;
  if (!lookup(name, parameter)) return false;
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    value = parameter.as_double_array();
  } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    const auto integers = parameter.as_integer_array();
    value.assign(integers.begin(), integers.end());
  } else {
    return false;
  }
  return true;
}

bool getParam(const std::string& name, std::vector<float>& value) {
  std::vector<double> doubles;
  if (!getParam(name, doubles)) return false;
  value.assign(doubles.begin(), doubles.end());
  return true;
}

bool getParam(const std::string& name, std::vector<int>& value) {
  rclcpp::Parameter parameter;
  if (!lookup(name, parameter) ||
      parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
    return false;
  const auto integers = parameter.as_integer_array();
  value.assign(integers.begin(), integers.end());
  return true;
}

bool getParam(const std::string& name, std::vector<std::string>& value) {
  rclcpp::Parameter parameter;
  if (!lookup(name, parameter) ||
      parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    return false;
  value = parameter.as_string_array();
  return true;
}

bool getParam(const std::string& name, std::vector<bool>& value) {
  rclcpp::Parameter parameter;
  if (!lookup(name, parameter) ||
      parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL_ARRAY)
    return false;
  value = parameter.as_bool_array();
  return true;
}

}  // namespace ros2_compat
