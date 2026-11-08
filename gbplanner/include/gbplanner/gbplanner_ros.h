#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "gbplanner/gbplanner.h"
#include "gbplanner/gbplanner_bt_nodes.h"

class GbplannerRos
{
private:
  ros2_compat::NodeHandle nh_;
  ros2_compat::NodeHandle nh_private_;

  ros2_compat::ServiceServer planner_service_;
  ros2_compat::ServiceServer planner_homing_service_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::shared_ptr<Gbplanner> gbplanner_;
public:
  GbplannerRos(const ros2_compat::NodeHandle& nh, const ros2_compat::NodeHandle& nh_private);
  bool plannerServiceCallback(planner_msgs::srv::PlannerSrv::Request& req,
                              planner_msgs::srv::PlannerSrv::Response& res);
  bool plannerHomingServiceCallback(planner_msgs::srv::PlannerHoming::Request& req,
                              planner_msgs::srv::PlannerHoming::Response& res);

  void registerTree();

};
