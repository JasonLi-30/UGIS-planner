
#pragma once

#include <UGIS_planner_interface/UGIS_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <rclcpp/rclcpp.hpp>

namespace my_interface
{
MOVEIT_CLASS_FORWARD(UGIS_PlanningContext);  // Defines UGIS_PlanningContextPtr, ConstPtr, WeakPtr... etc

class UGIS_PlanningContext : public planning_interface::PlanningContext
{
public:
  bool solve(planning_interface::MotionPlanResponse& res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& res) override;

  void clear() override;
  bool terminate() override;

  UGIS_PlanningContext(const std::string& name, const std::string& group, const moveit::core::RobotModelConstPtr& model,
                       const rclcpp::Node::SharedPtr& node);

  ~UGIS_PlanningContext() override = default;

  void initialize();

private:
  UGIS_InterfacePtr UGIS_interface_;
  moveit::core::RobotModelConstPtr robot_model_;
};

}  // namespace my_interface
