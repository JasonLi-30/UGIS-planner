
#include <UGIS_planner_interface/UGIS_planning_context.h>
#include <moveit/robot_state/conversions.h>

namespace my_interface
{
UGIS_PlanningContext::UGIS_PlanningContext(const std::string& name, const std::string& group,
                                           const moveit::core::RobotModelConstPtr& model,
                                           const rclcpp::Node::SharedPtr& node)
  : planning_interface::PlanningContext(name, group), robot_model_(model)
{
  UGIS_interface_ = std::make_shared<UGIS_Interface>(node);
}

bool UGIS_PlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  return UGIS_interface_->solve(planning_scene_, request_, UGIS_interface_->getParams(), res);
}

bool UGIS_PlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  planning_interface::MotionPlanDetailedResponse res_detailed;
  bool planning_success = solve(res_detailed);
  // std::cout << "planning_success2: " << planning_success << std::endl;
  res.error_code_ = res_detailed.error_code_;

  if (planning_success)
  {
    res.trajectory_ = res_detailed.trajectory_[0];
    res.planning_time_ = res_detailed.processing_time_[0];
  }

  return planning_success;
}

bool UGIS_PlanningContext::terminate()
{
  // TODO - make interruptible
  return true;
}

void UGIS_PlanningContext::clear()
{
}

}  // namespace my_interface
