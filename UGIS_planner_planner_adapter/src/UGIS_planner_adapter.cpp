/* Author: Shengjie Li */

#include <UGIS_motion_planner/UGIS_parameters.h>
#include <UGIS_motion_planner/UGIS_planner_orchestrator.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>

#include <eigen3/Eigen/Core>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>
#include <vector>

namespace my_planner
{
static rclcpp::Logger LOGGER = rclcpp::get_logger("UGIS_planner");

class PlannerAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  PlannerAdapter() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    std::string prefix = parameter_namespace.empty() ? "" : parameter_namespace + ".";
    if (!node->get_parameter(prefix + "planning_time_limit", params_.planning_time_limit_))
    {
      params_.planning_time_limit_ = 10.0;
      RCLCPP_DEBUG(LOGGER, "Param planning_time_limit was not set. Using default value: %f",
                   params_.planning_time_limit_);
    }
    if (!node->get_parameter(prefix + "max_iterations", params_.max_iterations_))
    {
      params_.max_iterations_ = 300;
      RCLCPP_DEBUG(LOGGER, "Param max_iterations was not set. Using default value: %d", params_.max_iterations_);
    }
  }

  std::string getDescription() const override
  {
    return "My Planner";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& ps,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    RCLCPP_DEBUG(LOGGER, "My Planner: adaptAndPlan ...");

    if (!planner(ps, req, res))
      return false;

    // create a hybrid collision detector to set the collision checker as hybrid
    collision_detection::CollisionDetectorAllocatorPtr hybrid_cd(
        collision_detection::CollisionDetectorAllocatorHybrid::create());

    // create a writable planning scene
    planning_scene::PlanningScenePtr planning_scene = ps->diff();
    RCLCPP_DEBUG(LOGGER, "Configuring Planning Scene for My Planner ...");
    planning_scene->allocateCollisionDetector(hybrid_cd);

    my_planner::UGIS_Planner UGIS_planner;
    planning_interface::MotionPlanDetailedResponse res_detailed;
    res_detailed.trajectory_.push_back(res.trajectory_);

    bool planning_success = UGIS_planner.solve(planning_scene, req, params_, res_detailed);

    if (planning_success)
    {
      res.trajectory_ = res_detailed.trajectory_[0];
      res.planning_time_ += res_detailed.processing_time_[0];
    }
    res.error_code_ = res_detailed.error_code_;

    return planning_success;
  }

private:
  my_planner::UGIS_Parameters params_;
};
}  // namespace my_planner

PLUGINLIB_EXPORT_CLASS(my_planner::PlannerAdapter, planning_request_adapter::PlanningRequestAdapter)
