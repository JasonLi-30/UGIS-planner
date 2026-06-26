
#include <UGIS_planner_interface/UGIS_planning_context.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>

#include <pluginlib/class_list_macros.hpp>
#include <vector>

namespace my_interface
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("UGIS_Planner");

class UGIS_PlannerManager : public planning_interface::PlannerManager
{
public:
  UGIS_PlannerManager() : planning_interface::PlannerManager()
  {
  }

  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& /* unused */) override
  {
    planning_interface::PlannerConfigurationMap pconfig;
    for (const std::string& group : model->getJointModelGroupNames())
    {
      planning_contexts_[group] = std::make_shared<UGIS_PlanningContext>("my_planning_context", group, model, node);
      const planning_interface::PlannerConfigurationSettings planner_config_settings{
        group, group, std::map<std::string, std::string>()
      };
      pconfig[planner_config_settings.name] = planner_config_settings;
    }

    setPlannerConfigurations(pconfig);
    return true;
  }

  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    std::cout << "===getPlanningContext===" << std::endl;
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    if (req.group_name.empty())
    {
      RCLCPP_ERROR(LOGGER, "No group specified to plan for");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    if (!planning_scene)
    {
      RCLCPP_ERROR(LOGGER, "No planning scene supplied as input");
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return planning_interface::PlanningContextPtr();
    }

    // create PlanningScene using hybrid collision detector
    planning_scene::PlanningScenePtr ps = planning_scene->diff();
    // ps->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create());

    // retrieve and configure existing context
    const UGIS_PlanningContextPtr& context = planning_contexts_.at(req.group_name);
    context->setPlanningScene(ps);
    context->setMotionPlanRequest(req);
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return context;
  }

  bool canServiceRequest(const planning_interface::MotionPlanRequest& /*req*/) const override
  {
    // TODO: this is a dummy implementation
    //      capabilities.dummy = false;
    return true;
  }

  std::string getDescription() const override
  {
    return "UGIS";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.resize(1);
    algs[0] = "UGIS";
  }

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override
  {
    config_settings_ = pcs;
  }

protected:
  std::map<std::string, UGIS_PlanningContextPtr> planning_contexts_;
};

}  // namespace my_interface

PLUGINLIB_EXPORT_CLASS(my_interface::UGIS_PlannerManager, planning_interface::PlannerManager)
