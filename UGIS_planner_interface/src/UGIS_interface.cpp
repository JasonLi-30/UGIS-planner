/* Author: Shengjie Li */

#include <UGIS_planner_interface/UGIS_interface.h>

namespace my_interface
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("UGIS_planner");

UGIS_Interface::UGIS_Interface(const rclcpp::Node::SharedPtr& node) : UGIS_Planner(), node_(node)
{
  loadParams();
}

void UGIS_Interface::loadParams()
{
  node_->get_parameter_or("UGIS.planning_time_limit", params_.planning_time_limit_, 10.0);
  node_->get_parameter_or("UGIS.max_iterations", params_.max_iterations_, 300);
}
}  // namespace my_interface
