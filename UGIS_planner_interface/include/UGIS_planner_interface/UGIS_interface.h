/* Author: E. Gil Jones */

#pragma once

#include <UGIS_motion_planner/UGIS_parameters.h>
#include <UGIS_motion_planner/UGIS_planner_orchestrator.h>

#include <rclcpp/rclcpp.hpp>

namespace my_interface
{
MOVEIT_CLASS_FORWARD(UGIS_Interface);  // Defines UGIS_InterfacePtr, ConstPtr, WeakPtr... etc

class UGIS_Interface : public my_planner::UGIS_Planner
{
public:
  UGIS_Interface(const rclcpp::Node::SharedPtr& node);

  const my_planner::UGIS_Parameters& getParams() const
  {
    return params_;
  }

protected:
  /** @brief Configure everything using the param server */
  void loadParams();

  std::shared_ptr<rclcpp::Node> node_;  /// The ROS node

  my_planner::UGIS_Parameters params_;
};
}  // namespace my_interface
