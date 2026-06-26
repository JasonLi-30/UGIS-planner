/* Author: Shengjie Li */

#pragma once

#include <UGIS_motion_planner/UGIS_parameters.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

namespace my_planner
{
  class UGIS_Planner
  {
  public:
    UGIS_Planner() = default;
    virtual ~UGIS_Planner() = default;

    bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
              const planning_interface::MotionPlanRequest& req, const UGIS_Parameters& params,
              planning_interface::MotionPlanDetailedResponse& res) const;

    static std::vector<Eigen::MatrixXd> createOccupancyMapVolumeStatic(
    const planning_scene::PlanningSceneConstPtr& scene,
    double resolution);
  };
}  // namespace my_planner
