#pragma once

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Core>
#include <iostream>

namespace my_planner
{
static const int DIFF_RULE_LENGTH = 7;

// the differentiation rules (centered at the center)
static const double DIFF_RULES[3][DIFF_RULE_LENGTH] = {
  { 0, 0, -2 / 6.0, -3 / 6.0, 6 / 6.0, -1 / 6.0, 0 },                       // velocity
  { 0, -1 / 12.0, 16 / 12.0, -30 / 12.0, 16 / 12.0, -1 / 12.0, 0 },         // acceleration
  { 0, 1 / 12.0, -17 / 12.0, 46 / 12.0, -46 / 12.0, 17 / 12.0, -1 / 12.0 }  // jerk
};

static inline void robotStateToArray(const moveit::core::RobotState& state, const std::string& planning_group_name,
                                     Eigen::MatrixXd::RowXpr joint_array)
{
  const moveit::core::JointModelGroup* group = state.getJointModelGroup(planning_group_name);
  size_t joint_index = 0;
  for (const moveit::core::JointModel* jm : group->getActiveJointModels())
    joint_array[joint_index++] = state.getVariablePosition(jm->getFirstVariableIndex());
}

// copied from geometry/angles/angles.h
static inline double normalizeAnglePositive(double angle)
{
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

static inline double normalizeAngle(double angle)
{
  double a = normalizeAnglePositive(angle);
  if (a > M_PI)
    a -= 2.0 * M_PI;
  return a;
}

static inline double shortestAngularDistance(double start, double end)
{
  double res = normalizeAnglePositive(normalizeAnglePositive(end) - normalizeAnglePositive(start));
  if (res > M_PI)
  {
    res = -(2.0 * M_PI - res);
  }
  return normalizeAngle(res);
}

}  // namespace my_planner
