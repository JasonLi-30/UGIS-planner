/* Author: Shengjie Li */

#include <UGIS_motion_planner/UGIS_parameters.h>

#include <algorithm>

namespace my_planner
{
UGIS_Parameters::UGIS_Parameters()
{
  planning_time_limit_ = 6.0;
  max_iterations_ = 300;
}

UGIS_Parameters::~UGIS_Parameters() = default;

void UGIS_Parameters::setRecoveryParams(int planning_time_limit, int max_iterations)
{
  this->planning_time_limit_ = planning_time_limit;
  this->max_iterations_ = max_iterations;
}
}  // namespace my_planner
