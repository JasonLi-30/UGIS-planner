/* Author: Shengjie Li */

#include <string>
#include <vector>

#pragma once

namespace my_planner
{
class UGIS_Parameters
{
public:
  UGIS_Parameters();
  virtual ~UGIS_Parameters();
  void setRecoveryParams(int planning_time_limit, int max_iterations);

public:
  double planning_time_limit_; 
  int max_iterations_;
};

}  // namespace my_planner
