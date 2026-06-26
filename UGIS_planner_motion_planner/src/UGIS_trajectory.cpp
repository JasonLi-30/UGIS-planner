/* Author: Shengjie Li */

#include <UGIS_motion_planner/UGIS_trajectory.h>

namespace my_planner
{
UGIS_Trajectory::UGIS_Trajectory(const moveit::core::RobotModelConstPtr& robot_model, double duration,
                                 double discretization, const std::string& group_name)
  : UGIS_Trajectory(robot_model, static_cast<size_t>(duration / discretization) + 1, discretization, group_name)
{
}

UGIS_Trajectory::UGIS_Trajectory(const moveit::core::RobotModelConstPtr& robot_model, size_t num_points,
                                 double discretization, const std::string& group_name)
  : planning_group_name_(group_name)
  , num_points_(num_points)
  , discretization_(discretization)
  , duration_((num_points - 1) * discretization)
  , start_index_(1)
  , end_index_(num_points_ - 2)
{
  const moveit::core::JointModelGroup* model_group = robot_model->getJointModelGroup(planning_group_name_);
  num_joints_ = model_group->getActiveJointModels().size();
  init();
}

UGIS_Trajectory::UGIS_Trajectory(const UGIS_Trajectory& source_traj, const std::string& group_name, int diff_rule_length)
  : planning_group_name_(group_name), discretization_(source_traj.discretization_)
{
  num_joints_ = source_traj.getNumJoints();
  // std::cout << "num_joints_: " << num_joints_ << std::endl;

  // figure out the num_points_:
  // we need diff_rule_length-1 extra points on either side:
  int start_extra = (diff_rule_length - 1) - source_traj.start_index_;
  int end_extra = (diff_rule_length - 1) - ((source_traj.num_points_ - 1) - source_traj.end_index_);

  num_points_ = source_traj.num_points_ + start_extra + end_extra;
  start_index_ = diff_rule_length - 1;
  end_index_ = (num_points_ - 1) - (diff_rule_length - 1);
  duration_ = (num_points_ - 1) * discretization_;

  // std::cout << "source_traj.num_points_: " << source_traj.num_points_ << std::endl;
  // std::cout << "start_extra: " << start_extra << std::endl;
  // std::cout << "end_extra: " << end_extra << std::endl;
  // std::cout << "num_points_: " << num_points_ << std::endl;

  // std::cout << "===traj init start===" << std::endl;

  // allocate the memory:
  init();

  full_trajectory_index_.resize(num_points_);

  // now copy the trajectories over:
  for (size_t i = 0; i < num_points_; ++i)
  {
    int source_traj_point = i - start_extra;
    if (source_traj_point < 0)
      source_traj_point = 0;
    if (static_cast<size_t>(source_traj_point) >= source_traj.num_points_)
      source_traj_point = source_traj.num_points_ - 1;
    full_trajectory_index_[i] = source_traj_point;
    getTrajectoryPoint(i) = const_cast<UGIS_Trajectory&>(source_traj).getTrajectoryPoint(source_traj_point);
  }

  // std::cout << "===traj init end===" << std::endl;

}

void UGIS_Trajectory::init()
{
  trajectory_.resize(num_points_, num_joints_);
}

void UGIS_Trajectory::updateFromGroupTrajectory(const UGIS_Trajectory& group_trajectory)
{
  end_index_ = group_trajectory.trajectory_.rows() - 2;
  num_points_ = group_trajectory.trajectory_.rows();
  size_t num_vars_free = end_index_ - start_index_ + 1;

  trajectory_ = group_trajectory.trajectory_;
}

void UGIS_Trajectory::fillInLinearInterpolation()
{
  double start_index = start_index_ - 1;
  double end_index = end_index_ + 1;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    double theta = ((*this)(end_index, i) - (*this)(start_index, i)) / (end_index - 1);
    for (size_t j = start_index + 1; j < end_index; ++j)
    {
      (*this)(j, i) = (*this)(start_index, i) + j * theta;
    }
  }
}

void UGIS_Trajectory::fillInCubicInterpolation()
{
  double start_index = start_index_ - 1;
  double end_index = end_index_ + 1;
  double dt = 0.001;
  std::vector<double> coeffs(4, 0);
  double total_time = (end_index - 1) * dt;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    coeffs[0] = (*this)(start_index, i);
    coeffs[2] = (3 / (pow(total_time, 2))) * ((*this)(end_index, i) - (*this)(start_index, i));
    coeffs[3] = (-2 / (pow(total_time, 3))) * ((*this)(end_index, i) - (*this)(start_index, i));

    double t;
    for (size_t j = start_index + 1; j < end_index; ++j)
    {
      t = j * dt;
      (*this)(j, i) = coeffs[0] + coeffs[2] * pow(t, 2) + coeffs[3] * pow(t, 3);
    }
  }
}

void UGIS_Trajectory::fillInMinJerk()
{
  double start_index = start_index_ - 1;
  double end_index = end_index_ + 1;
  double td[6];  // powers of the time duration
  td[0] = 1.0;
  td[1] = (end_index - start_index) * discretization_;

  for (unsigned int i = 2; i <= 5; ++i)
    td[i] = td[i - 1] * td[1];

  // calculate the spline coefficients for each joint:
  // (these are for the special case of zero start and end vel and acc)
  std::vector<std::array<double, 6>> coeff(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i)
  {
    double x0 = (*this)(start_index, i);
    double x1 = (*this)(end_index, i);
    coeff[i][0] = x0;
    coeff[i][1] = 0;
    coeff[i][2] = 0;
    coeff[i][3] = (-20 * x0 + 20 * x1) / (2 * td[3]);
    coeff[i][4] = (30 * x0 - 30 * x1) / (2 * td[4]);
    coeff[i][5] = (-12 * x0 + 12 * x1) / (2 * td[5]);
  }

  // now fill in the joint positions at each time step
  for (size_t i = start_index + 1; i < end_index; ++i)
  {
    double ti[6];  // powers of the time index point
    ti[0] = 1.0;
    ti[1] = (i - start_index) * discretization_;
    for (unsigned int k = 2; k <= 5; ++k)
      ti[k] = ti[k - 1] * ti[1];

    for (size_t j = 0; j < num_joints_; ++j)
    {
      (*this)(i, j) = 0.0;
      for (unsigned int k = 0; k <= 5; ++k)
      {
        (*this)(i, j) += ti[k] * coeff[j][k];
      }
    }
  }
}

bool UGIS_Trajectory::fillInFromTrajectory(const robot_trajectory::RobotTrajectory& trajectory)
{
  // check if input trajectory has less than two states (start and goal), function returns false if condition is true
  if (trajectory.getWayPointCount() < 2)
    return false;

  const size_t max_output_index = getNumPoints() - 1;
  const size_t max_input_index = trajectory.getWayPointCount() - 1;

  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  moveit::core::RobotState interpolated(trajectory.getRobotModel());
  for (size_t i = 0; i <= max_output_index; ++i)
  {
    double fraction = static_cast<double>(i * max_input_index) / max_output_index;
    const size_t prev_idx = std::trunc(fraction);  // integer part
    fraction = fraction - prev_idx;                // fractional part
    const size_t next_idx = prev_idx == max_input_index ? prev_idx : prev_idx + 1;
    trajectory.getWayPoint(prev_idx).interpolate(trajectory.getWayPoint(next_idx), fraction, interpolated, group);
    assign_UGIS_TrajectoryPointFromRobotState(interpolated, i, group);
  }
  return true;
}

void UGIS_Trajectory::assign_UGIS_TrajectoryPointFromRobotState(const moveit::core::RobotState& source,
                                                               size_t UGIS_trajectory_point_index,
                                                               const moveit::core::JointModelGroup* group)
{
  Eigen::MatrixXd::RowXpr target = getTrajectoryPoint(UGIS_trajectory_point_index);
  assert(group->getActiveJointModels().size() == static_cast<size_t>(target.cols()));
  size_t joint_index = 0;
  for (const moveit::core::JointModel* jm : group->getActiveJointModels())
  {
    assert(jm->getVariableCount() == 1);
    target[joint_index++] = source.getVariablePosition(jm->getFirstVariableIndex());
  }
}

}  // namespace my_planner
