/* Author: Shengjie Li */

#pragma once

#include <UGIS_motion_planner/UGIS_utils.h>
#include <moveit/robot_model/robot_model.h>

#include <eigen3/Eigen/Core>
#include <moveit_msgs/msg/motion_plan_detailed_response.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace my_planner
{
class UGIS_Trajectory
{
public:
  /**
   * \brief Constructs a trajectory for a given robot model, trajectory duration, and discretization
   */
  UGIS_Trajectory(const moveit::core::RobotModelConstPtr& robot_model, double duration, double discretization,
                  const std::string& group_name);

  /**
   * \brief Constructs a trajectory for a given robot model, number of trajectory points, and discretization
   */
  UGIS_Trajectory(const moveit::core::RobotModelConstPtr& robot_model, size_t num_points, double discretization,
                  const std::string& group_name);

  /**
   * \brief Creates a new containing only the joints of interest, and adds padding to the start
   * and end if needed, to have enough trajectory points for the differentiation rules
   */
  UGIS_Trajectory(const UGIS_Trajectory& source_traj, const std::string& group_name, int diff_rule_length);

  UGIS_Trajectory(const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name,
                  const trajectory_msgs::msg::JointTrajectory& traj);

  /**
   * \brief Destructor
   */
  virtual ~UGIS_Trajectory() = default;

  double& operator()(size_t traj_point, size_t joint);

  double operator()(size_t traj_point, size_t joint) const;

  Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point);

  Eigen::MatrixXd::ColXpr getJointTrajectory(int joint);

  /**
   * \brief Gets the number of points in the trajectory
   */
  size_t getNumPoints() const;

  size_t getNumFreePoints() const;

  /**
   * \brief Gets the number of joints in each trajectory point
   */
  size_t getNumJoints() const;

  /**
   * \brief Gets the discretization time interval of the trajectory
   */
  double getDiscretization() const;

  /**
   * \brief Generates a minimum jerk trajectory from the start index to end index
   *
   * Only modifies points from start_index_ to end_index_, inclusive.
   */
  void fillInMinJerk();

  /**
   * \brief Generates a linearly interpolated trajectory from the start index to end index
   *
   * Only modifies points from start_index_ to end_index_, inclusive
   */
  void fillInLinearInterpolation();

  /**
   * \brief Generates a cubic interpolation of the trajectory from the start index to end index
   *
   * Only modifies points from start_index_ to end_index_, inclusive
   */
  void fillInCubicInterpolation();

  /**
   * \brief Receives the path obtained from a given MotionPlanDetailedResponse res object's trajectory (e.g., trajectory
   * produced by OMPL) and puts it into the appropriate trajectory format required for UGIS
   * @param res
   */
  bool fillInFromTrajectory(const robot_trajectory::RobotTrajectory& trajectory);

  /**
   * \brief This function assigns the given \a source RobotState to the row at index \a UGIS_trajectory_point
   *
   * @param source The source RobotState
   * @param UGIS_trajectory_point index of the UGIS_trajectory's point (row)
   * @param group  JointModelGroup determining the joints to copy
   */
  void assign_UGIS_TrajectoryPointFromRobotState(const moveit::core::RobotState& source, size_t UGIS_trajectory_point,
                                                const moveit::core::JointModelGroup* group);

  /**
   * \brief Sets the start and end index for the modifiable part of the trajectory
   *
   * (Everything before the start and after the end index is considered fixed)
   * The values default to 1 and getNumPoints()-2
   */
  void setStartEndIndex(size_t start_index, size_t end_index);

  /**
   * \brief Gets the start index
   */
  size_t getStartIndex() const;

  /**
   * \brief Gets the end index
   */
  size_t getEndIndex() const;

  /**
   * \brief Gets the entire trajectory matrix
   */
  Eigen::MatrixXd& getTrajectory();

  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeTrajectoryBlock();

  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeJointTrajectoryBlock(size_t joint);

  /**
   * \brief Updates the full trajectory (*this) from the group trajectory
   */
  void updateFromGroupTrajectory(const UGIS_Trajectory& group_trajectory);

  /**
   * \brief Gets the index in the full trajectory which was copied to this group trajectory
   */
  size_t getFullTrajectoryIndex(size_t i) const;

  /**
   * \brief Gets the joint velocities at the given trajectory point
   */
  template <typename Derived>
  void getJointVelocities(size_t traj_point, Eigen::MatrixBase<Derived>& velocities);

  double getDuration() const;

private:
  void init(); /**< \brief Allocates memory for the trajectory */

  std::string planning_group_name_;  //< Planning group that this trajectory corresponds to, if any
  size_t num_points_;                //< Number of points in the trajectory
  size_t num_joints_;                //< Number of joints in each trajectory point
  double discretization_;            //< Discretization of the trajectory
  double duration_;                  //< Duration of the trajectory
  Eigen::MatrixXd trajectory_;       //< Storage for the actual trajectory
  size_t start_index_;  
  size_t end_index_; 
  std::vector<size_t> full_trajectory_index_;  //< If this is a "group" trajectory, the indices from the original traj
};

///////////////////////// inline functions follow //////////////////////

inline double& UGIS_Trajectory::operator()(size_t traj_point, size_t joint)
{
  return trajectory_(traj_point, joint);
}

inline double UGIS_Trajectory::operator()(size_t traj_point, size_t joint) const
{
  return trajectory_(traj_point, joint);
}

inline Eigen::MatrixXd::RowXpr UGIS_Trajectory::getTrajectoryPoint(int traj_point)
{
  return trajectory_.row(traj_point);
}

inline Eigen::MatrixXd::ColXpr UGIS_Trajectory::getJointTrajectory(int joint)
{
  return trajectory_.col(joint);
}

inline size_t UGIS_Trajectory::getNumPoints() const
{
  return num_points_;
}

inline size_t UGIS_Trajectory::getNumFreePoints() const
{
  return (end_index_ - start_index_) + 1;
}

inline size_t UGIS_Trajectory::getNumJoints() const
{
  return num_joints_;
}

inline double UGIS_Trajectory::getDiscretization() const
{
  return discretization_;
}

inline void UGIS_Trajectory::setStartEndIndex(size_t start_index, size_t end_index)
{
  start_index_ = start_index;
  end_index_ = end_index;
}

inline size_t UGIS_Trajectory::getStartIndex() const
{
  return start_index_;
}

inline size_t UGIS_Trajectory::getEndIndex() const
{
  return end_index_;
}

inline Eigen::MatrixXd& UGIS_Trajectory::getTrajectory()
{
  return trajectory_;
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> UGIS_Trajectory::getFreeTrajectoryBlock()
{
  return trajectory_.block(start_index_, 0, getNumFreePoints(), getNumJoints());
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
UGIS_Trajectory::getFreeJointTrajectoryBlock(size_t joint)
{
  return trajectory_.block(start_index_, joint, getNumFreePoints(), 1);
}

inline size_t UGIS_Trajectory::getFullTrajectoryIndex(size_t i) const
{
  return full_trajectory_index_[i];
}

template <typename Derived>
void UGIS_Trajectory::getJointVelocities(size_t traj_point, Eigen::MatrixBase<Derived>& velocities)
{
  velocities.setZero();
  double inv_time = 1.0 / discretization_;

  for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; ++k)
  {
    velocities += (inv_time * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2]) * trajectory_.row(traj_point + k).transpose();
  }
}

inline double UGIS_Trajectory::getDuration() const
{
  return duration_;
}
}  // namespace my_planner
