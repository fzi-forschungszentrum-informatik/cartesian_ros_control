// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    pass_through_controllers.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/16
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <memory>
#include <pass_through_controllers/trajectory_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <cartesian_ros_control/cartesian_state_handle.h>

namespace joint_trajectory_controllers {

class PassThroughController
  : public controller_interface::MultiInterfaceController<
      hardware_interface::JointTrajectoryInterface,
      hardware_interface::JointStateInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

private:
  std::unique_ptr<hardware_interface::JointTrajectoryHandle> m_trajectory_handle;
  std::vector<hardware_interface::JointStateHandle> m_joint_handles;
};
} // namespace joint_trajectory_controllers






namespace cartesian_trajectory_controllers {

class PassThroughController
  : public controller_interface::MultiInterfaceController<
      hardware_interface::CartesianTrajectoryInterface,
      cartesian_ros_control::CartesianStateInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

private:
};

} // namespace cartesian_trajectory_controllers
