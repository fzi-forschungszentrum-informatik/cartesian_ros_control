// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    pass_through_controller.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/16
 *
 */
//-----------------------------------------------------------------------------

#include <pass_through_controllers/pass_through_controller.h>

namespace pass_through_controllers {

template <class TrajectoryInterfaceType, class StateInterfaceType>
bool PassThroughController<TrajectoryInterfaceType, StateInterfaceType>::init(
  hardware_interface::RobotHW* robot_hw,
  ros::NodeHandle& root_nh,
  ros::NodeHandle& controller_nh)
{
  auto* interface = robot_hw->get<TrajectoryInterfaceType>();

  return true;
};

template <class TrajectoryInterfaceType, class StateInterfaceType>
void PassThroughController<TrajectoryInterfaceType, StateInterfaceType>::starting(
  const ros::Time& time)
{
};

template <class TrajectoryInterfaceType, class StateInterfaceType>
void PassThroughController<TrajectoryInterfaceType, StateInterfaceType>::stopping(
  const ros::Time& time)
{
};

template <class TrajectoryInterfaceType, class StateInterfaceType>
void PassThroughController<TrajectoryInterfaceType, StateInterfaceType>::update(
  const ros::Time& time, const ros::Duration& period)
{
};

}
