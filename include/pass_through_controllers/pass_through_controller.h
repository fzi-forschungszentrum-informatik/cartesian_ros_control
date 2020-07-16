// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    pass_through_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/16
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <pass_through_controllers/trajectory_interface.h>


namespace pass_through_controllers {

template <class TrajectoryInterfaceType, class StateInterfaceType>
class PassThroughController
  : public controller_interface::MultiInterfaceController<TrajectoryInterfaceType,
                                                          StateInterfaceType>
{
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

};

}

#include <pass_through_controllers/pass_through_controller.hpp>
