// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    pass_through_controllers.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/10/13
 *
 */
//-----------------------------------------------------------------------------

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <pass_through_controllers/pass_through_controllers.h>

// Exports

namespace joint_trajectory_controllers {

using PassThroughController =
  trajectory_controllers::PassThroughController<hardware_interface::JointTrajectoryInterface>;
}

namespace cartesian_trajectory_controllers {

using PassThroughController =
  trajectory_controllers::PassThroughController<hardware_interface::CartesianTrajectoryInterface>;
}

PLUGINLIB_EXPORT_CLASS(joint_trajectory_controllers::PassThroughController,
                       controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(cartesian_trajectory_controllers::PassThroughController,
                       controller_interface::ControllerBase)
