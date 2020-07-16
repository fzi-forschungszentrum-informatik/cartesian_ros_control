// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    pass_through_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/16
 *
 */
//-----------------------------------------------------------------------------

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <pass_through_controllers/pass_through_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <cartesian_ros_control/cartesian_state_handle.h>

namespace joint_trajectory_controllers
{
  using PassThroughController =
    pass_through_controllers::PassThroughController<hardware_interface::JointTrajectoryInterface,
    hardware_interface::JointStateInterface>;
}

namespace cartesian_trajectory_controllers
{
  using PassThroughController =
    pass_through_controllers::PassThroughController<hardware_interface::CartesianTrajectoryInterface,
    cartesian_ros_control::CartesianStateInterface>;
}


PLUGINLIB_EXPORT_CLASS(joint_trajectory_controllers::PassThroughController,
                       controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(cartesian_trajectory_controllers::PassThroughController,
                       controller_interface::ControllerBase)
