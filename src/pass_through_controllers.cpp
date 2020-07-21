// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    pass_through_controllers.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/16
 *
 */
//-----------------------------------------------------------------------------

// Pluginlib
#include <pass_through_controllers/trajectory_interface.h>
#include <pluginlib/class_list_macros.h>

// Project
#include <pass_through_controllers/pass_through_controllers.h>
#include <hardware_interface/joint_state_interface.h>
#include <cartesian_ros_control/cartesian_state_handle.h>

namespace joint_trajectory_controllers
{
  bool PassThroughController::init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh)
  {
    // Get names of readable joints from the parameter server
    std::vector<std::string> joint_names;
    if (!controller_nh.getParam("joints", joint_names))
    {
      ROS_ERROR_STREAM("Failed to load " << controller_nh.getNamespace()
                                         << "/joints from parameter server");
      return false;
    }

    // Get handle for trajectory forwarding from the hardware interface
    auto* traj_interface = robot_hw->get<hardware_interface::JointTrajectoryInterface>();
    if (traj_interface == nullptr)
    {
      ROS_ERROR("joint_trajectory_controllers/PassThroughController: Error getting "
          "trajectory interface from hardware");
      return false;
    }
    try
    {
      traj_interface->setResources(joint_names);
      m_trajectory_handle = std::make_unique<hardware_interface::JointTrajectoryHandle>(
          traj_interface->getHandle("joint_trajectory_handle"));
    }
    catch (hardware_interface::HardwareInterfaceException& ex)
    {
      ROS_ERROR_STREAM(
          "joint_trajectory_controllersPassThroughController: Exception getting trajectory handle from interface: "
          << ex.what());
      return false;
    }

    // Get the joint state handles for computing action feedback
    auto* joint_state_interface = robot_hw->get<hardware_interface::JointStateInterface>();
    if (joint_state_interface == nullptr)
    {
      ROS_ERROR_STREAM("joint_trajectory_controllersPassThroughController: Error getting joint "
                       "state interface from hardware");
      return false;
    }
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      try
      {
        m_joint_handles.push_back(joint_state_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& ex)
      {
        ROS_ERROR_STREAM("joint_trajectory_controllersPassThroughController: Exception getting "
                         "joint state handles: "
                         << ex.what());
        return false;
      }
    }


    return true;
  }

  void PassThroughController::starting(const ros::Time& time)
  {
  }

  void PassThroughController::stopping(const ros::Time& time)
  {
  }

  void PassThroughController::update(const ros::Time& time, const ros::Duration& period)
  {
  }

}

PLUGINLIB_EXPORT_CLASS(joint_trajectory_controllers::PassThroughController,
                       controller_interface::ControllerBase)




namespace cartesian_trajectory_controllers
{
  bool PassThroughController::init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh)
  {
    return true;
  }

  void PassThroughController::starting(const ros::Time& time)
  {
  }

  void PassThroughController::stopping(const ros::Time& time)
  {
  }

  void PassThroughController::update(const ros::Time& time, const ros::Duration& period)
  {
  }

}


PLUGINLIB_EXPORT_CLASS(cartesian_trajectory_controllers::PassThroughController,
                       controller_interface::ControllerBase)
