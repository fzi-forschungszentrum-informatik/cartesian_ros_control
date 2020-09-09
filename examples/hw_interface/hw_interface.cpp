// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    hw_interface.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/09/09
 *
 */
//-----------------------------------------------------------------------------


#include "hw_interface.h"


namespace examples {

HWInterface::HWInterface()
{
  // Get names of controllable joints from the parameter server
  ros::NodeHandle nh;
  if (!nh.getParam("joints", m_joint_names))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/joints"
                                       << " from parameter server");
    throw std::logic_error("Failed to initialize ros control.");
  }

  // TODO:
  bool error = false;
  error += nh.getParam("ref_frame_id", ref_frame_id_);
  error += nh.getParam("frame_id", frame_id_);

  const int nr_joints = m_joint_names.size();
  m_cmd.resize(nr_joints);
  m_pos.resize(nr_joints);
  m_vel.resize(nr_joints);
  m_eff.resize(nr_joints);

  // Initialize and register joint state handles
  for (int i = 0; i < nr_joints; ++i)
  {
    m_joint_state_handles.push_back(
      hardware_interface::JointStateHandle(m_joint_names[i], &m_pos[i], &m_vel[i], &m_eff[i]));

    m_jnt_state_interface.registerHandle(m_joint_state_handles[i]);
  }
  registerInterface(&m_jnt_state_interface);

  // Initialize and register a Cartesian state handle
  cartesian_ros_control::CartesianStateHandle cartesian_state_handle =
    cartesian_ros_control::CartesianStateHandle(ref_frame_id_,
                                                frame_id_,
                                                &cartesian_pose_,
                                                &cartesian_twist_,
                                                &cartesian_accel_,
                                                &cartesian_jerk_);
  m_cart_state_interface.registerHandle(cartesian_state_handle);
  registerInterface(&m_cart_state_interface);

  // Initialize and register joint position command handles.
  for (int i = 0; i < nr_joints; ++i)
  {
    m_joint_handles.push_back(hardware_interface::JointHandle(
      m_jnt_state_interface.getHandle(m_joint_names[i]), &m_cmd[i]));

    m_jnt_pos_interface.registerHandle(m_joint_handles[i]);
  }
  registerInterface(&m_jnt_pos_interface);

  // Initialize and register trajectory command handles for PassThroughControllers
  hardware_interface::JointTrajectoryHandle joint_trajectory_handle =
    hardware_interface::JointTrajectoryHandle(
      "joint_trajectory_handle",
      &m_jnt_traj_cmd,
      std::bind(&HWInterface::startJointInterpolation, this, std::placeholders::_1),
      std::bind(&HWInterface::cancelJointInterpolation, this));

  m_jnt_traj_interface.registerHandle(joint_trajectory_handle);
  registerInterface(&m_jnt_traj_interface);

  // Initialize and register Cartesian trajectory command handles for PassThroughControllers
  hardware_interface::CartesianTrajectoryHandle cartesian_trajectory_handle =
    hardware_interface::CartesianTrajectoryHandle("cartesian_trajectory_handle", &m_cart_traj_cmd);
  m_cart_traj_interface.registerHandle(cartesian_trajectory_handle);
  registerInterface(&m_cart_traj_interface);


  // Yellow greeting to get started
  ROS_INFO("\033[1;33mExample HW interface is ready \033[0m");
}

HWInterface::~HWInterface() {}


void HWInterface::read(const ros::Time& time, const ros::Duration& period)
{
  // nothing to do for this example HW
}

void HWInterface::write(const ros::Time& time, const ros::Duration& period)
{
  // nothing to do for this example HW
}

void HWInterface::startJointInterpolation(const hardware_interface::JointTrajectory& trajectory)
{
  // TODO: Send action goal to vendor joint trajectory controller.
}

void HWInterface::cancelJointInterpolation()
{
  // TODO: Send preemption request to vendor joint trajectory controller.
}


} // namespace examples
