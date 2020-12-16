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
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include <functional>


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
  error += nh.getParam("ref_frame_id", m_ref_frame_id);
  error += nh.getParam("frame_id", m_frame_id);

  // Connect dynamic reconfigure and overwrite the default values with values
  // on the parameter server. This is done automatically if parameters with
  // the according names exist.
  m_callback_type = std::bind(&HWInterface::dynamicReconfigureCallback,
                              this,
                              std::placeholders::_1,
                              std::placeholders::_2);

  m_reconfig_server =
    std::make_shared<dynamic_reconfigure::Server<SpeedScalingConfig> >(nh);
  m_reconfig_server->setCallback(m_callback_type);

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
    cartesian_ros_control::CartesianStateHandle(m_ref_frame_id,
                                                m_frame_id,
                                                &m_cartesian_pose,
                                                &m_cartesian_twist,
                                                &m_cartesian_accel,
                                                &m_cartesian_jerk);
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
      &m_jnt_traj_cmd,
      &m_jnt_traj_feedback,
      std::bind(&HWInterface::startJointInterpolation, this, std::placeholders::_1),
      std::bind(&HWInterface::cancelJointInterpolation, this));

  m_jnt_traj_interface.registerHandle(joint_trajectory_handle);
  registerInterface(&m_jnt_traj_interface);

  // Initialize and register Cartesian trajectory command handles for PassThroughControllers
  hardware_interface::CartesianTrajectoryHandle cartesian_trajectory_handle =
    hardware_interface::CartesianTrajectoryHandle(
      &m_cart_traj_cmd,
      &m_cart_traj_feedback,
      std::bind(&HWInterface::startCartesianInterpolation, this, std::placeholders::_1),
      std::bind(&HWInterface::cancelCartesianInterpolation, this));

  m_cart_traj_interface.registerHandle(cartesian_trajectory_handle);
  registerInterface(&m_cart_traj_interface);

  // Initialize and register speed scaling.
  // Note: The handle's name is a convention.
  // ROS-controllers will use this name when calling getHandle().
  m_speedsc_interface.registerHandle(
      ur_controllers::SpeedScalingHandle("speed_scaling_factor", &m_speed_scaling));
  registerInterface(&m_speedsc_interface);


  // Robot dummy communication
  m_joint_based_communication =
    std::make_unique<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> >(
      "/robot_dummy/vendor_joint_controller/follow_joint_trajectory", true);

  m_cartesian_based_communication =
    std::make_unique<actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> >(
      "/robot_dummy/vendor_cartesian_controller/follow_cartesian_trajectory", true);

  m_joint_based_communication->waitForServer();
  m_cartesian_based_communication->waitForServer();

  ROS_INFO("Example HW interface is ready");
}

HWInterface::~HWInterface() {}


void HWInterface::read(const ros::Time& time, const ros::Duration& period)
{
  // Code for conventional ROS-control loop here.
}

void HWInterface::write(const ros::Time& time, const ros::Duration& period)
{
  // Code for conventional ROS-control loop here.
}

void HWInterface::startJointInterpolation(const hardware_interface::JointTrajectory& trajectory)
{
  m_joint_based_communication->sendGoal(
    trajectory,
    0, // no done callback
    0, // no active callback
    std::bind(
      &HWInterface::handleJointFeedback, this, std::placeholders::_1)); // Process feedback continuously
}

void HWInterface::startCartesianInterpolation(const hardware_interface::CartesianTrajectory& trajectory)
{
  m_cartesian_based_communication->sendGoal(
    trajectory,
    0, // no done callback
    0, // no active callback
    std::bind(
      &HWInterface::handleCartesianFeedback, this, std::placeholders::_1)); // Process feedback continuously
}

void HWInterface::cancelJointInterpolation()
{
  m_joint_based_communication->cancelGoal();
}

void HWInterface::cancelCartesianInterpolation()
{
  m_cartesian_based_communication->cancelGoal();
}

void HWInterface::dynamicReconfigureCallback(SpeedScalingConfig& config, uint32_t level)
{
  // Let's hope for "thread safety" with fundamental types.
  m_speed_scaling = config.speed_scaling;
}

void HWInterface::handleJointFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  m_jnt_traj_interface.getHandle("joint_trajectory_handle").setFeedback(*feedback);
}

void HWInterface::handleCartesianFeedback(
  const cartesian_control_msgs::FollowCartesianTrajectoryFeedbackConstPtr& feedback)
{
  m_cart_traj_interface.getHandle("cartesian_trajectory_handle").setFeedback(*feedback);
}

} // namespace examples
