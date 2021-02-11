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
#include "cartesian_interface/cartesian_command_interface.h"
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
  if (!nh.getParam("joints", joint_names_))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/joints"
                                       << " from parameter server");
    throw std::logic_error("Failed to initialize ros control.");
  }

  // Current UR driver convention
  ref_frame_id_ = "base";
  frame_id_ = "tool0_controller";

  // Connect dynamic reconfigure and overwrite the default values with values
  // on the parameter server. This is done automatically if parameters with
  // the according names exist.
  callback_type_ = std::bind(&HWInterface::dynamicReconfigureCallback,
                              this,
                              std::placeholders::_1,
                              std::placeholders::_2);

  reconfig_server_ =
    std::make_shared<dynamic_reconfigure::Server<SpeedScalingConfig> >(nh);
  reconfig_server_->setCallback(callback_type_);

  const int nr_joints = joint_names_.size();
  cmd_.resize(nr_joints);
  pos_.resize(nr_joints);
  vel_.resize(nr_joints);
  eff_.resize(nr_joints);

  // Initialize and register joint state handles
  for (int i = 0; i < nr_joints; ++i)
  {
    joint_state_handles_.push_back(
      hardware_interface::JointStateHandle(joint_names_[i], &pos_[i], &vel_[i], &eff_[i]));

    jnt_state_interface_.registerHandle(joint_state_handles_[i]);
  }
  registerInterface(&jnt_state_interface_);

  // Initialize and register a Cartesian state handle
  cartesian_ros_control::CartesianStateHandle cartesian_state_handle =
    cartesian_ros_control::CartesianStateHandle(ref_frame_id_,
                                                frame_id_,
                                                &cartesian_pose_,
                                                &cartesian_twist_,
                                                &cartesian_accel_,
                                                &cartesian_jerk_);
  cart_state_interface_.registerHandle(cartesian_state_handle);
  registerInterface(&cart_state_interface_);

  // Initialize and register a Cartesian pose command handle
  cartesian_ros_control::PoseCommandHandle pose_cmd_handle =
    cartesian_ros_control::PoseCommandHandle(cartesian_state_handle, &pose_cmd_);
  pose_cmd_interface_.registerHandle(pose_cmd_handle);
  registerInterface(&pose_cmd_interface_);

  // Initialize and register joint position command handles.
  for (int i = 0; i < nr_joints; ++i)
  {
    joint_handles_.push_back(hardware_interface::JointHandle(
      jnt_state_interface_.getHandle(joint_names_[i]), &cmd_[i]));

    jnt_pos_interface_.registerHandle(joint_handles_[i]);
  }
  registerInterface(&jnt_pos_interface_);

  // Initialize and register trajectory command handles for PassThroughControllers
  hardware_interface::JointTrajectoryHandle joint_trajectory_handle =
    hardware_interface::JointTrajectoryHandle(
      &jnt_traj_cmd_,
      &jnt_traj_feedback_,
      std::bind(&HWInterface::startJointInterpolation, this, std::placeholders::_1),
      std::bind(&HWInterface::cancelJointInterpolation, this));

  jnt_traj_interface_.registerHandle(joint_trajectory_handle);
  registerInterface(&jnt_traj_interface_);

  // Initialize and register Cartesian trajectory command handles for PassThroughControllers
  hardware_interface::CartesianTrajectoryHandle cartesian_trajectory_handle =
    hardware_interface::CartesianTrajectoryHandle(
      &cart_traj_cmd_,
      &cart_traj_feedback_,
      std::bind(&HWInterface::startCartesianInterpolation, this, std::placeholders::_1),
      std::bind(&HWInterface::cancelCartesianInterpolation, this));

  cart_traj_interface_.registerHandle(cartesian_trajectory_handle);
  registerInterface(&cart_traj_interface_);

  // Initialize and register speed scaling.
  // Note: The handle's name is a convention.
  // ROS-controllers will use this name when calling getHandle().
  speedsc_interface_.registerHandle(
      hardware_interface::SpeedScalingHandle("speed_scaling_factor", &speed_scaling_));
  registerInterface(&speedsc_interface_);


  // Robot dummy communication
  joint_based_communication_ =
    std::make_unique<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> >(
      "/robot_dummy/vendor_joint_controller/follow_joint_trajectory", true);

  cartesian_based_communication_ =
    std::make_unique<actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> >(
      "/robot_dummy/vendor_cartesian_controller/follow_cartesian_trajectory", true);

  joint_based_communication_->waitForServer();
  cartesian_based_communication_->waitForServer();

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
  joint_based_communication_->sendGoal(
    trajectory,
    0, // no done callback
    0, // no active callback
    std::bind(
      &HWInterface::handleJointFeedback, this, std::placeholders::_1)); // Process feedback continuously
}

void HWInterface::startCartesianInterpolation(const hardware_interface::CartesianTrajectory& trajectory)
{
  cartesian_based_communication_->sendGoal(
    trajectory,
    0, // no done callback
    0, // no active callback
    std::bind(
      &HWInterface::handleCartesianFeedback, this, std::placeholders::_1)); // Process feedback continuously
}

void HWInterface::cancelJointInterpolation()
{
  joint_based_communication_->cancelGoal();
}

void HWInterface::cancelCartesianInterpolation()
{
  cartesian_based_communication_->cancelGoal();
}

void HWInterface::dynamicReconfigureCallback(SpeedScalingConfig& config, uint32_t level)
{
  // Let's hope for "thread safety" with fundamental types.
  speed_scaling_ = config.speed_scaling;
}

void HWInterface::handleJointFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  jnt_traj_interface_.getHandle("joint_trajectory_handle").setFeedback(*feedback);
}

void HWInterface::handleCartesianFeedback(
  const cartesian_control_msgs::FollowCartesianTrajectoryFeedbackConstPtr& feedback)
{
  cart_traj_interface_.getHandle("cartesian_trajectory_handle").setFeedback(*feedback);
}

} // namespace examples
