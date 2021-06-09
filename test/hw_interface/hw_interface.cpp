// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

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
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/simple_goal_state.h"
#include "cartesian_interface/cartesian_command_interface.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "control_msgs/FollowJointTrajectoryResult.h"
#include "pass_through_controllers/trajectory_interface.h"
#include "ros/duration.h"
#include <functional>

namespace examples
{
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
  callback_type_ =
      std::bind(&HWInterface::dynamicReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);

  reconfig_server_ = std::make_shared<dynamic_reconfigure::Server<SpeedScalingConfig> >(nh);
  reconfig_server_->setCallback(callback_type_);

  const int nr_joints = joint_names_.size();
  cmd_.resize(nr_joints);
  pos_.resize(nr_joints);
  vel_.resize(nr_joints);
  eff_.resize(nr_joints);

  // Initialize and register joint state handles
  for (int i = 0; i < nr_joints; ++i)
  {
    joint_state_handles_.push_back(hardware_interface::JointStateHandle(joint_names_[i], &pos_[i], &vel_[i], &eff_[i]));

    jnt_state_interface_.registerHandle(joint_state_handles_[i]);
  }
  registerInterface(&jnt_state_interface_);

  // Initialize and register a Cartesian state handle
  ros_controllers_cartesian::CartesianStateHandle cartesian_state_handle =
      ros_controllers_cartesian::CartesianStateHandle(ref_frame_id_, frame_id_, &cartesian_pose_, &cartesian_twist_,
                                                      &cartesian_accel_, &cartesian_jerk_);
  cart_state_interface_.registerHandle(cartesian_state_handle);
  registerInterface(&cart_state_interface_);

  // Initialize and register a Cartesian pose command handle
  ros_controllers_cartesian::PoseCommandHandle pose_cmd_handle =
      ros_controllers_cartesian::PoseCommandHandle(cartesian_state_handle, &pose_cmd_);
  pose_cmd_interface_.registerHandle(pose_cmd_handle);
  registerInterface(&pose_cmd_interface_);

  // Initialize and register joint position command handles.
  for (int i = 0; i < nr_joints; ++i)
  {
    joint_handles_.push_back(
        hardware_interface::JointHandle(jnt_state_interface_.getHandle(joint_names_[i]), &cmd_[i]));

    jnt_pos_interface_.registerHandle(joint_handles_[i]);
  }
  registerInterface(&jnt_pos_interface_);

  // Initialize and prepare joint trajectory interface for PassThroughControllers
  jnt_traj_interface_.registerGoalCallback(
      std::bind(&HWInterface::startJointInterpolation, this, std::placeholders::_1));
  jnt_traj_interface_.registerCancelCallback(std::bind(&HWInterface::cancelJointInterpolation, this));
  registerInterface(&jnt_traj_interface_);

  // Initialize and prepare Cartesian trajectory interface for PassThroughControllers
  cart_traj_interface_.registerGoalCallback(
      std::bind(&HWInterface::startCartesianInterpolation, this, std::placeholders::_1));
  cart_traj_interface_.registerCancelCallback(std::bind(&HWInterface::cancelCartesianInterpolation, this));
  registerInterface(&cart_traj_interface_);

  // Initialize and register speed scaling.
  // Note: The handle's name is a convention.
  // ROS-controllers will use this name when calling getHandle().
  speedsc_interface_.registerHandle(hardware_interface::SpeedScalingHandle("speed_scaling_factor", &speed_scaling_));
  registerInterface(&speedsc_interface_);

  // Robot dummy communication
  joint_based_communication_ =
      std::make_unique<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> >("joint_trajectory_"
                                                                                                  "controller/"
                                                                                                  "follow_joint_"
                                                                                                  "trajectory",
                                                                                                  true);

  cartesian_based_communication_ =
      std::make_unique<actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> >("cartes"
                                                                                                                "ian_"
                                                                                                                "trajec"
                                                                                                                "tory_"
                                                                                                                "contro"
                                                                                                                "ller/"
                                                                                                                "follow"
                                                                                                                "_carte"
                                                                                                                "sian_"
                                                                                                                "trajec"
                                                                                                                "tory",
                                                                                                                true);

  if (!joint_based_communication_->waitForServer(ros::Duration(10)) ||
      !cartesian_based_communication_->waitForServer(ros::Duration(10)))
  {
    ROS_ERROR("Trajectory action interfaces of the robot dummy are not available.");
    return;
  };

  ROS_INFO("Example HW interface is ready");
}

HWInterface::~HWInterface()
{
}

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
      trajectory, std::bind(&HWInterface::handleJointDone, this, std::placeholders::_1, std::placeholders::_2),
      0,                                                                           // no active callback
      std::bind(&HWInterface::handleJointFeedback, this, std::placeholders::_1));  // Process feedback continuously
}

void HWInterface::startCartesianInterpolation(const hardware_interface::CartesianTrajectory& trajectory)
{
  cartesian_based_communication_->sendGoal(
      trajectory, std::bind(&HWInterface::handleCartesianDone, this, std::placeholders::_1, std::placeholders::_2),
      0,                                                                               // no active callback
      std::bind(&HWInterface::handleCartesianFeedback, this, std::placeholders::_1));  // Process feedback continuously
}

void HWInterface::cancelJointInterpolation()
{
  joint_based_communication_->cancelGoal();

  // For your driver implementation, you might want to wait here for the robot to
  // actually cancel the execution.

  jnt_traj_interface_.setDone(hardware_interface::ExecutionState::PREEMPTED);
}

void HWInterface::cancelCartesianInterpolation()
{
  cartesian_based_communication_->cancelGoal();

  // For your driver implementation, you might want to wait here for the robot to
  // actually cancel the execution.

  cart_traj_interface_.setDone(hardware_interface::ExecutionState::PREEMPTED);
}

void HWInterface::dynamicReconfigureCallback(SpeedScalingConfig& config, uint32_t level)
{
  // Let's hope for "thread safety" with fundamental types.
  speed_scaling_ = config.speed_scaling;
}

void HWInterface::handleJointFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  jnt_traj_interface_.setFeedback(*feedback);
}

void HWInterface::handleJointDone(const actionlib::SimpleClientGoalState& state,
                                  const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  if (state == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    jnt_traj_interface_.setDone(hardware_interface::ExecutionState::PREEMPTED);
    return;
  }

  if (result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
  {
    jnt_traj_interface_.setDone(hardware_interface::ExecutionState::SUCCESS);
    return;
  }

  jnt_traj_interface_.setDone(hardware_interface::ExecutionState::ABORTED);
}

void HWInterface::handleCartesianFeedback(
    const cartesian_control_msgs::FollowCartesianTrajectoryFeedbackConstPtr& feedback)
{
  cart_traj_interface_.setFeedback(*feedback);
}

void HWInterface::handleCartesianDone(const actionlib::SimpleClientGoalState& state,
                                      const cartesian_control_msgs::FollowCartesianTrajectoryResultConstPtr& result)
{
  if (state == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    cart_traj_interface_.setDone(hardware_interface::ExecutionState::PREEMPTED);
    return;
  }

  if (result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
  {
    cart_traj_interface_.setDone(hardware_interface::ExecutionState::SUCCESS);
    return;
  }

  cart_traj_interface_.setDone(hardware_interface::ExecutionState::ABORTED);
}

}  // namespace examples
