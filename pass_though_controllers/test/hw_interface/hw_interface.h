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
/*!\file    hw_interface.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/09/09
 *
 */
//-----------------------------------------------------------------------------

#pragma once

// ROS
#include "cartesian_control_msgs/FollowCartesianTrajectoryResult.h"
#include "control_msgs/FollowJointTrajectoryResult.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Accel.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <hardware_interface/robot_hw.h>
#include <pass_through_controllers/trajectory_interface.h>

// Joint-based control
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

// Cartesian-based control
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryFeedback.h>
#include <cartesian_interface/cartesian_command_interface.h>
#include <cartesian_interface/cartesian_state_handle.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <pass_through_controllers/SpeedScalingConfig.h>

// Speed scaling
#include <speed_scaling_interface/speed_scaling_interface.h>

// Other
#include <string>
#include <vector>
#include <array>
#include <memory>

namespace examples
{
class HWInterface : public hardware_interface::RobotHW
{
public:
  HWInterface();
  ~HWInterface();

  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

private:
  /**
   * @brief Dummy implementation for joint interpolation on the robot
   *
   * Passes this trajectory straight to a
   * JointTrajectoryController to mimic external interpolation.
   *
   * @param trajectory The trajectory blob to forward to the vendor driver
   */
  void startJointInterpolation(const hardware_interface::JointTrajectory& trajectory);

  /**
   * @brief Dummy implementation for Cartesian interpolation on the robot
   *
   * Passes this trajectory straight to a
   * CartesianTrajectoryController to mimic external interpolation.
   *
   * @param trajectory The trajectory blob to forward to the vendor driver
   */
  void startCartesianInterpolation(const hardware_interface::CartesianTrajectory& trajectory);

  /**
   * @brief Dummy implementation for canceling a running joint interpolation on the robot
   *
   * Cancels the active goal of the JointTrajectoryController via preempt request.
   */
  void cancelJointInterpolation();

  /**
   * @brief Dummy implementation for canceling a running Cartesian interpolation on the robot
   *
   * Cancels the active goal of the CartesianTrajectoryController via preempt request.
   */
  void cancelCartesianInterpolation();

  //! Actuated joints in order from base to tip
  std::vector<std::string> joint_names_;

  // Interfaces
  ros_controllers_cartesian::CartesianStateInterface cart_state_interface_;
  ros_controllers_cartesian::PoseCommandInterface pose_cmd_interface_;
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  hardware_interface::JointTrajectoryInterface jnt_traj_interface_;
  hardware_interface::CartesianTrajectoryInterface cart_traj_interface_;
  hardware_interface::SpeedScalingInterface speedsc_interface_;

  // Buffers
  std::vector<double> cmd_;
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> eff_;
  double speed_scaling_;
  geometry_msgs::Pose pose_cmd_;

  // Configuration
  std::string ref_frame_id_;
  std::string frame_id_;

  // Dynamic reconfigure
  using SpeedScalingConfig = pass_through_controllers::SpeedScalingConfig;

  /**
   * @brief Use dynamic reconfigure to mimic the driver's speed scaling
   *
   * Note: The speed scaling interface used is not made for thread safety.
   * In real driver code, that's not a problem, because robot drivers will
   * operate in synchronous read-update-write cycles. Here, we use this
   * interface under somewhat unrealistic "dynamic reconfigure" conditions for
   * testing purposes.
   *
   * @param config The speed scaling from 0 to 1.0
   * @param level Not used
   */
  void dynamicReconfigureCallback(SpeedScalingConfig& config, uint32_t level);

  std::shared_ptr<dynamic_reconfigure::Server<SpeedScalingConfig>> reconfig_server_;
  dynamic_reconfigure::Server<SpeedScalingConfig>::CallbackType callback_type_;

  // States
  geometry_msgs::Pose cartesian_pose_;
  geometry_msgs::Twist cartesian_twist_;
  geometry_msgs::Accel cartesian_accel_;
  geometry_msgs::Accel cartesian_jerk_;

  // Handles
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<hardware_interface::JointStateHandle> joint_state_handles_;

  // Robot connection and communication
  std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> joint_based_communication_;
  void handleJointFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);

  void handleJointDone(const actionlib::SimpleClientGoalState& state,
                       const control_msgs::FollowJointTrajectoryResultConstPtr& result);

  std::unique_ptr<actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction>>
      cartesian_based_communication_;
  void handleCartesianFeedback(const cartesian_control_msgs::FollowCartesianTrajectoryFeedbackConstPtr& feedback);

  void handleCartesianDone(const actionlib::SimpleClientGoalState& state,
                           const cartesian_control_msgs::FollowCartesianTrajectoryResultConstPtr& result);
};

}  // namespace examples
