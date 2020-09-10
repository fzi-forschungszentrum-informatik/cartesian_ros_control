// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

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
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <array>
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Accel.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pass_through_controllers/trajectory_interface.h>
#include <cartesian_ros_control/cartesian_state_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// Other
#include <string>
#include <vector>

namespace examples {

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
   * @brief Dummy implementation for canceling a running joint interpolation on the robot
   *
   * Cancels the active goal of the JointTrajectoryController via preempt request.
   */
  void cancelJointInterpolation();

  //! Actuated joints in order from base to tip
  std::vector<std::string> m_joint_names;

  // Interfaces
  cartesian_ros_control::CartesianStateInterface m_cart_state_interface;
  hardware_interface::JointStateInterface m_jnt_state_interface;
  hardware_interface::PositionJointInterface m_jnt_pos_interface;
  hardware_interface::JointTrajectoryInterface m_jnt_traj_interface;
  hardware_interface::CartesianTrajectoryInterface m_cart_traj_interface;

  // Command buffers
  std::vector<double> m_cmd;
  std::vector<double> m_pos;
  std::vector<double> m_vel;
  std::vector<double> m_eff;
  hardware_interface::JointTrajectory m_jnt_traj_cmd;
  hardware_interface::CartesianTrajectory m_cart_traj_cmd;

  // Configuration
  std::string m_ref_frame_id;
  std::string m_frame_id;

  // States
  geometry_msgs::Pose m_cartesian_pose;
  geometry_msgs::Twist m_cartesian_twist;
  geometry_msgs::Accel m_cartesian_accel;
  geometry_msgs::Accel m_cartesian_jerk;


  // Handles
  std::vector<hardware_interface::JointHandle> m_joint_handles;
  std::vector<hardware_interface::JointStateHandle> m_joint_state_handles;

  // Robot connection and communication
  std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> >
    m_robot_communication;
  void handleFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
};

} // namespace examples
