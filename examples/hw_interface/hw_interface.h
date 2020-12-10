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
#include <ur_controllers/speed_scaling_interface.h>

// Other
#include <string>
#include <vector>
#include <array>
#include <memory>

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
  std::vector<std::string> m_joint_names;

  // Interfaces
  cartesian_ros_control::CartesianStateInterface m_cart_state_interface;
  hardware_interface::JointStateInterface m_jnt_state_interface;
  hardware_interface::PositionJointInterface m_jnt_pos_interface;
  hardware_interface::JointTrajectoryInterface m_jnt_traj_interface;
  hardware_interface::CartesianTrajectoryInterface m_cart_traj_interface;
  ur_controllers::SpeedScalingInterface m_speedsc_interface;

  // Buffers
  std::vector<double> m_cmd;
  std::vector<double> m_pos;
  std::vector<double> m_vel;
  std::vector<double> m_eff;
  hardware_interface::JointTrajectory m_jnt_traj_cmd;
  hardware_interface::JointTrajectoryFeedback m_jnt_traj_feedback;
  hardware_interface::CartesianTrajectory m_cart_traj_cmd;
  hardware_interface::CartesianTrajectoryFeedback m_cart_traj_feedback;
  double m_speed_scaling;

  // Configuration
  std::string m_ref_frame_id;
  std::string m_frame_id;

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

  std::shared_ptr<dynamic_reconfigure::Server<SpeedScalingConfig>> m_reconfig_server;
  dynamic_reconfigure::Server<SpeedScalingConfig>::CallbackType m_callback_type;

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
    m_joint_based_communication;
  void handleJointFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);

  std::unique_ptr<
    actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> >
    m_cartesian_based_communication;
  void handleCartesianFeedback(
    const cartesian_control_msgs::FollowCartesianTrajectoryFeedbackConstPtr& feedback);
};

} // namespace examples
