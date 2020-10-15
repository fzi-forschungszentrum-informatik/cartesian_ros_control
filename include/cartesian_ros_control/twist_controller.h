// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-07-02
 *
 */
//----------------------------------------------------------------------

#pragma once

#include <controller_interface/controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <realtime_tools/realtime_buffer.h>

#include <cartesian_ros_control/cartesian_command_interface.h>

namespace cartesian_ros_control
{

/**
 * @brief A Cartesian ROS-controller for commanding target twists to a robot
 *
 * This controller makes use of a TwistCommandInterface to set a user specified
 * twist message as reference for robot control.
 * The according hardware_interface::RobotHW can send these commands
 * directly to the robot driver in its write() function.
 */
class TwistController : public controller_interface::Controller<TwistCommandInterface>
{
public:
  TwistController() = default;
  virtual ~TwistController() = default;

  virtual bool init(TwistCommandInterface* hw, ros::NodeHandle& n) override;

  virtual void starting(const ros::Time& time) override;

  virtual void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) override
  {
    handle_.setTwist(*command_buffer_.readFromRT());
  }

  TwistCommandHandle handle_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> command_buffer_;

private:
  ros::Subscriber twist_sub_;
  void twistCallback(const geometry_msgs::TwistConstPtr& msg);
  double gain_ = { 0.1 };
};

}  // namespace cartesian_ros_control
