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

#include <cartesian_ros_control/twist_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace cartesian_ros_control
{
bool TwistController::init(TwistCommandInterface* hw, ros::NodeHandle& n)
{
  std::string frame_id;
  if (!n.getParam("frame_id", frame_id))
  {
    ROS_ERROR_STREAM("Required parameter " << n.resolveName("frame_id") << " not given");
    return false;
  }

  handle_ = hw->getHandle(frame_id);
  twist_sub_ = n.subscribe<geometry_msgs::Twist>("command", 1, &TwistController::twistCallback, this);

  std::vector<std::string> joint_names;
  if (!n.getParam("joints", joint_names))
  {
    ROS_ERROR_STREAM("Failed to read required parameter '" << n.resolveName("joints") << ".");
    return false;
  }

  for (auto& name : joint_names)
  {
    hw->claim(name);
  }

  return true;
}

void TwistController::starting(const ros::Time& time)
{
  geometry_msgs::Twist twist;
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
  command_buffer_.writeFromNonRT(twist);
}

void TwistController::twistCallback(const geometry_msgs::TwistConstPtr& msg)
{
  geometry_msgs::Twist twist;
  twist.linear.x = gain_ * msg->linear.x;
  twist.linear.y = gain_ * msg->linear.y;
  twist.linear.z = gain_ * msg->linear.z;
  twist.angular.x = gain_ * msg->angular.x;
  twist.angular.y = gain_ * msg->angular.y;
  twist.angular.z = gain_ * msg->angular.z;
  command_buffer_.writeFromNonRT(twist);
}
}  // namespace cartesian_ros_control

PLUGINLIB_EXPORT_CLASS(cartesian_ros_control::TwistController, controller_interface::ControllerBase)
