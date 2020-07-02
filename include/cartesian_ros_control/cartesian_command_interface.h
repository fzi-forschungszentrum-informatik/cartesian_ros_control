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

#include <cartesian_ros_control/cartesian_state_handle.h>

namespace cartesian_ros_control
{
class PoseCommandHandle : public CartesianStateHandle
{
public:
  PoseCommandHandle() = default;
  PoseCommandHandle(const CartesianStateHandle& state_handle, geometry_msgs::Pose* cmd)
    : CartesianStateHandle(state_handle), cmd_(cmd)
  {
    if (!cmd)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create pose command handle for frame '" +
                                                           state_handle.getName() + "'. Command data pointer is null.");
    }
  }
  virtual ~PoseCommandHandle() = default;

  void setPose(const geometry_msgs::Pose& pose)
  {
    assert(cmd_);
    *cmd_ = pose;
  }

  geometry_msgs::Pose getPose() const
  {
    assert(cmd_);
    return *cmd_;
  }
  const geometry_msgs::Pose* getPosePtr() const
  {
    assert(cmd_);
    return cmd_;
  }

private:
  geometry_msgs::Pose* cmd_ = { nullptr };
};

class TwistCommandHandle : public CartesianStateHandle
{
public:
  TwistCommandHandle() = default;
  TwistCommandHandle(const CartesianStateHandle& state_handle, geometry_msgs::Twist* cmd)
    : CartesianStateHandle(state_handle), cmd_(cmd)
  {
    if (!cmd)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create twist command handle for frame '" +
                                                           state_handle.getName() + "'. Command data pointer is null.");
    }
  }
  virtual ~TwistCommandHandle() = default;

  void setTwist(const geometry_msgs::Twist& twist)
  {
    assert(cmd_);
    *cmd_ = twist;
  }

  geometry_msgs::Twist getTwist() const
  {
    assert(cmd_);
    return *cmd_;
  }
  const geometry_msgs::Twist* getTwistPtr() const
  {
    assert(cmd_);
    return cmd_;
  }

private:
  geometry_msgs::Twist* cmd_ = { nullptr };
};

class PoseCommandInterface
  : public hardware_interface::HardwareResourceManager<PoseCommandHandle, hardware_interface::ClaimResources>
{
};

class TwistCommandInterface
  : public hardware_interface::HardwareResourceManager<TwistCommandHandle, hardware_interface::ClaimResources>
{
};
}  // namespace cartesian_ros_control
