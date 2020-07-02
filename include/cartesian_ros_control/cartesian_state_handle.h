// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author   mauch@fzi.de
 * \date    2020-07-01
 *
 */
//----------------------------------------------------------------------

#pragma once


#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace cartesian_ros_control {
class CartesianStateHandle
{
public:
  CartesianStateHandle() = default;
  CartesianStateHandle(const std::string& ref_frame_id,
                       const std::string& frame_id,
                       const geometry_msgs::Pose* pose,
                       const geometry_msgs::Twist* twist,
                       const geometry_msgs::Accel* accel,
                       const geometry_msgs::Accel* jerk)
    : frame_id_(frame_id)
    , ref_frame_id_(ref_frame_id)
    , pose_(pose)
    , twist_(twist)
    , accel_(accel)
    , jerk_(jerk)
  {
    if (!pose)
    {
      throw hardware_interface::HardwareInterfaceException(
        "Cannot create Cartesian handle for frame '" + frame_id_ + "'. Pose data pointer is null.");
    }
    if (!twist)
    {
      throw hardware_interface::HardwareInterfaceException(
        "Cannot create Cartesian handle for frame '" + frame_id_ +
        "'. Twist data pointer is null.");
    }
    if (!accel)
    {
      throw hardware_interface::HardwareInterfaceException(
        "Cannot create Cartesian handle for frame '" + frame_id_ +
        "'. Accel data pointer is null.");
    }
    if (!jerk)
    {
      throw hardware_interface::HardwareInterfaceException(
        "Cannot create Cartesian handle for frame '" + frame_id_ + "'. Jerk data pointer is null.");
    }
  }
  virtual ~CartesianStateHandle() = default;

  std::string getFrameID() const { return frame_id_; }
  geometry_msgs::Pose getPose() const
  {
    assert(pose_);
    return *pose_;
  }
  geometry_msgs::Twist getTwist() const
  {
    assert(twist_);
    return *twist_;
  }
  geometry_msgs::Accel getAccel() const
  {
    assert(accel_);
    return *accel_;
  }
  geometry_msgs::Accel getJerk() const
  {
    assert(jerk_);
    return *jerk_;
  }

private:
  std::string frame_id_;
  std::string ref_frame_id_;
  const geometry_msgs::Pose* pose_;
  const geometry_msgs::Twist* twist_;
  const geometry_msgs::Accel* accel_; // TODO: Find better datatype
  const geometry_msgs::Accel* jerk_;
};
class CartesianStateInterface
  : public hardware_interface::HardwareResourceManager<CartesianStateHandle>
{
};
} // namespace cartesian_hw_interfaec
