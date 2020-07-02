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

#include <gtest/gtest.h>

#include <cartesian_ros_control/cartesian_state_handle.h>

using namespace cartesian_ros_control;

TEST(CartesianStateHandleTest, TestConstructor)
{
  std::string reference_frame  = "base";
  std::string controlled_frame = "tool0";
  geometry_msgs::Pose pose_buffer;
  geometry_msgs::Twist twist_buffer;
  geometry_msgs::Accel accel_buffer;
  geometry_msgs::Accel jerk_buffer;

  EXPECT_NO_THROW(CartesianStateHandle obj(
    reference_frame, controlled_frame, &pose_buffer, &twist_buffer, &accel_buffer, &jerk_buffer));
  EXPECT_THROW(
    CartesianStateHandle obj(
      reference_frame, controlled_frame, nullptr, &twist_buffer, &accel_buffer, &jerk_buffer),
    hardware_interface::HardwareInterfaceException);
  EXPECT_THROW(
    CartesianStateHandle obj(
      reference_frame, controlled_frame, &pose_buffer, nullptr, &accel_buffer, &jerk_buffer),
    hardware_interface::HardwareInterfaceException);
  EXPECT_THROW(
    CartesianStateHandle obj(
      reference_frame, controlled_frame, &pose_buffer, &twist_buffer, nullptr, &jerk_buffer),
    hardware_interface::HardwareInterfaceException);
  EXPECT_THROW(
    CartesianStateHandle obj(
      reference_frame, controlled_frame, &pose_buffer, &twist_buffer, &accel_buffer, nullptr),
    hardware_interface::HardwareInterfaceException);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
