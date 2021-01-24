////////////////////////////////////////////////////////////////////////////////
// Copyright 2020 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    control_policies.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/24
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <ur_controllers/speed_scaling_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <cartesian_interface/cartesian_command_interface.h>
#include <cartesian_trajectory_interpolation/cartesian_trajectory.h>
#include "cartesian_trajectory_interpolation/cartesian_trajectory_segment.h"

namespace cartesian_ros_control
{

  /**
   * @brief A common control type with optional speed scaling interface
   *
   * @tparam HWInterface Hardware interface essential for control
   */
  template <class HWInterface>
    using Controller = controller_interface::MultiInterfaceController<HWInterface, ur_controllers::SpeedScalingInterface>;


  /**
   * @brief Primary template
   *
   * Fail compilation for all non-specialized control policies.
   *
   * @tparam HWInterface Hardware interface essential for control.
   */
  template <class HWInterface>
    class ControlPolicy;


  /**
   * @brief Specialization for Cartesian pose control
   */
  template <>
    class ControlPolicy<cartesian_ros_control::PoseCommandInterface>
    : public Controller<cartesian_ros_control::PoseCommandInterface>
    {

      public:
        ControlPolicy()
          : Controller<cartesian_ros_control::PoseCommandInterface>(true)  // optional speedscaling
        {
        };

        /**
         * @brief TODO:
         *
         * @param cmd Desired Cartesian state of the manipulator
         */
        void updateCommand(const cartesian_ros_control::CartesianState& cmd);
    };


  /**
   * @brief Specialization for Cartesian twist control
   */
  template <>
    class ControlPolicy<cartesian_ros_control::TwistCommandInterface>
    : public Controller<cartesian_ros_control::TwistCommandInterface>
    {

      public:
        ControlPolicy()
          : Controller<cartesian_ros_control::TwistCommandInterface>(true)  // optional speedscaling
        {
        };

        /**
         * @brief TODO:
         *
         * @param cmd Desired Cartesian state of the manipulator
         */
        void updateCommand(const cartesian_ros_control::CartesianState& cmd);
    };


  /**
   * @brief Specialization for joint position control
   */
  template <>
    class ControlPolicy<hardware_interface::PositionJointInterface>
    : public Controller<hardware_interface::PositionJointInterface>
    {

      public:
        ControlPolicy()
          : Controller<hardware_interface::PositionJointInterface>(true)  // optional speedscaling
        {
        };

        /**
         * @brief TODO:
         *
         * Uses KDL's Levenberg-Marquardt Inverse Kinematics solver for mapping
         * poses to joint positions.
         *
         * @param cmd Desired Cartesian state of the manipulator
         */
        void updateCommand(const cartesian_ros_control::CartesianState& cmd);
    };


  /**
   * @brief Specialization for joint velocity control
   */
  template <>
    class ControlPolicy<hardware_interface::VelocityJointInterface>
    : public Controller<hardware_interface::VelocityJointInterface>
    {

      public:
        ControlPolicy()
          : Controller<hardware_interface::VelocityJointInterface>(true)  // optional speedscaling
        {
        };

        /**
         * @brief TODO:
         *
         * Uses KDL's weighted DLS method for mapping twists to joint velocitiies.
         *
         * @param cmd Desired Cartesian state of the manipulator
         */
        void updateCommand(const cartesian_ros_control::CartesianState& cmd);
    };



}

#include <cartesian_trajectory_controller/control_policies.hpp>
