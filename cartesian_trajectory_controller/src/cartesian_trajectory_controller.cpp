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
/*!\file    cartesian_trajectory_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/24
 *
 */
//-----------------------------------------------------------------------------

#include <pluginlib/class_list_macros.h>
#include <cartesian_trajectory_controller/cartesian_trajectory_controller.h>
#include <cartesian_interface/cartesian_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include "hardware_interface/joint_state_interface.h"

namespace pose_controllers
{
  using CartesianTrajectoryController =
    cartesian_trajectory_controller::CartesianTrajectoryController<cartesian_ros_control::PoseCommandInterface>;
}

namespace twist_controllers
{
  using CartesianTrajectoryController =
    cartesian_trajectory_controller::CartesianTrajectoryController<cartesian_ros_control::TwistCommandInterface>;
}

namespace position_controllers
{
  using CartesianTrajectoryController =
    cartesian_trajectory_controller::CartesianTrajectoryController<hardware_interface::PositionJointInterface>;
}

namespace velocity_controllers
{
  using CartesianTrajectoryController =
    cartesian_trajectory_controller::CartesianTrajectoryController<hardware_interface::VelocityJointInterface>;
}

namespace cartesian_trajectory_publisher
{
  using CartesianTrajectoryPublisher =
    cartesian_trajectory_controller::CartesianTrajectoryController<hardware_interface::JointStateInterface>;
}



PLUGINLIB_EXPORT_CLASS(pose_controllers::CartesianTrajectoryController,
                       controller_interface::ControllerBase)

/* Not yet implemented.
PLUGINLIB_EXPORT_CLASS(twist_controllers::CartesianTrajectoryController,
                       controller_interface::ControllerBase)
*/

PLUGINLIB_EXPORT_CLASS(position_controllers::CartesianTrajectoryController,
                       controller_interface::ControllerBase)
/* Not yet implemented
PLUGINLIB_EXPORT_CLASS(velocity_controllers::CartesianTrajectoryController,
                       controller_interface::ControllerBase)
*/

PLUGINLIB_EXPORT_CLASS(cartesian_trajectory_publisher::CartesianTrajectoryPublisher,
                       controller_interface::ControllerBase)
