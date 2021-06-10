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
/*!\file    pass_through_controllers.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/10/13
 *
 */
//-----------------------------------------------------------------------------

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <pass_through_controllers/pass_through_controllers.h>

// Exports

namespace pass_through_controllers
{
using JointTrajectoryController =
    trajectory_controllers::PassThroughController<hardware_interface::JointTrajectoryInterface>;

using CartesianTrajectoryController =
    trajectory_controllers::PassThroughController<hardware_interface::CartesianTrajectoryInterface>;
}  // namespace pass_through_controllers

PLUGINLIB_EXPORT_CLASS(pass_through_controllers::JointTrajectoryController, controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(pass_through_controllers::CartesianTrajectoryController, controller_interface::ControllerBase)
