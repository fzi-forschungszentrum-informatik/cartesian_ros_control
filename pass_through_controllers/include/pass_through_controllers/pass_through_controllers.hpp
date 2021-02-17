////////////////////////////////////////////////////////////////////////////////
// Copyright 2021 FZI Research Center for Information Technology
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
/*!\file    pass_through_controllers.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/16
 *
 */
//-----------------------------------------------------------------------------

// Project
#include <pass_through_controllers/trajectory_interface.h>
#include <pass_through_controllers/pass_through_controllers.h>

// Other
#include "ros/duration.h"
#include "ros/timer.h"
#include "cartesian_interface/speed_scaling_interface.h"
#include <sstream>
#include <string>

namespace trajectory_controllers {

  template <class TrajectoryInterface>
  bool PassThroughController<TrajectoryInterface>::init(hardware_interface::RobotHW* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh)
  {
    // Get names of joints from the parameter server
    if (!controller_nh.getParam("joints", joint_names_))
    {
      ROS_ERROR_STREAM("Failed to load " << controller_nh.getNamespace()
                                         << "/joints from parameter server");
      return false;
    }

    // Availability checked by MultiInterfaceController
    auto traj_interface = hw->get<TrajectoryInterface>();
    if (!traj_interface)
    {
      ROS_ERROR_STREAM(controller_nh.getNamespace() << ": No suitable trajectory interface found.");
      return false;
    }

    try
    {
      traj_interface->setResources(joint_names_);
      trajectory_handle_ = std::make_unique<typename Base::TrajectoryHandle>(
          traj_interface->getHandle(Base::TrajectoryHandle::getName()));
    }
    catch (hardware_interface::HardwareInterfaceException& ex)
    {
      ROS_ERROR_STREAM(
          controller_nh.getNamespace() << ": Exception getting trajectory handle from interface: "
          << ex.what());
      return false;
    }

    // Use speed scaling interface if available (optional).
    auto speed_scaling_interface = hw->get<hardware_interface::SpeedScalingInterface>();
    if (!speed_scaling_interface)
    {
      ROS_INFO_STREAM(
        controller_nh.getNamespace()
        << ": Your RobotHW seems not to provide speed scaling. Starting without this feature.");
      speed_scaling_ = nullptr;
    }
    else
    {
      speed_scaling_ = std::make_unique<hardware_interface::SpeedScalingHandle>(
        speed_scaling_interface->getHandle("speed_scaling_factor"));
    }


    // Action server
    action_server_.reset(new actionlib::SimpleActionServer<typename Base::FollowTrajectoryAction>(
      controller_nh,
      std::is_same<TrajectoryInterface, hardware_interface::JointTrajectoryInterface>::value
        ? "follow_joint_trajectory"
        : "follow_cartesian_trajectory",
      std::bind(&PassThroughController::executeCB, this, std::placeholders::_1),
      false));

    // This is the cleanest method to notify the vendor robot driver of
    // preempted requests.
    action_server_->registerPreemptCallback(
        std::bind(&PassThroughController::preemptCB, this));

    action_server_->start();

    return true;
  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::starting(const ros::Time& time)
  {
    done_ = true;  // wait with update() until the next goal comes in.
  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::stopping(const ros::Time& time)
  {
    if (action_server_->isActive())
    {
      // Set canceled flag in the action result
      action_server_->setPreempted();

      // Stop trajectory interpolation on the robot
      trajectory_handle_->cancelCommand();
    }

  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::update(const ros::Time& time, const ros::Duration& period)
  {
    if (action_server_->isActive() && !done_)
    {
      // Measure action duration and apply speed scaling if available.
      const double factor = (speed_scaling_) ? *speed_scaling_->getScalingFactor() : 1.0;
      action_duration_.current += period * factor;

      typename Base::TrajectoryFeedback f = trajectory_handle_->getFeedback();
      action_server_->publishFeedback(f);

      // Check tolerances on each call and set terminal conditions for the
      // action server if special criteria are met.
      // Also set the done_ flag once that happens.
      monitorExecution(f);

      // Time is up. Check goal tolerances and set terminal state.
      if (action_duration_.current >= action_duration_.target)
      {
        timesUp();
      }
    }
  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::executeCB(const typename Base::GoalConstPtr& goal)
  {
    // Upon entering this callback, the simple action server has already
    // preempted the previously active goal (if any) and has accepted the new goal.

    if (!this->isRunning())
    {
      ROS_ERROR("Can't accept new action goals. Controller is not running.");
      typename Base::FollowTrajectoryResult result;
      result.error_code = Base::FollowTrajectoryResult::INVALID_GOAL;
      action_server_->setAborted(result);
      return;
    }

    // Only allow correct tolerances if given
    if (!isValid(goal))
    {
      return;
    }
    path_tolerances_ = goal->path_tolerance;
    goal_tolerances_ = goal->goal_tolerance;
    
    // Notify the  vendor robot control.
    trajectory_handle_->setCommand(*goal);

    // Time keeping
    action_duration_.current = ros::Duration(0.0);
    action_duration_.target =
      goal->trajectory.points.back().time_from_start + goal->goal_time_tolerance;

    done_ = false;
    while (!done_)
    {
      ros::Duration(0.01).sleep();
    }

    // When done, the action server is in one of the three states:
    // 1) succeeded: managed in timesUp()
    // 2) aborted: managed in update() or in timesUp()
    // 3) preempted: managed in preemptCB()
  }


  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::preemptCB()
  {
    // Notify the vendor robot control.
    trajectory_handle_->cancelCommand();

    typename Base::FollowTrajectoryResult result;
    result.error_string = "preempted";
    action_server_->setPreempted(result);

    done_ = true;
  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::monitorExecution(
    const typename Base::TrajectoryFeedback& feedback)
  {
    // Abort if any of the joints exceeds its path tolerance
    if (!withinTolerances(feedback.error, path_tolerances_))
    {
      typename Base::FollowTrajectoryResult result;
      result.error_code = Base::FollowTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      action_server_->setAborted(result);
      done_ = true;
      return;
    }
  }

  template <>
  bool PassThroughController<hardware_interface::JointTrajectoryInterface>::withinTolerances(
    const TrajectoryPoint& error,
    const Tolerance& tolerances)
  {
    // Precondition
    if (!tolerances.empty())
    {
      assert(error.positions.size() == tolerances.size() &&
          error.velocities.size() == tolerances.size() &&
          error.accelerations.size() == tolerances.size());
    }

    for (size_t i = 0; i < tolerances.size(); ++i)
    {
      // TODO: Velocity and acceleration limits will be affected by speed scaling.
      // Address this once we know more edge cases during beta testing.
      if ((path_tolerances_[i].position > 0.0 &&
           std::abs(error.positions[i]) > path_tolerances_[i].position) ||
          (path_tolerances_[i].velocity > 0.0 &&
           std::abs(error.velocities[i]) > path_tolerances_[i].velocity) ||
          (path_tolerances_[i].acceleration > 0.0 &&
           std::abs(error.accelerations[i]) > path_tolerances_[i].acceleration))
      {
        return false;
      }
    }
    return true;
  }

  template <>
  bool PassThroughController<hardware_interface::CartesianTrajectoryInterface>::withinTolerances(
    const typename Base::TrajectoryPoint& error, const typename Base::Tolerance& tolerances)
  {
    // Every error is ok for uninitialized tolerances
    Base::Tolerance uninitialized;
    std::stringstream str_1;
    std::stringstream str_2;
    str_1 << tolerances;
    str_2 << uninitialized;

    if (str_1.str() == str_2.str())
    {
      return true;
    }

    auto not_within_limits = [](auto& a, auto& b)
    {
      return a.x > b.x || a.y > b.y || a.z > b.z;
    };

    // Check each individual dimension separately.
    if (not_within_limits(error.pose.position, tolerances.position_error) ||
        not_within_limits(error.pose.orientation, tolerances.orientation_error) ||
        not_within_limits(error.twist.linear, tolerances.twist_error.linear) ||
        not_within_limits(error.twist.angular, tolerances.twist_error.angular) ||
        not_within_limits(error.acceleration.linear, tolerances.acceleration_error.linear) ||
        not_within_limits(error.acceleration.angular, tolerances.acceleration_error.angular))
    {
      return false;
    }

    return true;
  }

  template <>
  bool PassThroughController<hardware_interface::JointTrajectoryInterface>::isValid(
    const typename Base::GoalConstPtr& goal)
  {
    // If tolerances are given, they must be given for all joints.
    if ((!goal->path_tolerance.empty() && goal->path_tolerance.size() != joint_names_.size()) ||
        (!goal->goal_tolerance.empty() && goal->goal_tolerance.size() != joint_names_.size()))
    {
      ROS_ERROR("Given tolerances must match the number of joints");
      typename Base::FollowTrajectoryResult result;
      result.error_code = Base::FollowTrajectoryResult::INVALID_GOAL;
      action_server_->setAborted(result);
      return false;
    }
    return true;
  }

  template <>
  bool PassThroughController<hardware_interface::CartesianTrajectoryInterface>::isValid(
    const typename Base::GoalConstPtr& goal)
  {
    // No plausibility check required. All possible user inputs seem fine for now.
    return true;
  }

  template <class TrajectoryInterface>
  void PassThroughController<TrajectoryInterface>::timesUp()
  {
    // Use most recent feedback
    typename Base::TrajectoryPoint p = trajectory_handle_->getFeedback().error;
    typename Base::FollowTrajectoryResult result;

    // Abort if any of the joints exceeds its goal tolerance
    if (!withinTolerances(p, goal_tolerances_))
    {
      result.error_code = Base::FollowTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
      action_server_->setAborted(result);

      trajectory_handle_->cancelCommand();
    }
    else // Succeed
    {
      result.error_code = Base::FollowTrajectoryResult::SUCCESSFUL;
      action_server_->setSucceeded(result);
    }

    done_ = true;
  }


  } // namespace trajectory_controllers
