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
/*!\file    trajectory_interface.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/15
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <functional>
#include <cartesian_control_msgs/FollowCartesianTrajectoryGoal.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryResult.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <vector>
#include <hardware_interface/hardware_interface.h>
#include <ros/ros.h>

namespace hardware_interface
{
/**
 * @brief Hardware-generic done flags for trajectory execution
 *
 * When forwarding trajectories to robots, we hand-over control of the actual
 * execution.  This is a minimal set of generic flags to be reported by the
 * hardware that PassThroughControllers can process and react upon.
 */
enum class ExecutionState
{
  SUCCESS = 0,
  PREEMPTED = -1,
  ABORTED = -2,
};

/**
 * @brief TrajectoryType for joint-based trajectories
 */
using JointTrajectory = control_msgs::FollowJointTrajectoryGoal;

/**
 * @brief TrajectoryType for Cartesian trajectories
 */
using CartesianTrajectory = cartesian_control_msgs::FollowCartesianTrajectoryGoal;

/**
 * @brief FeedbackType for joint-based trajectories
 */
using JointTrajectoryFeedback = control_msgs::FollowJointTrajectoryFeedback;

/**
 * @brief FeedbackType for Cartesian trajectories
 */
using CartesianTrajectoryFeedback = cartesian_control_msgs::FollowCartesianTrajectoryFeedback;

/**
 * @brief Hardware interface for forwarding trajectories
 *
 * This special hardware interface is primarily used by PassThroughControllers,
 * which forward full trajectories to robots for interpolation. In contrast to
 * other hardware interfaces, this interface does not provide the usual handle
 * mechanism.  Instead, callbacks and respective setter methods allow to
 * implement control flow during trajectory execution.  Resources need to be
 * claimed in the forwarding controller's constructor with \a setResources().
 *
 * The intended usage is as follows:
 * - The RobotHW instantiates this TrajectoryInterface and registers callbacks
 *   for forwarded goals and cancel requests with \a registerGoalCallback() and
 *   \a registerCancelCallback(), respectively.
 * - On the controller side, the PassThroughController calls the respective \a
 *   setGoal() and \a setCancel() methods to trigger those events for the
 *   RobotHW. The PassThroughController also registers a callback for signals
 *   from the RobotHW when trajectory execution is done with \a
 *   registerDoneCallback().
 * - When done with the trajectory execution, the RobotHW calls \a setDone() to
 *   inform the PassThroughController via its registered callback.
 * - The RobotHW gives feedback continuously from the execution with \a
 *   setFeedback(), which is used by the PassThroughController via \a
 *   getFeedback().
 *
 * @tparam TrajectoryType Distinguish between joint-based and Cartesian trajectories
 * @tparam FeedbackType The type of feedback for the trajectory used.
 */
template <class TrajectoryType, class FeedbackType>
class TrajectoryInterface : public hardware_interface::HardwareInterface
{
public:
  /**
   * @brief Register a RobotHW-side callback for new trajectory execution
   *
   * Callback for triggering execution start in the RobotHW.
   * Use this callback mechanism to handle starting of
   * trajectories on the robot vendor controller.
   *
   * @param f The function to be called for starting trajectory execution
   */
  void registerGoalCallback(std::function<void(const TrajectoryType&)> f)
  {
    goal_callback_ = f;
  }

  /**
   * @brief Register a RobotHW-side callback for canceling requests
   *
   * @param f The function to be called for canceling trajectory execution
   */
  void registerCancelCallback(std::function<void()> f)
  {
    cancel_callback_ = f;
  }

  /**
   * @brief Register a Controller-side callback for done signals from the hardware
   *
   * Use this mechanism in the PassThroughController for handling the
   * ExecutionState of the forwarded trajectory.
   *
   * @param f The function to be called when trajectory execution is done
   */
  void registerDoneCallback(std::function<void(const ExecutionState&)> f)
  {
    done_callback_ = f;
  }

  /**
   * @brief Start the forwarding of new trajectories
   *
   * Controller-side method to send incoming trajectories to the RobotHW.
   *
   * Note: The JointTrajectory type reorders the joints according to the
   * given joint resource list.
   *
   * @param command The new trajectory
   * @return True if goal is feasible, false otherwise
   */
  bool setGoal(TrajectoryType goal);

  /**
   * @brief Cancel the current trajectory execution
   *
   * Controller-side method to cancel current trajectory execution on the robot.
   */
  void setCancel()
  {
    if (cancel_callback_ != nullptr)
      cancel_callback_();
  };

  /**
   * @brief RobotHW-side method to mark the execution of a trajectory done.
   *
   * Call this function when done with a forwarded trajectory in your
   * RobotHW or when unexpected interruptions occur. The
   * PassThroughController will implement a callback to set appropriate
   * result flags for the trajectory action clients.
   *
   * @param state The final state after trajectory execution
   */
  void setDone(const ExecutionState& state)
  {
    if (done_callback_ != nullptr)
      done_callback_(state);
  }

  /**
   * @brief Set trajectory feedback for PassThroughControllers
   *
   * This should be used by the RobotHW to provide continuous feedback on
   * trajectory execution for the PassThroughControllers.
   *
   * @param feedback The feedback content to write to the interface
   */
  void setFeedback(FeedbackType feedback)
  {
    feedback_ = feedback;
  }

  /**
   * @brief Get trajectory feedback
   *
   * This can be used by PassThroughControllers to continuously poll
   * trajectory feedback from the hardware interface.
   *
   * @return The most recent feedback on the current trajectory execution
   */
  FeedbackType getFeedback() const
  {
    return feedback_;
  }

  /**
   * @brief Get the joint names (resources) associated with this interface.
   *
   * @return Joint names
   */
  std::vector<std::string> getJointNames() const
  {
    return joint_names_;
  }

  /**
   * @brief Associate resources with this interface
   *
   * \Note: Call this inside the PassThroughController's constructor to
   * manage resource conflicts with other ROS controllers.
   *
   * @param resources A list of resource names
   */
  void setResources(std::vector<std::string> resources)
  {
    for (const std::string& joint : resources)
    {
      hardware_interface::HardwareInterface::claim(joint);
    }
    joint_names_ = resources;
  }

private:
  std::function<void(const TrajectoryType&)> goal_callback_;
  std::function<void()> cancel_callback_;
  std::function<void(const ExecutionState&)> done_callback_;
  FeedbackType feedback_;
  std::vector<std::string> joint_names_;
};

// Full spezialization for JointTrajectory
template <>
inline bool TrajectoryInterface<JointTrajectory, JointTrajectoryFeedback>::setGoal(JointTrajectory goal)
{
  control_msgs::FollowJointTrajectoryGoal tmp = goal;

  // Respect joint order by computing the map between msg indices to expected indices.
  // If msg is {A, C, D, B} and expected is {A, B, C, D}, the associated mapping vector is {0, 2, 3, 1}
  auto msg = goal.trajectory.joint_names;
  auto expected = joint_names_;
  std::vector<size_t> msg_joints(msg.size());
  if (msg.size() != expected.size())
  {
    // msg must contain all joint names.
    ROS_WARN("Not forwarding trajectory. It contains wrong number of joints");
    return false;
  }
  for (auto msg_it = msg.begin(); msg_it != msg.end(); ++msg_it)
  {
    auto expected_it = std::find(expected.begin(), expected.end(), *msg_it);
    if (expected.end() == expected_it)
    {
      ROS_WARN_STREAM("Not forwarding trajectory. It contains at least one unexpected joint name: " << *msg_it);
      return false;
    }
    else
    {
      const size_t msg_dist = std::distance(msg.begin(), msg_it);
      const size_t expected_dist = std::distance(expected.begin(), expected_it);
      msg_joints[msg_dist] = expected_dist;
    }
  }

  // Reorder the joint names and data fields in all trajectory points
  tmp.trajectory.joint_names = expected;
  tmp.trajectory.points.clear();

  for (auto point : goal.trajectory.points)
  {
    trajectory_msgs::JointTrajectoryPoint p{ point };  // init for equal data size

    for (size_t i = 0; i < expected.size(); ++i)
    {
      auto jnt_id = msg_joints[i];

      if (point.positions.size() == expected.size())
        p.positions[jnt_id] = point.positions[i];
      if (point.velocities.size() == expected.size())
        p.velocities[jnt_id] = point.velocities[i];
      if (point.accelerations.size() == expected.size())
        p.accelerations[jnt_id] = point.accelerations[i];
      if (point.effort.size() == expected.size())
        p.effort[jnt_id] = point.effort[i];
    }
    tmp.trajectory.points.push_back(p);
  }

  if (goal_callback_ != nullptr)
  {
    goal_callback_(tmp);
    return true;
  }
  return false;
}

// Full spezialization for CartesianTrajectory
template <>
inline bool TrajectoryInterface<CartesianTrajectory, CartesianTrajectoryFeedback>::setGoal(CartesianTrajectory goal)
{
  if (goal_callback_ != nullptr)
  {
    goal_callback_(goal);
    return true;
  }
  return false;
}

/**
 * @brief Hardware interface for commanding (forwarding) joint-based trajectories
 */
using JointTrajectoryInterface = TrajectoryInterface<JointTrajectory, JointTrajectoryFeedback>;

/**
 * @brief Hardware interface for commanding (forwarding) Cartesian trajectories
 */
using CartesianTrajectoryInterface = TrajectoryInterface<CartesianTrajectory, CartesianTrajectoryFeedback>;

}  // namespace hardware_interface
