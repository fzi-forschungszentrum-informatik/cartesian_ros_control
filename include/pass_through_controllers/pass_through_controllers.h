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
/*!\file    pass_through_controllers.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/16
 *
 */
//-----------------------------------------------------------------------------

#pragma once

// ROS control
#include <speed_scaling_interface/speed_scaling_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <pass_through_controllers/trajectory_interface.h>

// Joint-based
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/JointTolerance.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// Cartesian
#include <cartesian_control_msgs/CartesianTolerance.h>
#include <cartesian_control_msgs/CartesianTrajectoryPoint.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryGoal.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>

// Other
#include <actionlib/server/simple_action_server.h>
#include <vector>
#include <memory>
#include <atomic>

namespace trajectory_controllers
{
struct JointBase
{
  using Tolerance = std::vector<control_msgs::JointTolerance>;
  using TrajectoryPoint = trajectory_msgs::JointTrajectoryPoint;
  using TrajectoryFeedback = hardware_interface::JointTrajectoryFeedback;
  using FollowTrajectoryAction = control_msgs::FollowJointTrajectoryAction;
  using FollowTrajectoryResult = control_msgs::FollowJointTrajectoryResult;
  using GoalConstPtr = control_msgs::FollowJointTrajectoryGoalConstPtr;
};

struct CartesianBase
{
  using Tolerance = cartesian_control_msgs::CartesianTolerance;
  using TrajectoryPoint = cartesian_control_msgs::CartesianTrajectoryPoint;
  using TrajectoryFeedback = hardware_interface::CartesianTrajectoryFeedback;
  using FollowTrajectoryAction = cartesian_control_msgs::FollowCartesianTrajectoryAction;
  using FollowTrajectoryResult = cartesian_control_msgs::FollowCartesianTrajectoryResult;
  using GoalConstPtr = cartesian_control_msgs::FollowCartesianTrajectoryGoalConstPtr;
};

/**
 * @brief A ROS controller for forwarding trajectories to a robot for interpolation
 *
 * Instead of interpolating between the waypoints itself, this driver passes
 * the complete trajectories down to the according HW interfaces, assuming that
 * the driver's implementation makes use of whichever components are suitable
 * for that specific robot.
 *
 * This controller implements a simple action server that provides the common
 * /follow_joint_trajectory or /follow_cartesian_trajectory action interface.
 *
 * Users specify this controller in their .yaml files with:
 *
 * \code{.yaml}
 * # Your joint-based passthrough controller
 * forward_joint_trajectories:
 *     type: "pass_through_controllers/JointTrajectoryController"
 *     ...
 *
 *
 * # Your Cartesian passthrough controller
 * forward_cartesian_trajectories:
 *     type: "pass_through_controllers/CartesianTrajectoryController"
 *     ...
 * \endcode
 *
 * @tparam TrajectoryInterface The type of trajectory interface used for this
 * controller. Either hardware_interface::JointTrajectoryInterface or
 * hardware_interface::CartesianTrajectoryInterface.
 */
template <class TrajectoryInterface>
class PassThroughController
  : public controller_interface::MultiInterfaceController<TrajectoryInterface,
                                                          hardware_interface::SpeedScalingInterface>,
    public std::conditional<std::is_same<TrajectoryInterface, hardware_interface::JointTrajectoryInterface>::value,
                            JointBase, CartesianBase>::type
{
public:
  PassThroughController()
    : controller_interface::MultiInterfaceController<TrajectoryInterface,
                                                     hardware_interface::SpeedScalingInterface>(
          true)  // Make speed scaling optional
  {
  }

  // Alias for full qualifications of inherited types.
  // This enables a compact definition of member functions for both joint-based
  // and Cartesian-based PassThroughControllers.
  using Base =
      typename std::conditional<std::is_same<TrajectoryInterface, hardware_interface::JointTrajectoryInterface>::value,
                                JointBase, CartesianBase>::type;

  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * @brief Callback method for new action goals
   *
   * This method calls the \a setGoal() method from the TrajectoryInterface.
   * Implementers of the ROS-control HW can choose how the trajectory goal is
   * forwarded to the robot for interpolation.
   *
   * Things in detail:
   * - New goals are only accepted when the controller is running
   * - Switching the controller (stopping) cancels running goals
   * - The goal's success is monitored in update()
   *
   * Further info on how the simple action server works is given
   * <a href="http://wiki.ros.org/actionlib/DetailedDescription">here</a>.
   *
   * @param goal The trajectory goal
   */
  void executeCB(const typename Base::GoalConstPtr& goal);

  /**
   * @brief Callback method for preempt requests
   *
   * This method gets called on every preempt request that happens either
   * directly upon a client request or indirectly when receiving a new goal
   * while another is still active.
   *
   * This method calls the \a setCancel() method from the TrajectoryInterface.
   * The RobotHW should implement how this notification is handled by the robot
   * vendor control.
   *
   * Also check
   * <a href="https://answers.ros.org/question/333316/actionlib-preempt-vs-cancel/">this info</a>.
   * on the simple action server's preemption policy:
   */
  void preemptCB();

private:
  /**
   * @brief Container for easy time management
   *
   */
  struct ActionDuration
  {
    ActionDuration() : target(0.0), current(0.0)
    {
    }

    ros::Duration target;   ///< Target duration of the current action.
    ros::Duration current;  ///< Real duration of the current action.
  };

  /**
   * @brief  Monitor the trajectory execution
   *
   * @param feedback The feedback to use for evaluating tolerances
   */
  void monitorExecution(const typename Base::TrajectoryFeedback& feedback);

  /**
   * @brief Check if tolerances are met
   *
   * @param error The error to check
   * @param tolerances The tolerances to check against
   *
   * @return False if any of the errors exceeds its tolerance, else true
   */
  bool withinTolerances(const typename Base::TrajectoryPoint& error, const typename Base::Tolerance& tolerances);

  /**
   * @brief Check if follow trajectory goals are valid
   *
   * @param goal The goal to check.
   *
   * @return True if goal is valid, false otherwise
   */
  bool isValid(const typename Base::GoalConstPtr& goal);

  /**
   * @brief Will get called upon finishing the forwarded trajectory
   */
  void doneCB(const hardware_interface::ExecutionState& state);

  std::atomic<bool> done_;
  ActionDuration action_duration_;
  std::unique_ptr<hardware_interface::SpeedScalingHandle> speed_scaling_;
  std::vector<std::string> joint_names_;
  typename Base::Tolerance path_tolerances_;
  typename Base::Tolerance goal_tolerances_;
  TrajectoryInterface* trajectory_interface_;  ///* Resource managed by RobotHW
  std::unique_ptr<actionlib::SimpleActionServer<typename Base::FollowTrajectoryAction> > action_server_;
};

}  // namespace trajectory_controllers

#include <pass_through_controllers/pass_through_controllers.hpp>
