// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

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
#include "ur_controllers/speed_scaling_interface.h"
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



namespace trajectory_controllers {


struct JointBase
{
  using Tolerance              = std::vector<control_msgs::JointTolerance>;
  using TrajectoryPoint        = trajectory_msgs::JointTrajectoryPoint;
  using TrajectoryFeedback     = hardware_interface::JointTrajectoryFeedback; 
  using TrajectoryHandle       = hardware_interface::JointTrajectoryHandle;
  using FollowTrajectoryAction = control_msgs::FollowJointTrajectoryAction;
  using FollowTrajectoryResult = control_msgs::FollowJointTrajectoryResult;
  using GoalConstPtr           = control_msgs::FollowJointTrajectoryGoalConstPtr;
};

struct CartesianBase
{
  using Tolerance              = cartesian_control_msgs::CartesianTolerance;
  using TrajectoryPoint        = cartesian_control_msgs::CartesianTrajectoryPoint;
  using TrajectoryFeedback     = hardware_interface::CartesianTrajectoryFeedback;
  using TrajectoryHandle       = hardware_interface::CartesianTrajectoryHandle;
  using FollowTrajectoryAction = cartesian_control_msgs::FollowCartesianTrajectoryAction;
  using FollowTrajectoryResult = cartesian_control_msgs::FollowCartesianTrajectoryResult;
  using GoalConstPtr           = cartesian_control_msgs::FollowCartesianTrajectoryGoalConstPtr;
};


/**
 * @brief TODO
 *
 * @tparam TrajectoryInterface The type of trajectory interface used for this
 * controller. Either hardware_interface::JointTrajectoryInterface or
 * hardware_interface::CartesianTrajectoryInterface.
 */
template <class TrajectoryInterface>
class PassThroughController
  : public controller_interface::MultiInterfaceController<TrajectoryInterface,
                                                          ur_controllers::SpeedScalingInterface>
  , public std::conditional<
      std::is_same<TrajectoryInterface, hardware_interface::JointTrajectoryInterface>::value,
      JointBase,
      CartesianBase>::type
{
public:
  PassThroughController()
    : controller_interface::MultiInterfaceController<TrajectoryInterface,
                                                     ur_controllers::SpeedScalingInterface>(
        true) // Make speed scaling optional
  {
  }

  // Alias for full qualifications of inherited types.
  // This enables a compact definition of member functions for both joint-based
  // and Cartesian-based PassThroughControllers.
  using Base = typename std::conditional<
      std::is_same<TrajectoryInterface, hardware_interface::JointTrajectoryInterface>::value,
      JointBase,
      CartesianBase>::type;

  bool init(hardware_interface::RobotHW* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * @brief Callback method for new action goals
   *
   * This method calls the \a on_new_cmd callback from the
   * hardware_interface::TrajectoryHandle<JointTrajectory>.
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
   * This method calls the \a on_cancel callback from the hardware_interface::TrajectoryHandle<JointTrajectory>.
   * Implementers of the ROS-control HW can implement how this notification is
   * handled by the robot vendor control.
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
    ActionDuration() : target(0.0), current(0.0) {}

    ros::Duration target;  ///< Target duration of the current action.
    ros::Duration current; ///< Real duration of the current action.
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
  bool withinTolerances(const typename Base::TrajectoryPoint& error,
                        const typename Base::Tolerance& tolerances);

  /**
   * @brief Check if follow trajectory goals are valid
   *
   * @param goal The goal to check.
   *
   * @return True if goal is valid, false otherwise
   */
  bool isValid(const typename Base::GoalConstPtr& goal);

  /**
   * @brief Call this when the action goal's time is up.
   */
  void timesUp();

  bool m_done;
  ActionDuration m_action_duration;
  std::unique_ptr<ur_controllers::SpeedScalingHandle> m_speed_scaling;
  std::vector<std::string> m_joint_names;
  typename Base::Tolerance m_path_tolerances;
  typename Base::Tolerance m_goal_tolerances;
  std::unique_ptr<typename Base::TrajectoryHandle> m_trajectory_handle;
  std::unique_ptr<actionlib::SimpleActionServer<typename Base::FollowTrajectoryAction> >
    m_action_server;
};


} // namespace trajectory_controllers


#include <pass_through_controllers/pass_through_controllers.hpp>

