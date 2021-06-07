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
/*!\file    trajectory_interface.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/07/15
 *
 */
//-----------------------------------------------------------------------------


#pragma once

#include <functional>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryGoal.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryResult.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <vector>

namespace hardware_interface {

/**
 * @brief Hardware-specific done flags
 */
enum class ExecutionState
{
  SUCCESS = 0,
  NOT_RESPONDING = -1,
  GENERAL_ERROR = -2,
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
 * @brief A class implementing handles for trajectory hardware interfaces
 *
 * This is a special type of interface handle for PassThroughControllers. The
 * handles provide read/write access to trajectory command buffers and
 * read/write access to trajectory feedback buffers.
 *
 * @tparam TrajectoryType The type of trajectory used.
 * @tparam FeedbackType The type of feedback for the trajectory used.
 */
template<class TrajectoryType, class FeedbackType>
class TrajectoryHandle
{
  public:
    TrajectoryHandle() = delete;

    /**
     * @brief A trajectory handle for managing read/write functionality for
     * PassThroughControllers
     *
     * @param cmd The command buffer for read/write operations
     * @param feedback The feedback buffer for read/write operations
     */
    TrajectoryHandle(TrajectoryType* cmd, FeedbackType* feedback)
      : cmd_(cmd)
      , feedback_(feedback)
    {
      if (!cmd || !feedback)
      {
        throw HardwareInterfaceException(
          "Cannot create TrajectoryHandle. Make sure to provide both command and feedback buffers during construction");
      }
    };

    /**
     * @brief A trajectory handle for managing read/write functionality for
     * PassThroughControllers
     *
     * Overload for using callbacks to trigger precise start and cancel events
     * in user code. In the context of using PassThroughControllers,
     * implementers of ROS-control HW-interfaces can use this callback
     * mechanism to handle starting and canceling of trajectories on the robot
     * vendor controller.
     *
     * @param cmd The command buffer for read/write operations
     * @param feedback The feedback buffer for read/write operations
     * @param on_new_cmd Callback that is called upon receiving new commands
     * @param on_cancel Callback that is called when current commands are canceled
     */
    TrajectoryHandle(TrajectoryType* cmd,
                     FeedbackType* feedback,
                     std::function<void(const TrajectoryType&)> on_new_cmd,
                     std::function<void()> on_cancel)
      : cmd_(cmd)
      , feedback_(feedback)
      , cmd_callback_(on_new_cmd)
      , cancel_callback_(on_cancel)
    {
      if (!cmd || !feedback)
      {
        throw HardwareInterfaceException(
          "Cannot create TrajectoryHandle. Make sure to provide both command and feedback buffers during construction");
      }
    };

    ~TrajectoryHandle(){};

    /**
     * @brief Write the command buffer with the content of an new trajectory
     *
     * This will mainly be used by PassThroughControllers to store their new
     * incoming trajectories in the robot hardware interface.
     *
     * Note: The JointTrajectory type reorders the joints according to the given
     * joint resource list.
     *
     * @param command The new trajectory
     * @return True if command is feasible, false otherwise
     */
    bool setCommand(TrajectoryType command);

    /**
     * @brief Read a trajectory from the command buffer
     *
     * This can be used to access content from forwarded trajectories in the
     * robot hardware interface.
     *
     * @return The content of the trajectory command buffer
     */
    TrajectoryType getCommand() const {assert(cmd_); return *cmd_;}

    /**
     * @brief Cancel an active command
     */
    void cancelCommand()
    {
      if (cancel_callback_)
      {
        cancel_callback_();
      }
    }

    /**
     * @brief Set trajectory feedback for PassThroughControllers
     *
     * This should be used by the robot HW to provide feedback on trajectory
     * execution for the PassThroughControllers
     *
     * @param feedback The feedback content to write to the interface
     */
    void setFeedback(FeedbackType feedback)
    {
      assert(feedback_);
      *feedback_ = feedback;
    }

    /**
     * @brief Access trajectory feedback
     *
     * This can be used by PassThroughControllers to get trajectory feedback
     * from the hardware interface.
     *
     * @return The most recent feedback on the trajectory execution
     */
    FeedbackType getFeedback() const {assert(feedback_); return *feedback_;}

    /**
     * @brief Get the name of this trajectory handle
     *
     * By convention, this either returns \a joint_trajectory_handle for joint-based trajectory handles
     * and \a cartesian_trajectory_handle for Cartesian-based trajectory handles.
     *
     * @return The name that is associated with this specific handle
     */
    static std::string getName() noexcept;

    /**
     * @brief Set the order of joint names for trajectory reordering.
     *
     * @param joint_names
     */
    void setJointNames(const std::vector<std::string>& joint_names) noexcept {joint_names_ = joint_names;}

    /**
     * @brief Register a callback for the controller to react upon \a done signals from the hardware
     *
     * @param std::function The function to be called by the ROS controller
     */
    void registerDoneCallback(std::function<void(const ExecutionState&)>){}

    /**
     * @brief Mark the execution of a trajectory done.
     *
     * @param state The final state
     */
    void setDone(const ExecutionState& state){if (done_callback_ != nullptr) done_callback_(state);}

    /**
     * @brief Get the joint names associated with this handle
     *
     * @return Joint names
     */
    std::vector<std::string> getJointNames() const {return joint_names_;}

  private:
    TrajectoryType* cmd_;
    FeedbackType* feedback_;
    std::function<void(const TrajectoryType&)> cmd_callback_;
    std::function<void()> cancel_callback_;;
    std::function<void(const ExecutionState&)> done_callback_{nullptr};
    std::vector<std::string> joint_names_;
};


// Full spezialization for JointTrajectory
template<> inline
bool TrajectoryHandle<JointTrajectory, JointTrajectoryFeedback>::setCommand(JointTrajectory command)
{
  control_msgs::FollowJointTrajectoryGoal tmp;

  // Respect joint order by computing the map between msg indices to expected indices.
  // If msg is {A, C, D, B} and expected is {A, B, C, D}, the associated mapping vector is {0, 2, 3, 1}
  auto msg = command.trajectory.joint_names;
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

  for (auto point : command.trajectory.points)
  {
    trajectory_msgs::JointTrajectoryPoint p{point};  // init for equal data size

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

  assert(cmd_);
  *cmd_ = tmp;

  if (cmd_callback_)
  {
    cmd_callback_(*cmd_);
  }
  return true;
}


// Full spezialization for CartesianTrajectory
template<> inline
bool TrajectoryHandle<CartesianTrajectory, CartesianTrajectoryFeedback>::setCommand(CartesianTrajectory command)
{
  assert(cmd_);
  *cmd_ = command;

  if (cmd_callback_)
  {
    cmd_callback_(*cmd_);
  }
  return true;
}

// Full spezializations for name deduction
template<> inline
std::string TrajectoryHandle<JointTrajectory, JointTrajectoryFeedback>::getName() noexcept
{
  return "joint_trajectory_handle";
};

template<> inline
std::string TrajectoryHandle<CartesianTrajectory, CartesianTrajectoryFeedback>::getName() noexcept
{
  return "cartesian_trajectory_handle";
};

using JointTrajectoryHandle = TrajectoryHandle<JointTrajectory, JointTrajectoryFeedback>;
using CartesianTrajectoryHandle = TrajectoryHandle<CartesianTrajectory, CartesianTrajectoryFeedback>;


/**
 * @brief Hardware interface for commanding trajectories
 *
 * This special hardware interface is primarily used by PassThroughControllers,
 * which forward full trajectories to robots for interpolation. In contrast to
 * other hardware interfaces, this interface claims multiple resources and
 * offers write access to full trajectory buffers.
 *
 * @tparam TrajectoryType Distinguish between joint-based and Cartesian trajectories
 * @tparam FeedbackType The type of feedback for the trajectory used.
 */
template <class TrajectoryType, class FeedbackType>
class TrajectoryInterface
  : public hardware_interface::HardwareResourceManager<TrajectoryHandle<TrajectoryType, FeedbackType>,
                                                       hardware_interface::ClaimResources>
{
  public:

    /**
     * @brief Associate resources with this interface
     *
     * Call this right after initialization.
     * \Note: Proper resource handling depends on calling this
     * method \a before acquiring handles to this interface via getHandle().
     *
     * @param resources A list of resource names
     */
    void setResources(std::vector<std::string> resources)
    {
      joint_names_ = resources;
    }

    /**
     * @brief Claim multiple resources when using a single TrajectoryHandle
     *
     * This makes sure that PassThroughControllers claim all associated
     * resources.
     *
     * @param std::string Not used
     */
    void claim(std::string /*resource*/) override
    {
      for (const std::string& joint : joint_names_)
      {
        hardware_interface::HardwareResourceManager<
          TrajectoryHandle<TrajectoryType, FeedbackType>,
          hardware_interface::ClaimResources>::claim(joint);
      }
    }

    /**
     * @brief Specialized override for joint trajectory handles
     *
     * This passes the resource names associated with this interface to the
     * returned JointTrajectoryHandle, which can then make use of them for
     * reordering the incoming joint trajectories.
     *
     * In the case of CartesianTrajectoryHandle, this forwards the call
     * to \a getHandle from HardwareResourceManager.
     *
     * @param name Name of trajectory handle
     *
     * @return Trajectory handle associated to \e name.
     */
    TrajectoryHandle<TrajectoryType, FeedbackType> getHandle(const std::string& name);
    

  private:
    std::vector<std::string> joint_names_;
};

template<> inline
JointTrajectoryHandle TrajectoryInterface<JointTrajectory, JointTrajectoryFeedback>::getHandle(const std::string& name)
{
  JointTrajectoryHandle joint_handle = this->hardware_interface::HardwareResourceManager<
    JointTrajectoryHandle, hardware_interface::ClaimResources>::getHandle(name);

  joint_handle.setJointNames(joint_names_);
  return joint_handle;
};

template<> inline
CartesianTrajectoryHandle TrajectoryInterface<CartesianTrajectory, CartesianTrajectoryFeedback>::getHandle(const std::string& name)
{
  return this->hardware_interface::HardwareResourceManager<
    CartesianTrajectoryHandle, hardware_interface::ClaimResources>::getHandle(name);
};

/**
 * @brief Hardware interface for commanding (forwarding) joint-based trajectories
 */
using JointTrajectoryInterface = TrajectoryInterface<JointTrajectory, JointTrajectoryFeedback>;

/**
 * @brief Hardware interface for commanding (forwarding) Cartesian trajectories
 */
using CartesianTrajectoryInterface = TrajectoryInterface<CartesianTrajectory, CartesianTrajectoryFeedback>;

}
