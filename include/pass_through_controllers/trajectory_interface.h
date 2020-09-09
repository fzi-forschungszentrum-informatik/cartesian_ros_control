// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

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
#include <trajectory_msgs/JointTrajectory.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>
#include <vector>

namespace hardware_interface {

using JointTrajectory = trajectory_msgs::JointTrajectory;
using CartesianTrajectory = cartesian_control_msgs::CartesianTrajectory;



/**
 * @brief A class implementing handles for trajectory hardware interfaces
 *
 * This is a special type of interface handles for PassThroughControllers. The
 * handles provide access to trajectory command buffers.
 *
 * @tparam TrajectoryType The type of trajectory used
 */
template<class TrajectoryType>
class TrajectoryHandle
{
  public:
    TrajectoryHandle() = delete;

    TrajectoryHandle(const std::string& name, TrajectoryType* cmd)
      : m_name(name), m_cmd(cmd)
    {
      if (!cmd)
      {
        throw HardwareInterfaceException(
          "Cannot create TrajectoryHandle. Command data pointer is null.");
      }
    };

    TrajectoryHandle(const std::string& name,
                     TrajectoryType* cmd,
                     std::function<void(const TrajectoryType&)> on_new_cmd,
                     std::function<void()> on_cancel
                     )
      : m_name(name)
      , m_cmd(cmd)
      , m_cmd_callback(on_new_cmd)
      , m_cancel_callback(on_cancel)
    {
      if (!cmd)
      {
        throw HardwareInterfaceException(
          "Cannot create TrajectoryHandle. Command data pointer is null.");
      }
    };

    ~TrajectoryHandle(){};

    void setCommand(TrajectoryType command)
    {
      assert(m_cmd);
      *m_cmd = command;

      if (m_cmd_callback)
      {
        m_cmd_callback(*m_cmd);
      }
    }

    void cancelCommand()
    {
      if (m_cancel_callback)
      {
        m_cancel_callback();
      }
    }

    TrajectoryType getCommand() const {assert(m_cmd); return *m_cmd;}

    std::string getName() const noexcept { return m_name; }

  private:
    std::string m_name;
    TrajectoryType* m_cmd;
    std::function<void(const TrajectoryType&)> m_cmd_callback;
    std::function<void()> m_cancel_callback;;
};


using JointTrajectoryHandle = TrajectoryHandle<JointTrajectory>;
using CartesianTrajectoryHandle = TrajectoryHandle<CartesianTrajectory>;


/**
 * @brief Hardware interface for commanding trajectories
 *
 * This special hardware interface is primarily used by PassThroughControllers,
 * which forward full trajectories to robots for interpolation. In contrast to
 * other hardware interfaces, this interface claims multiple resources and
 * offers write access to full trajectory buffers.
 *
 * @tparam TrajectoryType Distinguish between joint-based and Cartesian trajectories
 */
template <class TrajectoryType>
class TrajectoryInterface
  : public hardware_interface::HardwareResourceManager<TrajectoryHandle<TrajectoryType>,
                                                       hardware_interface::ClaimResources>
{
  public:

    /**
     * @brief Associate resources with this interface
     *
     * Call this during initialization of your controller.
     * \Note: Proper resource handling depends on calling this
     * method \a before acquiring handles to this interface via getHandle().
     *
     * @param resources A list of resource names
     */
    void setResources(std::vector<std::string> resources)
    {
      m_joint_names = resources;
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
      for (const std::string& joint : m_joint_names)
      {
        hardware_interface::HardwareResourceManager<
          TrajectoryHandle<TrajectoryType>,
          hardware_interface::ClaimResources>::claim(joint);
      }
    }

  private:
    std::vector<std::string> m_joint_names;
};

/**
 * @brief Hardware interface for commanding (forwarding) joint-based trajectories
 */
using JointTrajectoryInterface = TrajectoryInterface<JointTrajectory>;

/**
 * @brief Hardware interface for commanding (forwarding) Cartesian trajectories
 */
using CartesianTrajectoryInterface = TrajectoryInterface<CartesianTrajectory>;

}
