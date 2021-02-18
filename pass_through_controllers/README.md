# Pass-Through Controllers
A package for ROS-controllers and HW Interface mechanisms to enable forwarding of trajectories (both
joint-based and Cartesian) to the robot for interpolation.

## Rationale
The idea behind these controllers is to enable forwarding (pass through) incoming trajectories directly
to the robot and let the OEM driver-side take care of interpolating between the waypoints.
This is useful if your application demands industrial scale trajectory execution, and you prefer not to interpolate on the ROS side.
Having the complete trajectory available on the driver side can be beneficial regarding smoothness of execution and avoid
issues related to sending (streaming) ad hoc commands, as is classically done with current ROS-control approaches.
The controllers implement simple action servers for trajectory execution (both Cartesian and joint-based).
They are meant to be light-weight and their functionality is deliberately
limited to starting trajectories, canceling them, and providing real-time
feedback on progress.

Note that most of the work has to be done in the hardware abstraction of the robot.
And, unfortunately, this is very robot specific and hard to generalize. It's
somewhat similar to the _read()_ and _write()_ functions of ROS-control, which
need to implement robot-specific protocols.
This package provides the necessary HW interfaces to implement this feature.

## Controller .yaml
An example config could look like this:
```yaml
# A controller to forward joint trajectories to the robot
forward_joint_trajectories:
    type: "joint_trajectory_controllers/PassThroughController"
    joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6

# A controller to forward joint trajectories to the robot
forward_cartesian_trajectories:
    type: "cartesian_trajectory_controllers/PassThroughController"
    joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
```

## Examples
Call `` roslaunch pass_through_controllers demo.launch ``
to get a first impression and inspect how things work.
The ``cartesian_trajectory_action_client.py`` and the
``joint_trajectory_action_client.py`` from the _script_ folder trigger random
motion.


## Adapting your RobotHW
The pass_through_controllers expect either a ``JointTrajectoryInterface`` or a ``CartesianTrajectoryInterface``, depending on the trajectory to forward.
Registering them in your RobotHW could look like this:
```c++

#include <pass_through_controllers/trajectory_interface.h>

class YourRobot : public hardware_interface::RobotHW
{
...

  // Callbacks for the pass_through_controllers' events
  void startJointInterpolation(const hardware_interface::JointTrajectory& trajectory);
  void startCartesianInterpolation(const hardware_interface::CartesianTrajectory& trajectory);
  void cancelJointInterpolation();
  void cancelCartesianInterpolation();

  // New HW interfaces for trajectory forwarding
  hardware_interface::JointTrajectoryInterface jnt_traj_interface_;
  hardware_interface::CartesianTrajectoryInterface cart_traj_interface_;

  // Command and feedback buffers
  hardware_interface::JointTrajectory jnt_traj_cmd_;
  hardware_interface::JointTrajectoryFeedback jnt_traj_feedback_;
  hardware_interface::CartesianTrajectory cart_traj_cmd_;
  hardware_interface::CartesianTrajectoryFeedback cart_traj_feedback_;
}
```
And in the implementation:
```c++

YourRobot::YourRobot()
{
...
  // Initialize and register joint trajectory command handles for PassThroughControllers
  hardware_interface::JointTrajectoryHandle joint_trajectory_handle =
    hardware_interface::JointTrajectoryHandle(
      &jnt_traj_cmd_,
      &jnt_traj_feedback_,
      std::bind(&HWInterface::startJointInterpolation, this, std::placeholders::_1),
      std::bind(&HWInterface::cancelJointInterpolation, this));

  jnt_traj_interface_.registerHandle(joint_trajectory_handle);
  registerInterface(&jnt_traj_interface_);

  // Initialize and register Cartesian trajectory command handles for PassThroughControllers
  hardware_interface::CartesianTrajectoryHandle cartesian_trajectory_handle =
    hardware_interface::CartesianTrajectoryHandle(
      &cart_traj_cmd_,
      &cart_traj_feedback_,
      std::bind(&HWInterface::startCartesianInterpolation, this, std::placeholders::_1),
      std::bind(&HWInterface::cancelCartesianInterpolation, this));

  cart_traj_interface_.registerHandle(cartesian_trajectory_handle);
  registerInterface(&cart_traj_interface_);
  ...
}

void YourRobot::startJointInterpolation(const hardware_interface::JointTrajectory& trajectory)
{
  // Robot-specific implementation here
}

void YourRobot::startCartesianInterpolation(const hardware_interface::CartesianTrajectory& trajectory)
{
  // Robot-specific implementation here
}

void YourRobot::cancelJointInterpolation()
{
  // Robot-specific implementation here
}

void YourRobot::cancelCartesianInterpolation()
{
  // Robot-specific implementation here
}


```

The pass_through_controllers check for these interfaces and call the registered
callbacks upon receiving new action goals or preemption requests. The implementation is robot-specific.

In order to provide feedback for the pass_through_controllers' action clients,
you should periodically fill the feedback buffers:
```c++
  // Handle name is a convention
  jnt_traj_interface_.getHandle("joint_trajectory_handle").setFeedback(feedback);

  // Handle name is a convention
  cart_traj_interface_.getHandle("cartesian_trajectory_handle").setFeedback(feedback);

```

***
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 
