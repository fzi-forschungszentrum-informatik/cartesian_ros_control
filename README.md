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
    type: "pass_through_controllers/JointTrajectoryController"
    joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6

# A controller to forward joint trajectories to the robot
forward_cartesian_trajectories:
    type: "pass_through_controllers/CartesianTrajectoryController"
    joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
```

## Examples
The [integration
tests](https://github.com/UniversalRobots/Universal_Robots_ROS_passthrough_controllers/test) from this package might give a first impression on how to use this controller.

Secondly, the [`ur_robot_driver`](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_robot_driver)
uses those controllers.


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
}
```
And in the implementation:
```c++

YourRobot::YourRobot()
{
  ...
  // Register callbacks for trajectory passthrough
  jnt_traj_interface_.registerGoalCallback(
      std::bind(&HardwareInterface::startJointInterpolation, this, std::placeholders::_1));
  jnt_traj_interface_.registerCancelCallback(std::bind(&HardwareInterface::cancelInterpolation, this));
  cart_traj_interface_.registerGoalCallback(
      std::bind(&HardwareInterface::startCartesianInterpolation, this, std::placeholders::_1));
  cart_traj_interface_.registerCancelCallback(std::bind(&HardwareInterface::cancelInterpolation, this));
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

// Once execution is finished on the robot, it should signal the driver about the finished
// trajectory. This is vendor-specific code and the below is only an example.
void trajectorySignalFromRobotReceived(robot_vendor::TrajectoryResult result)
{
  hardware_interface::ExecutionState final_state;
  switch (result)
  {
    case robot_vendor::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS:
    {
      final_state = hardware_interface::ExecutionState::SUCCESS;
      break;
    }
    case urcl::control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED:
    {
      final_state = hardware_interface::ExecutionState::PREEMPTED;
      break;
    }
    case urcl::control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE:
    {
      final_state = hardware_interface::ExecutionState::ABORTED;
      break;
    }
    default:
    {
      std::stringstream ss;
      ss << "Unknown trajectory result: " << urcl::toUnderlying(result);
      throw(std::invalid_argument(ss.str()));
    }
  }

  // If you track which controller is running...
  // You will have to know which interface to send the result
  if (joint_forward_controller_running_)
  {
    jnt_traj_interface_.setDone(final_state);
  }
  else if (cartesian_forward_controller_running_)
  {
    cart_traj_interface_.setDone(final_state);
  }
  else
  {
    ROS_ERROR_STREAM("Received forwarded trajectory result with no forwarding controller running.");
  }

}


```

The pass_through_controllers check for these interfaces and call the registered
callbacks upon receiving new action goals or preemption requests. The implementation is robot-specific.

In order to provide feedback for the pass_through_controllers' action clients,
you should periodically fill the feedback buffers:
```c++
  if (joint_forward_controller_running_)
  {
    jnt_traj_interface_.setFeedback(feedback);
  }
  if (cartesian_forward_controller_running_)
  {
    cart_traj_interface_.setFeedback(feedback);
  }

```

## FAQ
The passthrough controllers forward the trajectories to the robot for interpolation.
On the ROS side, the possibilities to interfere with a running action are fairly limited and you can only start or preempt the trajectory execution.
On the robot side, however, there will be additional ways to interfere with the execution that mostly depend on the OEM robot program and the implementation of the respective driver.
The passthrough controllers support the additional concept of speed scaling, which can be used to pause trajectories.

The following examples explain the passthrough controllers' behavior with the Universal Robots [`ur_robot_driver`](http://wiki.ros.org/ur_robot_driver).

### What happens when you hit the emergency stop?
The trajectory execution is paused in this case by internally setting the speed scaling to zero. Once the program resumes, the trajectory execution continues.
The passthrough controllers will wait until the execution finishes but will report an execution failure due to missing the waypoints' time requirements if a goal tolerance is specified.

### What happens when you power-off the system?
This aborts the current trajectory execution.

### What happens when the robot goes into protective stop?
This is handled similar to hitting the emergency stop. The speed scaling is set
to zero and the trajectory execution continues once the protective
stop is resolved.

### What happens when you stop the program?
This stops all ROS-controllers and aborts the current trajectory execution.

## Acknowledgement
Developed in collaboration between:

[<img height="60" alt="Universal Robots A/S" src="doc/resources/ur_logo.jpg">](https://www.universal-robots.com/) &nbsp; and &nbsp;
[<img height="60" alt="FZI Research Center for Information Technology" src="doc/resources/fzi_logo.png">](https://www.fzi.de).

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
