# Pass-Through Controllers
A package for ROS-controllers that forward trajectories (both
joint-based and Cartesian) to the robot for interpolation.

This package introduces new functionality to ROS-control for managing hardware interfaces and handles for Trajectories.
In particular, this packages offers:

- JointTrajectoryInterface
- CartesianTrajectoryInterface

and the according handles:
- JointTrajectoryHandle
- CartesianTrajectoryHandle

The rational behind these controllers is that they pass incomming trajectory action goals down to the robot in an open loop fashion.
They shall limit their functionality to starting trajectories, aborting them, and providing real-time feedback on progress.

# Examples
Currently, the examples need a customly adjusted ROS-control HW to make use of the new interfaces and handles.
Use e.g. [this one][1] until we decide on a suitable way of providing this within this package.

```bash
roslaunch pass_through_controllers examples.launch
```


[1]: https://ids-git.fzi.de/scherzin/ros_control_boilerplate/-/tree/pass_through_controllers


