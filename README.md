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

The rational behind these controllers is that they pass incoming trajectory action goals down to the robot in an open loop fashion.
They shall limit their functionality to starting trajectories, canceling them, and providing real-time feedback on progress.

# Examples

```bash
roslaunch pass_through_controllers demo.launch
```

