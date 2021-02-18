# Cartesian ROS Control

This package brings mechanisms for Cartesian control to the ROS-control framework.

## Overview

New functionality (colored):

![Colored: New contributions from this package](./cartesian_ros_control/doc/cartesian_ros_control.png)


## Rationale

As opposed to joint-based control, Cartesian control is often more intuitive for programmers to specify how a tool (robot end-effector) should move in their application.
For instance, gluing, grinding, polishing and all sorts of other surface-related tasks benefit from a straight-forward task formulation with Cartesian coordinates.

It comes at a surprise that there hasn't been native support of Cartesian control in ROS. Yet, the number of OEMs, whose drivers support Cartesian control interfaces is growing.
This set of packages aims at filling this gap and get you started with Cartesian control.


## Major features at a glance
- **Add Cartesian functionality to ROS control**. This brings new interfaces for
  controller design, such as a ```PoseCommandInterface```, a ```TwistCommandInterface```, and a new Cartesian trajectory  definition. Example controllers include a ```TwistController``` and a ```CartesianTrajectoryController```. 

- **Enable Cartesian trajectory control** in your applications. Specify your task comfortably with 
  waypoints in task space. ROS-side interpolation and streaming of setpoints over the new interfaces is only one of several alternatives. 

- **Use (conventional) ROS control** for Cartesian trajectory execution.  You don't need to change anything in the driver's HW-abstraction of your specific robot if that supports current ROS control.

- **Hand-over control to the robot by forwarding trajectories**.
Two new interfaces ```CartesianTrajectoryInterface``` and ```JointTrajectoryInterface``` let robots take care of driver-side interpolation to achieve best performance.

- **Speed-scale trajectory execution**. All trajectory executions (both Cartesian and joint-based) can be speed-scaled within 0 to 100% at runtime. This gives you flexibility in setting-up new applications and during test runs. Changing this continuously even lets you reshape trajectory execution without re-teaching. 

## Robots Overview
In the spirit of ROS control, the implementation is robot-agnostic and shall support applications on a wide range of robots. The table below shows what features will be available with this enhancement.

| Feature | Robots with new interfaces | Robots with current ROS control |
| -------- | -------- | --- |
| Cartesian trajectory control | &check; | &check; |
| Cartesian trajectory forwarding | &check; | |
| Joint trajectory forwarding | &check;  | |
| Speed-scale trajectories | &check; |  |


## Acknowledgement
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
