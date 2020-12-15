// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    control_node.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/09/09
 *
 */
//-----------------------------------------------------------------------------


// Ros
#include <ros/ros.h>

// Ros control
#include <controller_manager/controller_manager.h>

// This project
#include "hw_interface.h"
#include "ros/time.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw_interface_example");
  ros::NodeHandle nh;
  examples::HWInterface hw_interface;
  controller_manager::ControllerManager controller_manager(&hw_interface);
  
  int control_rate; 
  if (!nh.getParam("control_rate", control_rate))
  { 
    control_rate = 125;
    ROS_INFO_STREAM("Failed to load 'control_rate' from parameter server. Using default " << control_rate);
  }

  // Control cycle for the robot
  ros::Rate rate(control_rate);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok())
  {
    auto period = rate.expectedCycleTime();  // use nominal cycle

    hw_interface.read(ros::Time::now(), period);
    hw_interface.write(ros::Time::now(), period);
    controller_manager.update(ros::Time::now(), period);

    rate.sleep();
  }


  return 0;
}
