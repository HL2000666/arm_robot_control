#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "hardware_interface.h" 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hardware_interface");
  ros::NodeHandle nh;

  MyRobot robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate loop_rate(100); // Adjust loop rate as needed

  double time = 0.0;

  while (ros::ok())
  {
    robot.setReferencePosition(time);
    robot.readSim(ros::Time::now(), loop_rate.expectedCycleTime());
    cm.update(ros::Time::now(), loop_rate.expectedCycleTime());
    robot.writeSim(ros::Time::now(), loop_rate.expectedCycleTime());
    time += 0.1;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
