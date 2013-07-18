// System
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>

// Private
#include <planlogging/CPlanLoggerROS.h>

using namespace std;


int main(int argc, char **argv) {
  ros::init(argc, argv, "planlogger");
  
  CPlanLoggerROS *plLogger = new CPlanLoggerROS();
  ros::spin();
  
  return 0;
}
