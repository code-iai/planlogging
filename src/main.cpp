// System
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Private
#include <planlogging/CPlanLoggerROS.h>

using namespace std;


// Global variables


int main(int argc, char **argv) {
  ros::init(argc, argv, "planlogger");
  
  CPlanLoggerROS *plLogger = new CPlanLoggerROS();
  plLogger->setExperimentsResultRoot(ros::package::getPath("planlogger") + "/experiments");
  plLogger->renewSession();
  
  ros::spin();
  
  delete plLogger;
  return 0;
}
