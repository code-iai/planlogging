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
  int nReturnvalue = 0;
  
  if(ros::ok()) {
    ros::NodeHandle nh;
    
    string strPackageName;
    nh.param("package_name", strPackageName, string("planlogger"));
    
    CPlanLoggerROS *plLogger = new CPlanLoggerROS();
    plLogger->setExperimentsResultRoot(ros::package::getPath(strPackageName) + "/experiments");
    plLogger->renewSession();
    
    ros::spin();
    
    delete plLogger;
  } else {
    cerr << "Couldn't initialize ROS node. Something went terribly wrong here." << endl;
    nReturnvalue = -1;
  }
  
  return nReturnvalue;
}
