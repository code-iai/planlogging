#ifndef __C_PLANLOGGER_ROS_H__
#define __C_PLANLOGGER_ROS_H__


// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// Other
#include <designators/CDesignator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

// Private
#include <planlogging/CPlanLogger.h>
#include <planlogging/CImageCapturer.h>
#include <export/CExporterDot.h>
#include <export/CExporterOwl.h>


class CPlanLoggerROS : public CPlanLogger {
 private:
  ros::ServiceServer m_srvStartNode;
  ros::ServiceServer m_srvStopNode;
  ros::ServiceServer m_srvAlterNode;
  ros::ServiceServer m_srvControl;
  
  void startServices();
  void stopServices();
  
  bool serviceCallbackStartNode(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
  bool serviceCallbackStopNode(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
  bool serviceCallbackAlterNode(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
  bool serviceCallbackControl(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res);
  
 public:
  CPlanLoggerROS();
  ~CPlanLoggerROS();
};


#endif /* __C_PLANLOGGER_ROS_H__ */
