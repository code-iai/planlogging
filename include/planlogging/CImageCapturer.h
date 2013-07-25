#ifndef __C_IMAGECAPTURER_H__
#define __C_IMAGECAPTURER_H__


// System
#include <string>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV
#include <cv.h>
#include <highgui.h>

using namespace std;


class CImageCapturer {
 private:
  ros::Subscriber m_subImage;
  bool m_bReceived;
  sensor_msgs::Image m_imgReceived;

 public:
  CImageCapturer();
  ~CImageCapturer();
  
  bool captureFromTopic(string strTopicName, string strFileName);
  void imageCallback(const sensor_msgs::Image &imgData);
};


#endif /* __C_IMAGECAPTURER_H__ */
