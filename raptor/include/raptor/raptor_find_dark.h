/*!
 * \file raptor_find_dark.h
 * \brief Image processing framework.
 *
 * This file provides hooks to image subscriptions and sets up the output server for the mcom node.
 *
 * \author omernick
 * \date March 22,2012
 */

#ifndef RAPTOR_FIND_DARK_H_
#define RAPTOR_FIND_DARK_H_
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <raptor/polar_histogram.h>
#include <raptor/obstacle_histogram.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>
#include <cxcore.h>
#include <iostream>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include "cv_bridge/CvBridge.h"
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <blob/blob.h>
#include <blob/BlobResult.h>
#include <std_msgs/Int16.h>
#include <raptor/light_level_srv.h>
#include <std_msgs/Float32.h>

class raptor_find_dark
{
public:
  raptor_find_dark();
  ~raptor_find_dark();
  
private:
  bool get_vector_field(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res);
  bool get_darkness(raptor::light_level_srv::Request &req, raptor::light_level_srv::Response &res);
  void handle_new_image(const sensor_msgs::Image::ConstPtr& msg);
  void alter_thrsval(const std_msgs::Int16::ConstPtr &msg);
  void adjust_gain(const std_msgs::Float32::ConstPtr& msg);

  int threshold;
  IplImage* img;
  IplImage donkey_kong;
  cv::Mat kong;
  bool isFirstImg;
  float volume;
  sensor_msgs::CvBridge bridge;
  cv_bridge::CvImagePtr cv_ptr;
  ros::NodeHandle node;
  
  ros::Subscriber fd_volume;
  
  ros::Publisher near_light_sense;
  ros::Subscriber dark_thresh;
  ros::Subscriber image_subscription;
  ros::ServiceServer vector_gen;
  ros::ServiceServer light_sense;
  
};

int main(int argc, char **argv);

#endif