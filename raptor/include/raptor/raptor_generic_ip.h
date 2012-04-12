/*!
 * \file raptor_generic_ip.h
 * \brief Image processing framework.
 *
 * This file provides hooks to image subscriptions and sets up the output server for the mcom node.
 *
 * \author omernick
 * \date March 22,2012
 */

#ifndef RAPTOR_GENERIC_IP_H_
#define RAPTOR_GENERIC_IP_H_

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

class raptor_generic_ip
{
public:
  raptor_generic_ip();
  ~raptor_generic_ip();

private:
  bool get_vector_field(raptor::obstacle_histogram::Request &req, raptor::obstacle_histogram::Response &res);
  void handle_new_image(const sensor_msgs::Image::ConstPtr& msg);
 void alter_darkval(const std_msgs::Int16::ConstPtr &msg);
 
 
 
  IplImage* img;
  IplImage donkey_kong;
  cv::Mat kong;
  bool isFirstImg;
  int volume;
  sensor_msgs::CvBridge bridge;
  cv_bridge::CvImagePtr cv_ptr;
  ros::NodeHandle node;
  
  ros::Subscriber dark_thresh;
  ros::Subscriber image_subscription;
  ros::ServiceServer vector_gen;
};

int main(int argc, char **argv);

#endif