/*!
 * \file raptor_stalk.h
 * \brief Image processing framework.
 *
 * This file provides hooks to image subscriptions and sets up the output server for the mcom node.
 *
 * \author omernick
 * \date March 22,2012
 */

#ifndef RAPTOR_STALK_H_
#define RAPTOR_STALK_H_

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
#include <sensor_msgs/image_encodings.h>
#include <boost/array.hpp>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

class raptor_stalk
{
public:
  raptor_stalk();
  ~raptor_stalk();
  
private:
  bool get_vector_field(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res);
  void handle_new_image(const sensor_msgs::Image::ConstPtr& msg);
  void adjust_gain(const std_msgs::Float32::ConstPtr& msg);
 
  IplImage donkey_kong;
  IplImage* img;
  
  bool isFirstImg;
  float volume;
  
  ros::NodeHandle node;
  sensor_msgs::CvBridge bridge;
  cv_bridge::CvImagePtr cv_ptr;
  ros::Publisher target_centroid;
  ros::Subscriber image_subscription;
  ros::Subscriber stalk_volume;
  ros::ServiceServer vector_gen;
};

int main(int argc, char **argv);

#endif