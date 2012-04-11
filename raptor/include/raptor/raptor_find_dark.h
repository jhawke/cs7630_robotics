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
#include <cv.h>
#include <highgui.h>
#include <math.h>

class raptor_find_dark
{
public:
  raptor_find_dark();
  ~raptor_find_dark();
  
private:
  bool get_vector_field(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res);
  void handle_new_image(const sensor_msgs::Image::ConstPtr& msg);
 
  IplImage donkey_kong;
  IplImage* img;
  
  bool isFirstImg;
  int volume;
  
  ros::NodeHandle node;
  
  ros::Subscriber image_subscription;
  ros::ServiceServer vector_gen;
};

int main(int argc, char **argv);

#endif