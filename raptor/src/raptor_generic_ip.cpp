/*!
 * \file raptor_generic_ip.cpp
 * \brief Image processing framework.
 *
 * This file provides hooks to image subscriptions and sets up the output server for the mcom node.
 *
 * \author omernick
 * \date March 22,2012
 */

#include <raptor/raptor_generic_ip.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <raptor/polar_histogram.h>
#include <raptor/obstacle_histogram.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>

//Put any local defines here.

#define YOUR_MOTHER "town bicycle"

using namespace std;

raptor_generic_ip::raptor_generic_ip()
{
    //Subscribe to the image producing publisher. (/gscam/image_raw)
    image_subscription = node.subscribe<sensor_msgs::Image>("/gscam/image_raw",1,&raptor_generic_ip::handle_new_image, this);
    //Advertise your service.
    vector_gen = node.advertiseService("raptor_generic_ip_srv",&raptor_generic_ip::get_vector_field,this);
}

raptor_generic_ip::~raptor_generic_ip()
{
  
}

bool raptor_generic_ip::get_vector_field(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res)
{
  ROS_DEBUG("Function 'name' called.");
  //Your image processing shit goes here.
  
  //Finally, fill up the array res.hist
  //res.hist = WHATEVER;
  return true;
}

void raptor_generic_ip::handle_new_image(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("ASS"); 
  //Put your pre-handler shite in here. You can just copy the latest or some damn thing.
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"raptor_generic_ip");
  
  raptor_generic_ip ip_core;
  
  ros::spin();
  
  return EXIT_SUCCESS; 
}

