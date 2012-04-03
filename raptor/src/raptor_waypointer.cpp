/*!
 * \file raptor_waypointer.cpp
 * \brief Waypoint generator behavior
 *
 * raptor_waypointer generates a desired heading input to the vector sum controller of the Raptor system.
 *
 * \author omernick
 * \date March 22,2012
 */

#include <raptor/raptor_waypointer.h>

#include <raptor/twist_srv.h>
#include <math.h>
#include <ros/ros.h>
#include <rovio_shared/rovio_position.h>
#include <raptor/polar_histogram.h>
#include <raptor/position_request_srv.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>

#define REMAINDER .9
#define NUM_UNITS 10

using namespace std;

raptor_waypointer::raptor_waypointer()
{
    //Advertise the waypoint setting service
    location_setter = node.advertiseService("set_raptor_waypoint",&raptor_waypointer::register_new_location,this);
    //Advertise the vector output service
    vector_output = node.advertiseService("vector_output_waypointer",&raptor_waypointer::give_location,this);
    //Subscribe to the locational service
    location_request = node.serviceClient<rovio_shared::rovio_position>("rovio_position_test");
         ROS_INFO("Waypointer initialized.");
}

raptor_waypointer::~raptor_waypointer()
{
  
}

bool raptor_waypointer::register_new_location(raptor::position_request_srv::Request &req, raptor::position_request_srv::Response &res)
{
  goal_x = req.x;
  goal_y = req.y;
  goal_val = req.val;
  stop_time = (ros::Time::now().toSec())+req.time_sec;
  ROS_DEBUG("Registered new location. x=%d, y=%d, val=%f, time=%f.",goal_x,goal_y,goal_val,stop_time);
  return true;
}

bool raptor_waypointer::give_location(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res)
{
  ROS_DEBUG("Recieved a waypoint request.");
  boost::array<int16_t,360> arr_out = {0};
  if((ros::Time::now().toSec())<stop_time)
  {
    ROS_DEBUG("Calling get-loc.");
    rovio_shared::rovio_position srv;
    //Get current Rovio location.
    //if(location_request.call(srv))
    //{
    if(true)
    {
      srv.response.x = 200;
      srv.response.y = 200;
      srv.response.theta = 3.1415;
      ROS_DEBUG("Get-loc returned with %d,%d,%g",srv.response.x,srv.response.y,srv.response.theta);
    //Calculate theta required to go from current to goal.
      float delta_x = (goal_x-srv.response.x);
      float delta_y = (goal_y-srv.response.y);
      float goal_theta = atan2(delta_y,delta_x);
      int16_t rel_theta = ((goal_theta-srv.response.theta)*180/(3.1415)+360);
      rel_theta = 180+(360-rel_theta);
      if(rel_theta>359){rel_theta = rel_theta%360;}
      ROS_DEBUG("Processed waypoint request:\n\tGoal is x=%d, y=%d\n\tPosition is x=%d,y=%d\n\tGoal theta is %f; current theta is %f\n\tRelative theta is %d.",goal_x,goal_y,srv.response.x,srv.response.y,goal_theta,srv.response.theta,rel_theta);
    //Place fuzzed value into array.
      for(int i=0;i<NUM_UNITS;i++)
      {
	arr_out[rel_theta+i]=goal_val*pow(REMAINDER,i);
	arr_out[rel_theta-i]=arr_out[rel_theta+i];
      }
    }
  }
  res.hist = arr_out;
  return true;
}

int main(int argc, char **argv)
{
  
  ros::init(argc,argv,"raptor_waypointer");
  
  raptor_waypointer waypointer;
  
  ros::spin();
  
  return EXIT_SUCCESS;
}