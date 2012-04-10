/*!
 * \file raptor_absmove.cpp
 * \brief Waypoint generator behavior
 *
 * raptor_absmove generates a desired heading input to the vector sum controller of the Raptor system.
 *
 * \author omernick
 * \date March 22,2012
 */

#include <raptor/raptor_absmove.h>

#include <raptor/twist_srv.h>
#include <math.h>
#include <ros/ros.h>
#include <rovio_shared/rovio_position.h>
#include <raptor/polar_histogram.h>
#include <raptor/abs_pos_req.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>

#define REMAINDER .9
#define NUM_UNITS 10

using namespace std;

raptor_absmove::raptor_absmove()
{
    //Advertise the waypoint setting service
    location_setter = node.subscribe<raptor::abs_pos_req>("set_abs_waypoint",1,&raptor_absmove::register_new_location,this);
    //Advertise the vector output service
    vector_output = node.advertiseService("raptor_absmove_srv",&raptor_absmove::give_location,this);
    //Subscribe to the locational service
    location_request = node.serviceClient<rovio_shared::rovio_position>("rovio_position");
    ROS_INFO("Absolute waypointer initialized.");
    loc_is_registered = false;
}

raptor_absmove::~raptor_absmove()
{
  
}

void raptor_absmove::register_new_location(const raptor::abs_pos_req::ConstPtr &msg)
{
  loc_is_registered = true;
  goal_x = msg->x;
  goal_y = msg->y;
  goal_val = msg->val;
  stop_time = (ros::Time::now().toSec())+msg->time_sec;
  ROS_DEBUG("Registered new location. x=%d, y=%d, val=%f, time=%f.",goal_x,goal_y,goal_val,stop_time);
}

bool raptor_absmove::give_location(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res)
{
  ROS_DEBUG("Recieved a waypoint request.");
  boost::array<int16_t,360> arr_out = {0};
  if((ros::Time::now().toSec())<stop_time)
  {
    ROS_DEBUG("Calling get-loc.");
    rovio_shared::rovio_position srv;
    //Get current Rovio location.
    if(location_request.call(srv))
    {
//    if(true)
//    {
//     srv.response.x = 200;
//      srv.response.y = 200;
//      srv.response.theta = 3.1415;
      ROS_DEBUG("Get-loc returned with %d,%d,%g",srv.response.x,srv.response.y,srv.response.theta);
    //Calculate theta required to go from current to goal.
      float delta_x = (goal_x-srv.response.x);
      float delta_y = (goal_y-srv.response.y);
      float goal_theta = atan2(delta_y,delta_x);
      ROS_DEBUG("Delta x is %f; dy is %f",delta_x,delta_y);
      int16_t rel_theta = ((goal_theta-srv.response.theta)*180/(3.1415)+360);
      rel_theta = 180+(360-rel_theta);
      if(rel_theta>359){rel_theta = rel_theta%360;}
      if(rel_theta<0){rel_theta = rel_theta+360;}
      ROS_DEBUG("Processed waypoint request:\n\tGoal is x=%d, y=%d\n\tPosition is x=%d,y=%d\n\tGoal theta is %f; current theta is %f\n\tRelative theta is %d.",goal_x,goal_y,srv.response.x,srv.response.y,goal_theta,srv.response.theta,rel_theta);
    //Place fuzzed value into array.
      for(int i=0;i<NUM_UNITS;i++)
      {
	if((rel_theta+i)>359) 
	{
	  arr_out[rel_theta+i-360] = goal_val*pow(REMAINDER,i);
	}
	else
	{
	  arr_out[rel_theta+i]=goal_val*pow(REMAINDER,i);
	}
	if((rel_theta-i)<0)
	{
	 arr_out[(rel_theta-i)+360]=goal_val*pow(REMAINDER,i);
	}
	else
	{
	  arr_out[rel_theta-i]=goal_val*pow(REMAINDER,i);
	}
      }
    }
  }
  res.hist = arr_out;
  return true;
}

int main(int argc, char **argv)
{
  
  ros::init(argc,argv,"raptor_absmove");
  
  raptor_absmove abs_waypointer;
  
  ros::spin();
  
  return EXIT_SUCCESS;
}