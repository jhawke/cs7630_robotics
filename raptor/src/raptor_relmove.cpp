/*!
 * \file raptor_relmove.cpp
 * \brief Waypoint generator behavior
 *
 * raptor_relmove generates a desired heading input to the vector sum controller of the Raptor system.
 *
 * \author omernick
 * \date March 22,2012
 */

#include <raptor/raptor_relmove.h>

#include <std_msgs/Float32.h>
#include <raptor/twist_srv.h>
#include <math.h>
#include <ros/ros.h>
#include <rovio_shared/rovio_position.h>
#include <raptor/polar_histogram.h>
#include <raptor_commander/rel_pos_req.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>

#define REMAINDER .9
#define NUM_UNITS 10

using namespace std;

raptor_relmove::raptor_relmove()
{
    volume = 1;
    loc_is_registered = false;
    //Advertise the waypoint setting service
    relmove_vol = node.subscribe<std_msgs::Float32>("REL_MOVE_GAIN",1,&raptor_relmove::adjust_gain,this);
    location_setter = node.subscribe<raptor_commander::rel_pos_req>("REL_MOVE_PARAMS",1,&raptor_relmove::register_new_location,this);
    location_ll_setter = node.subscribe<raptor_commander::rel_pos_req>("REL_MOVE_PARAMS_LL",1,&raptor_relmove::register_new_location_ll,this);
    mcom_ll_relmover = node.advertise<std_msgs::Float32>("mcom_ll_relmove",1);
    //Advertise the vector output service
    
    vector_output = node.advertiseService("raptor_relmove_srv",&raptor_relmove::give_location,this);
    //Subscribe to the locational service
    location_request = node.serviceClient<rovio_shared::rovio_position>("rovio_position");
         ROS_INFO("Relative waypointer initialized.");
}

raptor_relmove::~raptor_relmove()
{
  
}

void raptor_relmove::adjust_gain(const std_msgs::Float32::ConstPtr& msg)
{
   volume = msg->data;
   ROS_INFO("Relmove volume updated to %f",volume);
}

void raptor_relmove::register_new_location_ll(const raptor_commander::rel_pos_req::ConstPtr &msg)
{
  loc_is_registered = true;
  std_msgs::Float32 msg_out;
  msg_out.data = msg->theta_rad;
  mcom_ll_relmover.publish(msg_out);
      //Get current Rovio location.
  ROS_DEBUG("Low-level relative rotation request recieved.");
}
void raptor_relmove::register_new_location(const raptor_commander::rel_pos_req::ConstPtr &msg)
{
  loc_is_registered = true;
  goal_theta = msg->theta_rad;
  goal_val = msg->val;
      //Get current Rovio location.
  ROS_INFO("Calling get-loc.");
  rovio_shared::rovio_position srv;
  if(location_request.call(srv))
  //if(true)  
  {
    if(srv.response.is_valid>0)
    {
      //srv.response.x = 200;
      //srv.response.y = 200;
      //srv.response.theta = 3.1415;
      goal_theta += srv.response.theta;
    }
    else
    {
      ROS_DEBUG("Northstar signal too low for NS-RELMOVE");
    }
  }  
  stop_time = (ros::Time::now().toSec())+msg->time_sec;
  ROS_DEBUG("Registered new relative loc. theta = %f, val=%f, time=%f.",goal_theta,goal_val,stop_time);
}

bool raptor_relmove::give_location(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res)
{
  ROS_DEBUG("Recieved a waypoint request.");
  boost::array<int16_t,360> arr_out = {0};
  if(!loc_is_registered)
  {
      ROS_DEBUG("Location not registered. Output should be zeros.");
  }
  if(((ros::Time::now().toSec())<stop_time)&&(loc_is_registered))
  {
    ROS_DEBUG("Calling get-loc.");
    rovio_shared::rovio_position srv;
    //Get current Rovio location.
    if(location_request.call(srv))
    //if(true)
    {
      //srv.response.x = 200;
      //srv.response.y = 200;
      //srv.response.theta = 3.1415;
      ROS_DEBUG("Get-loc returned with %d,%d,%g",srv.response.x,srv.response.y,srv.response.theta);
    //Calculate theta required to go from current to goal.
      int16_t rel_theta = ((goal_theta-srv.response.theta)*180/(3.1415)+360);
      rel_theta = 180+(360-rel_theta);
      if(rel_theta>359){rel_theta = rel_theta%360;}
      while(rel_theta<0){rel_theta = rel_theta+360;}
      ROS_DEBUG("Processed waypoint request:\n\tGoal is theta = %f\n\tPosition is theta = %f\n\tRelative theta is %d.",goal_theta,srv.response.theta,rel_theta);
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
  
  for(int i = 0; i<360; i++)
  {
   arr_out[i]*=volume; 
  }
  
  res.hist = arr_out;
  return true;
}

int main(int argc, char **argv)
{
  
  ros::init(argc,argv,"raptor_relmove");
  
  raptor_relmove rel_waypointer;
  
  ros::spin();
  
  return EXIT_SUCCESS;
}