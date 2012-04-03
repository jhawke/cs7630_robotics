/*!
 * \file raptor_waypointer_testbench.cpp
 * \brief Waypoint generator behavior
 *
 * raptor_waypointer generates a desired heading input to the vector sum controller of the Raptor system.
 *
 * \author omernick
 * \date March 22,2012
 */

#include <raptor/raptor_waypointer_testbench.h>

#include <rovio_shared/twist_srv.h>
#include <math.h>
#include <ros/ros.h>
#include <rovio_shared/rovio_position.h>
#include <raptor/polar_histogram.h>
#include <raptor/position_request_srv.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>

raptor_waypointer_testbench::raptor_waypointer_testbench()
{

  //Provide a location service
  give_loc = node.advertiseService("rovio_position_test", &raptor_waypointer_testbench::fake_location,this);
  //Register the waypoint set service
  set_loc = node.serviceClient<raptor::position_request_srv>("set_raptor_waypoint");
  //Register the vector-get service
  get_vec = node.serviceClient<raptor::polar_histogram>("vector_output_waypointer");
  
  curr_x = 100;
  curr_y = 200;
  curr_theta = 0;
  ROS_INFO("Testbench initialized.");
}

raptor_waypointer_testbench::~raptor_waypointer_testbench()
{

}

bool raptor_waypointer_testbench::fake_location(rovio_shared::rovio_position::Request &req, rovio_shared::rovio_position::Response &resp)
{
  resp.x = curr_x;
  resp.y = curr_y;
  resp.theta = curr_theta;
  return true;
}

bool raptor_waypointer_testbench::set_waypoint(int16_t x, int16_t y, float value, float time)
{
  raptor::position_request_srv srv;
  srv.request.x = x;
  srv.request.y = y;
  srv.request.val = value;
  srv.request.time_sec = time;
  set_loc.call(srv);
  ROS_INFO("Vector-set called!");
  return true;
}

void raptor_waypointer_testbench::print_output()
{
  raptor::polar_histogram srv;
  get_vec.call(srv);
  ROS_INFO("Vector-get called!");
}

int main(int argc, char **argv)
{
  
  ros::init(argc,argv,"raptor_waypointer_testbench");
  
  raptor_waypointer_testbench waypointer_testbench;
  
  ros::Rate loop_rate(1);
    // continue until a ctrl-c has occurred
    while (ros::ok())
    {
      ros::spinOnce();
      for(int i = 0;i<15;i++)
      {
	loop_rate.sleep();
      }
      ROS_INFO("Starting test routine.");
      waypointer_testbench.set_waypoint(100,100,10,2);
      waypointer_testbench.print_output();
      loop_rate.sleep();
      //waypointer_testbench.set_waypoint(-100,100,10,2);
      waypointer_testbench.print_output();
      loop_rate.sleep();
      loop_rate.sleep();
      //loop_rate.sleep();
      waypointer_testbench.print_output();
      //waypointer_testbench.set_waypoint(200,100,10,2);
      waypointer_testbench.print_output();
      ROS_INFO("End test routine.");
      for(int i = 0;i<15;i++)
      {
	//loop_rate.sleep();
      }
    }
    return EXIT_SUCCESS;
}