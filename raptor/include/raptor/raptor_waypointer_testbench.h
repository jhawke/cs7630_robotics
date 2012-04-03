/*!
 * \file raptor_waypointer_testbench.h
 * \brief Waypoint generator behavior
 *
 * raptor_waypointer generates a desired heading input to the vector sum controller of the Raptor system.
 *
 * \author omernick
 * \date March 22,2012
 */

#ifndef RAPTOR_WAYPOINTER_TESTBENCH_H_
#define RAPTOR_WAYPOINTER_TESTBENCH_H_

#include <rovio_shared/twist_srv.h>
#include <math.h>
#include <ros/ros.h>
#include <rovio_shared/rovio_position.h>
#include <raptor/polar_histogram.h>
#include <raptor/position_request_srv.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>

class raptor_waypointer_testbench
{
  
public:
  
  raptor_waypointer_testbench();
  ~raptor_waypointer_testbench();
  int16_t curr_x;
  int16_t curr_y;
  float   curr_theta;
  bool set_waypoint(int16_t x, int16_t y, float value, float time);
  void print_output();
  
private:
  
  ros::NodeHandle node;
  ros::ServiceClient set_loc;
  ros::ServiceServer give_loc;
  
  ros::ServiceClient get_vec;

  
  bool fake_location(rovio_shared::rovio_position::Request &req, rovio_shared::rovio_position::Response &resp);
  
};

/*!
 * Creates and runs the mcom node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::initobst
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif