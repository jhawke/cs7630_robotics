/*!
 * \file raptor_drive_testbench.h
 * \brief Motor driver testbench routine
 *
 * Testbench for integration between mcom and the drive subsystem.
 *
 * \author omernick
 * \date March 22,2012
 */

#ifndef RAPTOR_DRIVE_TESTBENCH_H_
#define RAPTOR_DRIVE_TESTBENCH_H_

#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>
#include <rovio_shared/man_drv.h>
#include <raptor/polar_histogram.h>
#include <raptor/obstacle_histogram.h>
#include <raptor/twist_srv.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>

class raptor_drive_testbench
{

public:

  raptor_drive_testbench();
  ~raptor_drive_testbench();

private:

  ros::NodeHandle node;
  ros::ServiceServer testgen_1;
  ros::ServiceServer testgen_3;

  bool tester_1(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res);
  bool tester_obs(raptor::obstacle_histogram::Request &req, raptor::obstacle_histogram::Response &res);

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