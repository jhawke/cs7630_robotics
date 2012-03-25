/*!
 * \file raptor_mcom_testbench.h
 * \brief Motor controller routine tester
 *
 * raptor_mcom creates a node that subscribes to various behavioral nodes and generates a motor command to the rovio_move node.
 *
 * \author omernick
 * \date March 22,2012
 */

#ifndef RAPTOR_MCOM_TESTBENCH_H_
#define RAPTOR_MCOM_TESTBENCH_H_

#include <ros/ros.h>
#include <raptor/polar_histogram.h>
#include <raptor/obstacle_histogram.h>
#include <raptor/twist_srv.h>
#include <string>
#include <vector>
#include <boost/array.hpp>

#define NUM_TESTS 13

/*!
 * \class raptor_mcom_testbench
 * \brief Provides direct communication to the Rovio to control the motors.
 *
 * The move_controller handles communication to the Rovio's motor devices. ROS nodes and topics are created and maintained within this object.
 */
class raptor_mcom_testbench
{
public:
  
  raptor_mcom_testbench();
  ~raptor_mcom_testbench();
  
private:

  ros::NodeHandle node;
  ros::ServiceServer testgen_1;
  ros::ServiceServer testgen_2;
  ros::ServiceServer testgen_3;
  bool move_com(raptor::twist_srv::Request &req, raptor::twist_srv::Response &res);
  bool tester_1(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res);
  bool tester_2(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res);
  bool tester_obs(raptor::obstacle_histogram::Request &req, raptor::obstacle_histogram::Response &res);
  int test_index;
  int test_index2;
  int test_index3;
  ros::ServiceServer output_srv;
  
  

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