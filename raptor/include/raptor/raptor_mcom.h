/*!
 * \file raptor_mcom.h
 * \brief Motor controller routine
 *
 * raptor_mcom creates a node that subscribes to various behavioral nodes and generates a motor command to the rovio_move node.
 *
 * \author omernick
 * \date March 22,2012
 */

#ifndef RAPTOR_MCOM_H_
#define RAPTOR_MCOM_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <rovio_shared/man_drv.h>
#include <raptor/polar_histogram.h>
#include <string>
#include <vector>

/*!
 * \class raptor_mcom
 * \brief Provides direct communication to the Rovio to control the motors.
 *
 * The move_controller handles communication to the Rovio's motor devices. ROS nodes and topics are created and maintained within this object.
 */
class raptor_mcom
{
public:
  
  raptor_mcom();
  ~raptor_mcom();
  void system_heartbeat();
  
private:

  ros::NodeHandle node;
  ros::ServiceClient obstacle_detector;
  ros::ServiceClient drive_commander;
  std::vector<ros::ServiceClient> behavioral_modules;
  double sigma;
  
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