/*!
 * \file raptor_relmove.h
 * \brief Waypoint generator behavior
 *
 * raptor_relmove generates a desired heading input to the vector sum controller of the raptor system.
 *
 * \author omernick
 * \date March 22,2012
 */


#ifndef _RAPTOR_RELMOVE_H
#define _RAPTOR_RELMOVE_H

#include <raptor/twist_srv.h>
#include <math.h>
#include <ros/ros.h>
#include <rovio_shared/rovio_position.h>
#include <raptor/polar_histogram.h>
#include <raptor_commander/rel_pos_req.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>
#include <std_msgs/Float32.h>

class raptor_relmove
{

public:

raptor_relmove();
~raptor_relmove();

private:

ros::NodeHandle node;
ros::ServiceClient location_request;
ros::Subscriber location_ll_setter;
ros::Subscriber location_setter;
ros::Publisher mcom_ll_relmover;
ros::ServiceServer vector_output;
ros::Subscriber relmove_vol;

void register_new_location(const raptor_commander::rel_pos_req::ConstPtr &msg);
bool give_location(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res);
void register_new_location_ll(const raptor_commander::rel_pos_req::ConstPtr &msg);

void adjust_gain(const std_msgs::Float32::ConstPtr& msg);

float volume;

float goal_theta;
double	stop_time;
float	goal_val;
bool loc_is_registered;
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