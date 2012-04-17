/*!
 * \file raptor_absmove.h
 * \brief Waypoint generator behavior
 *
 * raptor_absmove generates a desired heading input to the vector sum controller of the raptor system.
 *
 * \author omernick
 * \date March 22,2012
 */


#ifndef _RAPTOR_ABSMOVE_H_
#define _RAPTOR_ABSMOVE_H_

#include <raptor/twist_srv.h>
#include <math.h>
#include <ros/ros.h>
#include <rovio_shared/rovio_position.h>
#include <raptor/polar_histogram.h>
#include <raptor_commander/abs_pos_req.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>
#include <std_msgs/Float32.h>

class raptor_absmove
{

public:

raptor_absmove();
~raptor_absmove();

private:

ros::NodeHandle node;
ros::ServiceClient location_request;
ros::Subscriber location_setter;
ros::ServiceServer vector_output;
ros::Subscriber absmove_volume;

void register_new_location(const raptor_commander::abs_pos_req::ConstPtr &msg);
bool give_location(raptor::polar_histogram::Request &req, raptor::polar_histogram::Response &res);
void adjust_gain(const std_msgs::Float32::ConstPtr& msg);

float volume;

bool loc_is_registered;
int16_t goal_x;
int16_t goal_y;
double	stop_time;
float	goal_val;
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