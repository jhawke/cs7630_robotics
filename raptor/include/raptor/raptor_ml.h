/*!
 * \file raptor_ml.h
 * \brief RAPTOR machine learning node.
 *
 * Creates a node that determines follow distance through basic machine learning principles.
 *
 * \author omernick
 * \date 4/7/12
 */

#ifndef _RAPTOR_ML_H_
#define _RAPTOR_ML_H_

#include <ros/ros.h>
#include <raptor/state_set_srv.h>
#include <raptor/distance_adv_srv.h>
#include <std_msgs/Bool.h>

#include <math.h>
#include <boost/array.hpp>
#include <map>
#include <string>

#define MARKOV_ARRAY_SIZE 480

class raptor_ml{

public:
  raptor_ml();
  ~raptor_ml();
  
private:
  typedef boost::array<float, MARKOV_ARRAY_SIZE> Markov_Array;
  
  bool recieve_new_state(raptor::state_set_srv::Request &req, raptor::state_set_srv::Response &res);
  bool advise_distance(raptor::distance_adv_srv::Request &req, raptor::distance_adv_srv::Response &res);
  void reward_callback(const std_msgs::Bool::ConstPtr &msg);
  
  std::map<int,Markov_Array> markov_holder;
  
  int model_id;
  int current_state;
  int last_state;
  bool went_forward;
  
  ros::NodeHandle node;
  ros::Subscriber reward_listener;
  ros::ServiceServer state_getter,dist_advisor;
};

int main(int argc, char** argv);

#endif