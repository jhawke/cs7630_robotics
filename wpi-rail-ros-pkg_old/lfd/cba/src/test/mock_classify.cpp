/*
 * cba.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: rctoris
 */

#include <iostream>
#include <lfd_common/classification_point.h>
#include <lfd_common/conf_classification.h>
#include <ros/ros.h>
#include <time.h>
#include <vector>

using namespace std;

vector<float*> data; /*!< the data set */

bool classify_callback(lfd_common::conf_classification::Request &req, lfd_common::conf_classification::Response &resp)
{
  // break them into 5 labels
  if (req.s.state_vector.at(2) < 5)
    resp.l = 5;
  else if (req.s.state_vector.at(2) < 10)
    resp.l = 10;
  else if (req.s.state_vector.at(2) < 15)
    resp.l = 15;
  else if (req.s.state_vector.at(2) < 20)
    resp.l = 20;
  else
    resp.l = 25;

  // choose their decision boundary
  if (req.s.state_vector.at(1) < 13)
    resp.db = 1;
  else
    resp.db = 2;

  // let confidence be based on the number of points out of the 1562575 possible states
  resp.c = ((float)data.size()) / 1562575.0;

  // now randomly change the label to create some noise
  if ((rand() % 50) == 0)
  {
    resp.l = ((rand() % 5) + 1) * 5;
    resp.c /= 2.0;
  }

  return true;
}

void add_point_callback(const lfd_common::classification_point::ConstPtr &msg)
{

  // check if we have a new point
  for (uint i = 0; i < data.size(); i++)
    if (data.at(i)[0] == msg->s.state_vector.at(0) && data.at(i)[1] == msg->s.state_vector.at(1) && data.at(i)[2]
        == msg->s.state_vector.at(2))
      return;

  // new point
  float *data_point = (float *)malloc(sizeof(float) * msg->s.state_vector.size());
  for (uint i = 0; i < msg->s.state_vector.size(); i++)
    data_point[i] = msg->s.state_vector.at(i);
  data.push_back(data_point);

  // check if the labels match (-100 is the default correction value)
  if (msg->l != -100)
    if (msg->s.state_vector.at(2) < 5)
      assert(msg->l == 5);
    else if (msg->s.state_vector.at(2) < 10)
      assert(msg->l == 10);
    else if (msg->s.state_vector.at(2) < 15)
      assert(msg->l == 15);
    else if (msg->s.state_vector.at(2) < 20)
      assert(msg->l == 20);
    else
      assert(msg->l == 25);
}

void change_point_callback(const lfd_common::classification_point::ConstPtr &msg)
{
  // check if we have this point
  for (uint i = 0; i < data.size(); i++)
    if (data.at(i)[0] == msg->s.state_vector.at(0) && data.at(i)[1] == msg->s.state_vector.at(1) && data.at(i)[2]
        == msg->s.state_vector.at(2))
    {
      cout << "\"Changed\" data point successfully!" << endl;
      return;
    }

  // point never found
  cout << "[ERROR]: Data point provided in 'change_point' does not exist!";
  exit(-1);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "mock_classify");
  // a handle for this ROS node
  ros::NodeHandle node;

  // create services and topics
  ros::ServiceServer classify = node.advertiseService("classify", classify_callback);
  ros::Subscriber add_point = node.subscribe<lfd_common::classification_point> ("add_point", -1, add_point_callback);
  ros::Subscriber change_point = node.subscribe<lfd_common::classification_point> ("change_point", -1,
                                                                                   change_point_callback);

  // used to randomly classify incorrectly
  srand(time(NULL));

  ROS_INFO("Mock Classifier Initialized");

  // run the node
  ros::spin();

  return EXIT_SUCCESS;
}
