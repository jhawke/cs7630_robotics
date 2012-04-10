/*
 * cba.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: rctoris
 */

#include <lfd_common/state.h>
#include <ros/ros.h>
#include <time.h>

using namespace std;

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "mock_state_publisher");
  // a handle for this ROS node
  ros::NodeHandle node;

  // published topics
  ros::Publisher update_state = node.advertise<lfd_common::state> ("state_listener", 1);

  // used to create random states
  srand(time(NULL));

  ROS_INFO("Mock State Publisher Initialized");

  // publish states continuously
  while (ros::ok())
  {
    // 3 dimensional state vector all in range [0,25)
    lfd_common::state s;
    for (int i = 0; i < 3; i++)
      s .state_vector.push_back(rand() % 25);
    update_state.publish(s);
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
