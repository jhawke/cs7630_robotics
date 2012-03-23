/*!
 * \file raptor_mcom.cpp
 * \brief Motor controller routine
 *
 * raptor_mcom creates a node that subscribes to various behavioral nodes and generates a motor command to the rovio_move node.
 *
 * \author omernick
 * \date March 22,2012
 */

#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>
#include <rovio_shared/man_drv.h>
#include <raptor/raptor_mcom.h>
#include <raptor/polar_histogram.h>
#include <raptor/obstacle_histogram.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>

#define NUM_BEH_MODS 		0
#define LOOP_RATE 		10
#define FOV_START 		159
#define FOV_END			199
#define FOV_CENTER		179
#define FOV_SIZE		40
#define BEHAVIORAL_SUBDIVISIONS	360

#define FORWARD_SPEED		1.0

//empiricially determine these.
#define CONST_P			.05;
#define CONST_I			.01;

using namespace std;

raptor_mcom::raptor_mcom()
{
  //This is where the behavioral modules get registered.
  behavioral_modules.resize(NUM_BEH_MODS);
 
  //Publish drive command data.
  drive_commander = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
  
  //Initialize the error sum to 0.
  sigma = 0;
  
  ROS_INFO("Raptor MCOM node initialized.");
}

raptor_mcom::~raptor_mcom()
{
  
}

void raptor_mcom::system_heartbeat()
{
  vector<int16_t> behavioral_commands(BEHAVIORAL_SUBDIVISIONS,0);
  boost::array<int16_t,360> behavioral_inputs;
  bool is_in_fov=false;
     //Get new commands here.
      for(vector<ros::ServiceClient>::size_type i = 0; i<behavioral_modules.size(); i++)
      {
	raptor::polar_histogram srv;
	if(behavioral_modules[i].call(srv))
	{
	  behavioral_inputs = srv.response.hist;
	  
	  //Sum the commands. Let the commands be from -100 to 100.
	  for(vector<int16_t>::size_type i = 0; i<BEHAVIORAL_SUBDIVISIONS;i++)
	  {
	    //If behavioral_commands+new_command<-32768, set to -32768. Just in case.
	    if((behavioral_commands[i]+behavioral_inputs[i])<-32768)
	    {
		behavioral_commands[i] = -32768;
	    }
	    else
	    {
		behavioral_commands[i] += behavioral_inputs[i];
	    }
	  }
	  //End summing loop
	}
      }
    int16_t max_vect= -32678;
    int max_loc=0;
    
    for(int i = 0; i<FOV_START;i++)
    {
      if(behavioral_commands[i]>=max_vect)
      {
	max_vect = behavioral_commands[i];
	max_loc = i;
      }
    }
    
    for(uint i=(FOV_END+1); i<behavioral_commands.size();i++)
    {
      if(behavioral_commands[i]>=max_vect)
      {
	max_vect = behavioral_commands[i];
	max_loc = i;
      }
    }
    
    for(int i=FOV_START;i<=FOV_END;i++)
    {
      if(behavioral_commands[i]>max_vect)
      {
	max_vect = behavioral_commands[i];
	max_loc = i;
	is_in_fov = true;
      }
      if((behavioral_commands[i]==max_vect)&&(((max_loc-FOV_CENTER)*(max_loc-FOV_CENTER))>((i-FOV_CENTER)*(i-FOV_CENTER))))	//Minimize the square of the distance to FOV_CENTER instead of worrying about calls to abs().
      {
	max_vect = behavioral_commands[i];
	max_loc = i;
	is_in_fov = true;
      }
    }
     //We have a goal. Now generate a drive command.
     int goal_loc;
     geometry_msgs::Twist d_com;
     d_com.linear.x = 0; //x
     d_com.linear.y = 0; //y
     d_com.linear.z = 0; //z
     //If the command is within the FOV, do obstacle avoidance to find the best non-obstacle path and set to drive forward.
     if(is_in_fov)
     {
      raptor::obstacle_histogram obst_srv;
      if(obstacle_detector.call(obst_srv))
      {
	  boost::array<int16_t,FOV_SIZE> is_free = obst_srv.response.hist;
	  //Briefly check if the entire histogram is full.
	  bool is_all_block = false;
	  for(int i = 0; i<FOV_SIZE; i++)
	  {
	      is_all_block = is_all_block||is_free[i];
	  }
	  if(!is_all_block)
	  {
	    //First, we know that a path exists, so we can drive forward.
	    d_com.linear.x = FORWARD_SPEED;
	    //We want to find the nearest non-obstacled path to the goal.
	    //Bias is toward a path in the FOV, only going out-of-FOV if no non-obstacled path exists.
	    int down=0;
	    int up=0;
	    int start = max_loc-FOV_START;
	    while(((max_loc-down)>=FOV_START)||(is_free[start-down]))
	    {
	      down++;
	    }
	    while(((max_loc+up)>=FOV_END)||(is_free[start+up]))
	    {
	      up++;
	    }
	    if(down<up)
	    {
	      goal_loc = max_loc-down;
	    }
	    else
	    {
	      goal_loc = max_loc+up;
	    }
	  }
	  else	//This is what happens if the entire fov is obstacled.
	  {
	     goal_loc = (max_loc<180)?(max_loc-40):(max_loc+40); //Shift CCW/CW by one camera frame. 
	  } 
     }
     //Calculate the angular rotational command using PI control.
     
     double ang_rot = (max_loc-180)*CONST_P+sigma*CONST_I;
     sigma+=((max_loc-180));
     
     if(ang_rot>0.99)
     {
       ang_rot = 0.99;
     }
     if(ang_rot<-0.99)
     {
	ang_rot= -0.99;
     }
     
     d_com.angular.z = ang_rot;
}

int main(int argc, char **argv)
{

  // initialize ROS and the node
  ros::init(argc, argv, "raptor_mcom");

  // initialize the Rovio controller
  raptor_mcom mcom;

  // update at 5 Hz
  ros::Rate loop_rate(LOOP_RATE);
  // continue until a ctrl-c has occurred
  while (ros::ok())
  {
    ros::spinOnce();
 
     
    loop_rate.sleep();
  }
}