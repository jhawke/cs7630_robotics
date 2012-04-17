/*!
 * \file raptor_mcom.cpp
 * \brief Motor controller routine
 *
 * raptor_mcom creates a node that subscribes to various behavioral nodes and generates a motor command to the rovio_move node.
 *
 * \author omernick
 * \date March 22,2012
 */

#include <rovio_shared/twist_srv.h>
#include <math.h>
#include <ros/ros.h>
#include <rovio_shared/man_drv.h>
#include <rovio_shared/man_drv_srv.h>
#include <raptor/raptor_mcom.h>
#include <raptor/polar_histogram.h>
#include <raptor/obstacle_histogram.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <raptor/light_level_srv.h>
#include <string>
#include <limits.h>
#include <boost/array.hpp>

#define NUM_BEH_MODS 		6

#define LOOP_RATE 		1

//FOV parameters. Needs to be synced with histogram message types.
#define FOV_START 		159
#define FOV_END			200
#define FOV_CENTER		180
#define FOV_SIZE		41
#define BEHAVIORAL_SUBDIVISIONS	360

//Forward speed commands
#define FORWARD_SPEED_MAX	1.0
#define MAX_INPUT_LEVEL		100
#define MIN_INPUT_LEVEL		-100

//empiricially determine these.
#define CONST_P			.03
#define CONST_I			.00

#define LL_FINDLIGHT		1
#define LL_RELMOVE		2
#define LL_NONE			0

#define MIN_MCOM		6

#define PI 3.14159

using namespace std;

raptor_mcom::raptor_mcom()	
{
  low_level_active = LL_NONE;
  //This is where the behavioral modules get registered.
  behavioral_modules.resize(NUM_BEH_MODS);
  
  
  //This is where the obstacle detecter is registered.
  obstacle_detector = node.serviceClient<raptor::obstacle_histogram>("raptor_obs_det_srv");
  
  //This is where the low-level switch is registered.
  low_level_switch = node.subscribe<std_msgs::Int32>("mcom_bright_positioner",1,&raptor_mcom::lls_callback,this);
  low_level_rel = node.subscribe<std_msgs::Float32>("mcom_ll_relmove",1,&raptor_mcom::lls_relmove_callback,this);
  low_level_finished = node.advertise<std_msgs::Int32>("bright_positioner_done",10);
  /**************************************/
  //TESTBENCH CODE BELOW.
  
  behavioral_modules[0] = node.serviceClient<raptor::polar_histogram>("raptor_relmove_srv");
  behavioral_modules[1] = node.serviceClient<raptor::polar_histogram>("raptor_absmove_srv"); 
  behavioral_modules[2] = node.serviceClient<raptor::polar_histogram>("raptor_stalk_srv");
  behavioral_modules[3] = node.serviceClient<raptor::polar_histogram>("raptor_find_dark_srv");
  behavioral_modules[4] = node.serviceClient<raptor::polar_histogram>("mcom_tester_1"); 
  behavioral_modules[5] = node.serviceClient<raptor::polar_histogram>("mcom_tester_2"); 
  /**************************************/
  
  light_level_finder = node.serviceClient<raptor::light_level_srv>("raptor_full_bright_srv");
  //Publish drive command data.
  drive_commander = node.serviceClient<rovio_shared::twist_srv>("rovio_move_srv");
  man_driver = node.serviceClient<rovio_shared::man_drv_srv>("rovio_man_drv");
  
  //Initialize the error sum to 0.
  sigma = 0;
  
  ROS_INFO("Raptor MCOM node initialized.");
}

raptor_mcom::~raptor_mcom()
{
  
}

void raptor_mcom::lls_callback(const std_msgs::Int32::ConstPtr &msg)
{
     
    if(msg->data>0)
    {
     current_heading = 0;
     low_level_active = LL_FINDLIGHT;
     best_heading_found = false;
    }
}

void raptor_mcom::lls_relmove_callback(const std_msgs::Float32::ConstPtr &msg)
{
  float moves_float;
  int theta_degrees;
  if(msg->data>=0)
  {
    theta_degrees = 360*(msg->data/(2*PI));
    theta_degrees = theta_degrees%360;
    moves_remaining = round(theta_degrees*(NUM_MOVES/360.0));
     low_level_active = LL_RELMOVE;
     ROS_DEBUG("There was a ll-relmove request. Theta was %f, deg was %d, moves is %d",msg->data,theta_degrees,moves_remaining);
  }
  else
  {
   ROS_ERROR("The low-level absmove does not accept negative numbers."); 
  }

}

void raptor_mcom::find_best_light()
{
  if(!best_heading_found)
    {
      raptor::light_level_srv srv;
      if(light_level_finder.call(srv))
      if(true)
      {
	light_levels[current_heading]=srv.response.light_level;
	//light_levels[current_heading]=current_heading;
	ROS_INFO("Light level at %d is %d",current_heading,light_levels[current_heading]);
	current_heading++;
      }
      if(current_heading==NUM_MOVES)
      {
	int loc;
	int val = -1;
	int temp;
	for(int i = 0; i<NUM_MOVES;i++)
	{
	  temp = light_levels[i];
	  ROS_INFO("LLi is %d comp to %d.",temp,val);
	  
	  if(temp>val)
	  {
	    ROS_INFO("Comp hit.");
	   loc = i;
	   val =temp;
	  }
	  
	}
	current_heading = loc;
	best_heading_found=true;
	ROS_INFO("Found best heading at %d",current_heading);
      }
    }
    else
    {
      current_heading--;
    }
    
    if(current_heading>=0)
    {
      rovio_shared::man_drv_srv srv;
      srv.request.drive = 17;
      srv.request.speed = 1;
      man_driver.call(srv);
    }
    else
    {
      low_level_active = LL_NONE;
      std_msgs::Int32 msg;
      msg.data = 1;
      low_level_finished.publish(msg);
    }
}

void raptor_mcom::abs_rotation()
{
  if(moves_remaining>0)
  {
      rovio_shared::man_drv_srv srv;
      srv.request.drive = 17;
      srv.request.speed = 1;
      man_driver.call(srv);
  }
  moves_remaining--;
  ROS_DEBUG("Moves remaining: %d",moves_remaining);
  if(moves_remaining<=0)
  {
      rovio_shared::man_drv_srv srv;
      srv.request.drive = 1;
      srv.request.speed = 1;
      man_driver.call(srv);
    
   low_level_active=LL_NONE; 
  }
}

void raptor_mcom::system_heartbeat()
{
  ROS_DEBUG("\n***");
  if(low_level_active>0)
  {
    ROS_DEBUG("Low level call made...");
    switch(low_level_active)
    {
      case LL_FINDLIGHT:
	find_best_light();
	break;
      case LL_RELMOVE:
	abs_rotation();
	break;
    }
  }
  else
  {
    vector<int16_t> behavioral_commands(BEHAVIORAL_SUBDIVISIONS,0);
    boost::array<int16_t,360> behavioral_inputs = {0};
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
	}  bool tester_obs(raptor::obstacle_histogram::Request &req, raptor::obstacle_histogram::Response &res);
    }
    
    ROS_DEBUG("Maximum location is %d with value %d",max_loc,max_vect);
    //We have a goal. Now generate a drive command.
    int goal_loc = max_loc;
    rovio_shared::twist_srv d_com;
    d_com.request.linear.x = 0; //x
    d_com.request.linear.y = 0; //y
    d_com.request.linear.z = 0; //z
    //If the command is within the FOV, do obstacle avoidance to find the best non-obstacle path and set to drive forward.
    if(is_in_fov)
    {
      ROS_DEBUG("Obstacle detection routine running...");
      raptor::obstacle_histogram obst_srv;
      if(obstacle_detector.call(obst_srv))
      {
	ROS_DEBUG("ODR returned. Processing.");
	boost::array<int16_t,FOV_SIZE> is_free = obst_srv.response.hist;
	//boost::array<int16_t,FOV_SIZE> is_free = {0};
	//Briefly check if the entire histogram is full.
	bool is_all_block = true;
	for(int i = 0; i<FOV_SIZE; i++)
	{
	  is_all_block = is_all_block&&(!is_free[i]);
	}
	if(!is_all_block)
	{
	  ROS_DEBUG("FOV is not all blocked.");
	  //First, we know that a path exists, so we can drive forward.
	  d_com.request.linear.x = FORWARD_SPEED_MAX*(max_vect/(float)(MAX_INPUT_LEVEL-MIN_INPUT_LEVEL));
	  //We want to find the nearest non-obstacled path to the goal.
	  //Bias is toward a path in the FOV, only going out-of-FOV if no non-obstacled path exists.
	  goal_loc = max_loc-FOV_START;
	  int closest=1000;
	  for(int i = 0; i<41; i++)
	  {
	    if(is_free[i])
	    {
	      goal_loc = goal_loc<(abs(goal_loc-i))?closest:i;
	    }
	  }
	  goal_loc+=FOV_START;
	}
	else	//This is what happens if the entire fov is obstacled.
	{
	  ROS_DEBUG("FOV is all blocked.");
	  goal_loc = (max_loc<FOV_CENTER)?(max_loc-FOV_SIZE):(max_loc+FOV_SIZE); //Shift CCW/CW by one camera frame. 
	} 
      }
    }
      //Calculate the angular rotational command using PI control.
      ROS_DEBUG("Final heading: %d",goal_loc);
      
      double ang_rot = (max_loc-FOV_CENTER)*CONST_P+sigma*CONST_I;
      sigma+=((max_loc-FOV_CENTER));
      
      if(ang_rot>0.99)
      {
	ang_rot = 0.99;
      }
      if(ang_rot<-0.99)
      {
	ang_rot= -0.99;
      }
      
      ROS_DEBUG("ang_rot is %.4g",ang_rot);
      
      d_com.request.angular.x = 0.0;
      d_com.request.angular.y = 0.0;
      d_com.request.angular.z = ang_rot;
      
      ROS_DEBUG("Twist command is l:[%g,%g,%g] r:[%g,%g,%g]",d_com.request.linear.x,d_com.request.linear.y,d_com.request.linear.z,d_com.request.angular.x,d_com.request.angular.y,d_com.request.angular.z);
      
      drive_commander.call(d_com);
    } 
    
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
      mcom.system_heartbeat();
      loop_rate.sleep();
    }
    return EXIT_SUCCESS;
  }