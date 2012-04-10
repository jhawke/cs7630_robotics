  
/*
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Nao Natural Interface Teleoperation in ROS
 *
 * Copyright December 2010, H. Bener SUAY 
 * 
 * Worcester Polytechnic Institute
 *
 * wpi.edu/~benersuay
 * benersuay@wpi.edu
 */

/* Release 0.0.1
 * This code is the fusion of Nao Remote of Albert-Ludwig University
 * and OpenNI_SampleUserTracker of The University of Tokyo, with, of course, modifications.
 */

/* It doesn't show the video as SampleUserTracker does, so you might want to run the UserTracker as well to see your self.
 * Also, it takes more time to actually start tracking the user because of the low publishing rate.
 * IMPORTANT: Wait in Psi Pose until this code prints out "Calibration complete, start tracking user", NOTE THAT IT IS SLOWER THAN Sample-NiUserTracker!!!
 */

/*
 * Main purpose is to gets data from Microsoft Kinect, and to remap 
 * the data to teleoperation commands that nao_ctrl understands.
 */

/* A lot To Do:
 * - More comments (it's never enough)
 * - Speed up user calibration, make it independent of Nao's pubRate (see main).
 * - Fix wrong user enumeration
 * - Right now arm angles are published as HeadAngles, a new ArmAngles message type should be defined in nao_ctrl package
 * - Some messages are still published as Button messages, new State messages should be defined in nao_ctrl package
 * - Find a better way for Body Control / Gaze Direction Control switch
 * - What is the smart way to add voice control to this (i.e. when someone hacks the audio driver)?
 * - Fix false detections
 * - Fix 2 seconds waitings with a nice solution
 * - Build a gesture library
 * - Use error codes 
 * - Find more things to do.
 */

/* Known issues:
 * - Sometimes OpenNI recognizes the skeleton with high confidence but still in weird positions, which confuses
 *   this code and causes false control mode switches.
 * - Calibration takes way too long.
 */   

// General Includes
#include <math.h>

// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>


// Nao Includes
#include <std_msgs/String.h> // For speech messages to Nao
#include <geometry_msgs/Twist.h> // For lin. and ang. motion messages to Nao
#include <nao_ctrl/MotionCommandBtn.h> // For messages such as stiffness on/of, control enable/disable
#include <nao_ctrl/HeadAngles.h> // Originally this was created to change head pitch / yaw angle, but here it's
                                 // used for both head AND arm angles.

// Prime Sense Includes
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

using std::string;

// Prime Sense Variables and objects begin
xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";
// Prime Sense Variables and objects end

std_msgs::String naosay;
XnPoint3D init_usr_loc; // Previous body location of the user

/*           left_safe_line
 *           :     
 *           :    right_safe_line
 *           :    :
 *        -----------    .....anterior_control_line
 *       |             |
 *       |    -----    | .....anterior_safe_line
 *       |   |     |cd |
 *       |   |  O  |-->|
 *       |   |_____|   | .....posterior_safe_line
 *       |      |cd    |
 *       |__ ___v_____ | .....posterior_control_line
 *       :             :
 *       :             right_control_line
 *       :
 *       left_control_line
 *
 * cd: control distance, see define section.
 *
 * This is basically how the control pad works:
 * - Point O is where you turn on the motors of Nao
 * - If you step somewhere between anterior_safe_line and anterior_control_line, the robot walks forward
 * - If you rotate your shoulders between safety_rot and control_rot the robot rotates its body.
 *
 */

// Initially all lines are zero. They will be set when the user turns the motors ON.
float anterior_safe_line = 0.0;
float posterior_safe_line = 0.0;
float right_safe_line = 0.0;
float left_safe_line = 0.0;
float safe_angle = 0.0; 

float anterior_control_line = 0.0;
float posterior_control_line = 0.0;
float right_control_line = 0.0;
float left_control_line = 0.0;
float control_angle = 0.0;

XnPoint3D init_usr_RS; // Initial location of the user's Right Shoulder (for rotation of the body)
XnPoint3D init_usr_LS; // Initial location of the user's Left Shoulder (for rotation of the body)

// OpenNI Check Fault
#define CHECK_RC(nRetVal, what)						\
  if (nRetVal != XN_STATUS_OK)						\
    {									\
      printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));	\
      return nRetVal;							\
    }

#define my_pi 3.141592653589793 // Define the PI number for vector calculations
#define walking_speed 0.3 // Walking speed of the robot [0.0 - 1.0]
#define rotating_speed 0.3 // Angular speed of the robot

#define control_dist 300.0 // Distance to step in order to control the robot
#define safety_dist 200.0 // Distance where user can reset the speed of the robot

#define control_rot 1.57 // Angle to turn, in order to rotate the torso of the robot
#define safety_rot 0.3 // Angle up to which the user can reset the rotational speed of the robot

#define rot_constant 0.04 // Tune this!

class TeleopNaoNi
{
public:
  TeleopNaoNi();
  void pubMsg();
  ros::NodeHandle nh; // Node that will publish messages 
  ros::NodeHandle privateNh; // Initialized but not used in this code. Inherited from Albert-Ludwig University's implementation

  bool m_enabled; // Motors enabled
  bool m_teleopEnabled; // Teleoperation enabled - Body Control (mutually exclusive with attentionEnabled)
  bool m_attentionEnabled; // Attention enabled - Gaze Direction Control (mutually exclusive with bodyControl)
  bool m_commandEnabled; // Apply command (i.e. apply the defined command, in this case walk until receiving STOP command)
  int m_xAxis; // Not Used
  int m_yAxis; // Not Used
  int m_turnAxis; // Not Used
  int m_headYawAxis; // Not Used
  int m_headPitchAxis; // Not Used
  int m_crouchBtn; // Button number which makes Nao crouch
  int m_initPoseBtn; // Button number wich makes Nao to stand up and take the initial pose
  int m_enableBtn;  // Button number which makes Nao to enable it's motors
  int m_modifyHeadBtn; // Not Used
  int m_startScanBtn; // Not Used
  int m_stopScanBtn; // Not Used
  double m_maxVx; // Max. Linear Velocity in X
  double m_maxVy; // Max. Linear Velocity in Y
  double m_maxVw; // Max. Angular Velocity
  double m_maxHeadYaw; // Max. Head Yaw Angle
  double m_maxHeadPitch; // Max. Head Pitch Angle
  
  ros::Publisher m_movePub; // Publishes the lin. and ang. motions
  ros::Publisher m_moveBtnPub; // Publishes button numbers
  ros::Publisher m_headPub; // Publishes head angles
  ros::Publisher m_speechPub; // Publishes speech string
  ros::Publisher m_lShoulderPub; // Publishes left shoulder angles
  ros::Publisher m_rShoulderPub; //           right shoulder angles
  ros::Publisher m_lElbowPub; // Publishes left elbow angles
  ros::Publisher m_rElbowPub; //           right elbow angles
  geometry_msgs::Twist m_motion; // Linear velocities for Nao's navigation
  nao_ctrl::HeadAngles m_headAngles; // Head angles of Nao
  nao_ctrl::HeadAngles m_lShoulderAngles; // Left shoulder angles
  nao_ctrl::HeadAngles m_rShoulderAngles; // Right shoulder angles
  nao_ctrl::HeadAngles m_lElbowAngles; // Left elbow angles
  nao_ctrl::HeadAngles m_rElbowAngles; // Right elbow angles
};

// Initialization list of object TeleopNaoNi
TeleopNaoNi::TeleopNaoNi()
  :	privateNh("~"), m_enabled(false), m_teleopEnabled(false), m_attentionEnabled(false), m_commandEnabled(false),
   m_xAxis(3), m_yAxis(2), m_turnAxis(0), m_headYawAxis(4),	m_headPitchAxis(5),
   m_crouchBtn(9), m_initPoseBtn(0), m_enableBtn(8), m_modifyHeadBtn(5),
   m_startScanBtn(2), m_stopScanBtn(3),
   m_maxVx(1.0), m_maxVy(1.0), m_maxVw(0.5),
   m_maxHeadYaw(2.0943), m_maxHeadPitch(0.7853)
{
	privateNh.param("axis_x", m_xAxis, m_xAxis);
	privateNh.param("axis_y", m_yAxis, m_yAxis);
	privateNh.param("axis_turn", m_turnAxis, m_turnAxis);
	privateNh.param("axis_head_yaw", m_headYawAxis, m_headYawAxis);
	privateNh.param("axis_head_pitch", m_headPitchAxis, m_headPitchAxis);
	privateNh.param("btn_crouch", m_crouchBtn, m_crouchBtn);
	privateNh.param("btn_enable_control", m_enableBtn, m_enableBtn);
	privateNh.param("btn_head_mod", m_modifyHeadBtn, m_modifyHeadBtn);
	privateNh.param("btn_start_scan", m_startScanBtn, m_startScanBtn);
	privateNh.param("btn_stop_scan", m_stopScanBtn, m_stopScanBtn);
	privateNh.param("max_vx", m_maxVx, m_maxVx);
	privateNh.param("max_vy", m_maxVy, m_maxVy);
	privateNh.param("max_vw", m_maxVw, m_maxVw);

	privateNh.param("max_head_yaw", m_maxHeadYaw, m_maxHeadYaw);
	privateNh.param("max_head_pitch", m_maxHeadPitch, m_maxHeadPitch);

	// Initialize all velocities and angles at zero.
	m_motion.linear.x = m_motion.linear.y = m_motion.angular.z = 0;
	m_headAngles.yaw = 0.0;
	m_headAngles.pitch = 0.0;
	m_headAngles.absolute = 1.0;

	m_lShoulderAngles.yaw=0.0;
	m_lShoulderAngles.pitch=0.0;
	m_lShoulderAngles.absolute=1.0;

	m_rShoulderAngles.yaw=0.0;
	m_rShoulderAngles.pitch=0.0;
	m_rShoulderAngles.absolute=1.0;

	m_lElbowAngles.yaw=0.0;
	m_lElbowAngles.pitch=0.0;
	m_lElbowAngles.absolute=1.0;

	m_rElbowAngles.yaw=0.0;
	m_rElbowAngles.pitch=0.0;
	m_rElbowAngles.absolute=1.0;

	// Messages to publish (to be subscribed by nao_ctrl)
	m_movePub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); 
	m_moveBtnPub = nh.advertise<nao_ctrl::MotionCommandBtn>("motion_command_btn", 1);
	m_headPub = nh.advertise<nao_ctrl::HeadAngles>("head_angles", 1);
	m_speechPub = nh.advertise<std_msgs::String>("speech", 1);
	m_lShoulderPub = nh.advertise<nao_ctrl::HeadAngles>("lshoulder_angles",1);
	m_rShoulderPub = nh.advertise<nao_ctrl::HeadAngles>("rshoulder_angles",1);
	m_lElbowPub = nh.advertise<nao_ctrl::HeadAngles>("lelbow_angles",1);
	m_rElbowPub = nh.advertise<nao_ctrl::HeadAngles>("relbow_angles",1);	
}



//Publish motion message to Nao
void TeleopNaoNi::pubMsg(){
  // m_enabled = false in the beginning by default. See the initialization list of TeleopNaoNi.
  if (m_enabled){
    // Publish appropriate messages (to be subscribed by nao_ctrl)
    m_movePub.publish(m_motion); 
    m_headPub.publish(m_headAngles); 
    m_lShoulderPub.publish(m_lShoulderAngles);
    m_rShoulderPub.publish(m_rShoulderAngles);
    m_lElbowPub.publish(m_lElbowAngles);
    m_rElbowPub.publish(m_rElbowAngles);
  }
}

void stopWalking(TeleopNaoNi &robot){
  	robot.m_motion.linear.x = robot.m_motion.linear.y = robot.m_motion.angular.z = 0;
}


void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	printf("New User %d\n", nId);
	//naosay.data="New user";
	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
  //naosay.data="Lost user";
  printf("Lost user %d\n", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
  //naosay.data="Calibration started for user";
  printf("Calibration started for user %d\n", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
  if (bSuccess) {
    printf("Calibration complete, start tracking user %d\n", nId);
    //naosay.data="Calibration complete. Start tracking user";
    g_UserGenerator.GetSkeletonCap().StartTracking(nId);
  }
  else {
    printf("Calibration failed for user %d\n", nId);
    //naosay.data="Calibration failed for user";
    if (g_bNeedPose)
      g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
    else
      g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
  }
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    printf("Pose %s detected for user %d\n", strPose, nId);
    //naosay.data="Pose is detected for user";
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

// Get and return the location of the user (the origin point, for the control of Nao)
XnPoint3D getBodyLoc(XnUserID const& user, XnSkeletonJoint const& eJoint1){
  XnPoint3D pt;

  // Fault Check
  if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
    {
      printf("not tracked!\n");
      pt.X=0.0; pt.Y=0.0; pt.Z=0.0;
      return pt;
    }
  
  // Get joint position (this is supposed to be "the Torso point" of the skeleton)
  XnSkeletonJointPosition joint1;
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, eJoint1, joint1);

  // Make sure that all joints are found with enough confidence...
  if (joint1.fConfidence < 0.7)
    {
      pt.X=0.0; pt.Y=0.0; pt.Z=0.0;
      return pt;
    }
  
  pt = joint1.position; // Return the position
  return pt;
}

// Get and return the angle between two NOT JOINT limbs of the skeleton, e.g. angle between (hip to hip vector) ^ ( upper arm )
float getAngleBetweenLimbs(XnUserID const& user, XnSkeletonJoint const& eJoint1, XnSkeletonJoint const& eJoint2, XnSkeletonJoint const& eJoint3, XnSkeletonJoint const& eJoint4){

  // This function is different than getLimbAngle, in the sense that, it finds the angle of two disconnected
  // limbs (each limb considered as a vector).

  // In order to find the shoulder roll angles of Nao, we need to find the angle between
  // [Right Hip --> Left Hip] vector and [Right Shoulder --> Right Elbow] vector
 
  if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
    {
      printf("not tracked!\n");
      return -998.0; // Error code (though, it's not used yet)
    }
  
  // Get joint positions
  XnSkeletonJointPosition joint1, joint2, joint3, joint4;
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, eJoint1, joint1);
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, eJoint2, joint2);
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, eJoint3, joint3);
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, eJoint4, joint4);
  

  // Make sure that all joints are found with enough confidence...
  if (joint1.fConfidence < 0.6 && joint2.fConfidence < 0.6 && joint3.fConfidence < 0.6 && joint4.fConfidence < 0.6)
    {
      return -999.0; // Error code (though, it's not used yet)
    }
  
  XnPoint3D pt[4]; // Create four points
  pt[0] = joint1.position; // Vector 0, Point 0
  pt[1] = joint2.position; // Vector 0, Point 1
  pt[2] = joint3.position; // Vector 1, Point 0
  pt[3] = joint4.position; // Vector 1, Point 1

  XnVector3D v[2]; // Create two vectors

  // First vector
  // V_{RHIP,LHIP} = V_{LHIP} - V_{RHIP}
  v[0].X=pt[2].X-pt[3].X;
  v[0].Y=pt[2].Y-pt[3].Y;
  v[0].Z=pt[2].Z-pt[3].Z;

  // Second vector
  // V_{S,E} = V_{E} - V_{S}
  v[1].X=pt[0].X-pt[1].X;
  v[1].Y=pt[0].Y-pt[1].Y; 
  v[1].Z=pt[0].Z-pt[1].Z;

  // Calculate the magnitude of the vectors (limbs)
  float v0_magnitude = sqrt(v[0].X*v[0].X + v[0].Y*v[0].Y + v[0].Z*v[0].Z);
  float v1_magnitude = sqrt(v[1].X*v[1].X + v[1].Y*v[1].Y + v[1].Z*v[1].Z);

  //printf("V0 MAG: %f, V1 MAG: %f\n",v0_magnitude, v1_magnitude);

  // If neither of vectors != 0.0 then find the angle between them
  if(v0_magnitude != 0.0 && v1_magnitude != 0.0){
    v[0].X = v[0].X * (1.0/v0_magnitude); 
    v[0].Y = v[0].Y * (1.0/v0_magnitude);
    v[0].Z = v[0].Z * (1.0/v0_magnitude);
    
    v[1].X = v[1].X * (1.0/v1_magnitude); 
    v[1].Y = v[1].Y * (1.0/v1_magnitude); 
    v[1].Z = v[1].Z * (1.0/v1_magnitude); 
  

    // Find and convert the angle between upper and lower arms
    float theta = acos(v[0].X*v[1].X + v[0].Y*v[1].Y + v[0].Z*v[1].Z);
    float angle_in_degrees = theta * 180 / my_pi;

    return angle_in_degrees;
  }
  else return -997.0; // Error code (though, it's not used yet)
}

// Get and return the angle between JOINT limbs (e.g. upper arm ^ lower arm; upper leg ^ lower leg; hip_to_hip ^ upper_leg etc.)
float getLimbAngle(XnUserID const& user, XnSkeletonJoint const& eJoint1, XnSkeletonJoint const& eJoint2, XnSkeletonJoint const& eJoint3){
  if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
    {
      printf("not tracked!\n");
      return -998.0; // Error code (though, it's not used yet) 
    }
  
  // Get joint positions
  XnSkeletonJointPosition joint1, joint2, joint3;
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, eJoint1, joint1);
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, eJoint2, joint2);
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, eJoint3, joint3);

  // Make sure that all joints are found with enough confidence...
  if (joint1.fConfidence < 0.6 && joint2.fConfidence < 0.6 && joint3.fConfidence < 0.6 )
    {
      return -999.0; // Error code (though, it's not used yet) 
    }
  
  XnPoint3D pt[3]; // Create 3 points
  pt[0] = joint1.position; // E.g. Shoulder
  pt[1] = joint2.position; // E.g. Elbow
  pt[2] = joint3.position; // E.g. Hand
  
  // or another e.g. 
  // pt_0: Hip
  // pt_1: Knee
  // pt_2: Foot

  // The first vector is, Shoulder --> Elbow or Hip --> Knee
  // The second vector is,  Elbow --> Hand or Knee --> Foot 

  XnVector3D v[2]; // Create two vectors

  // S: Shoulder, E: Elbow, H: Hand
  // S  *
  //    |
  // E  *---*
  //        H

  // Calculate the first vector
  // V_{H,E} = V_{E} - V_{H}
  v[0].X=pt[1].X-pt[2].X;
  v[0].Y=pt[1].Y-pt[2].Y;
  v[0].Z=pt[1].Z-pt[2].Z;

  // Calculate the second vector
  // V_{E,S} = V_{S} - V_{E}
  v[1].X=pt[0].X-pt[1].X;
  v[1].Y=pt[0].Y-pt[1].Y; 
  v[1].Z=pt[0].Z-pt[1].Z;

  // Calculate the magnitudes of the vectors
  float v0_magnitude = sqrt(v[0].X*v[0].X + v[0].Y*v[0].Y + v[0].Z*v[0].Z);
  float v1_magnitude = sqrt(v[1].X*v[1].X + v[1].Y*v[1].Y + v[1].Z*v[1].Z);

  //printf("V0 MAG: %f, V1 MAG: %f\n",v0_magnitude, v1_magnitude);

  if(v0_magnitude != 0.0 && v1_magnitude != 0.0){
    v[0].X = v[0].X * (1.0/v0_magnitude); 
    v[0].Y = v[0].Y * (1.0/v0_magnitude);
    v[0].Z = v[0].Z * (1.0/v0_magnitude);
    
    v[1].X = v[1].X * (1.0/v1_magnitude); 
    v[1].Y = v[1].Y * (1.0/v1_magnitude); 
    v[1].Z = v[1].Z * (1.0/v1_magnitude); 
  

    // Find and convert the angle between JOINT limbs
    float theta = acos(v[0].X*v[1].X + v[0].Y*v[1].Y + v[0].Z*v[1].Z);
    float angle_in_degrees = theta * 180 / my_pi;

    return angle_in_degrees;
  }
  else return -997.0; // Error code (though, it's not used yet) 
}

// Gives the angle between:
// the initial AND current position of the user's shoulders, to understand how much he/she rotated his/her shoulders.
float getTorsoRotation(XnUserID const& user, XnSkeletonJoint const& eJoint1, XnSkeletonJoint const& eJoint2){

  if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
    {
      printf("not tracked!\n");
      return -998.0; // Error code (though, it's not used yet)
    }
  
  // Get joint positions
  XnSkeletonJointPosition joint1, joint2;
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, eJoint1, joint1);
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, eJoint2, joint2);

  // Make sure that all joints are found with enough confidence...
  if (joint1.fConfidence < 0.6 && joint2.fConfidence < 0.6)
    {
      return -999.0; // Error code (though, it's not used yet)
    }
  
  XnPoint3D pt[4]; // Create four points
  pt[0] = init_usr_LS; // Vector 0, Point 0 - Initial location of the left shoulder
  pt[1] = init_usr_RS; // Vector 0, Point 1 - Initial location of the right shoulder
  pt[2] = joint1.position; // Vector 1, Point 0 - Current location of the left shoulder
  pt[3] = joint2.position; // Vector 1, Point 1 - Current location of the right shoulder

  XnVector3D v[2]; // Create two vectors

  // Assign the vectors
  v[0].X=pt[2].X-pt[3].X;
  v[0].Y=pt[2].Y-pt[3].Y;
  v[0].Z=pt[2].Z-pt[3].Z;

  v[1].X=pt[0].X-pt[1].X;
  v[1].Y=pt[0].Y-pt[1].Y; 
  v[1].Z=pt[0].Z-pt[1].Z;

  // Calculate the magnitude of the vectors
  float v0_magnitude = sqrt(v[0].X*v[0].X + v[0].Y*v[0].Y + v[0].Z*v[0].Z);
  float v1_magnitude = sqrt(v[1].X*v[1].X + v[1].Y*v[1].Y + v[1].Z*v[1].Z);

  //printf("V0 MAG: %f, V1 MAG: %f\n",v0_magnitude, v1_magnitude);

  // If neither of the vectors are != 0.0 then calculate the angle between them
  if(v0_magnitude != 0.0 && v1_magnitude != 0.0){
    v[0].X = v[0].X * (1.0/v0_magnitude); 
    v[0].Y = v[0].Y * (1.0/v0_magnitude);
    v[0].Z = v[0].Z * (1.0/v0_magnitude);
    
    v[1].X = v[1].X * (1.0/v1_magnitude); 
    v[1].Y = v[1].Y * (1.0/v1_magnitude); 
    v[1].Z = v[1].Z * (1.0/v1_magnitude); 
  

    // Calculate and convert the angle between the initial and current locations of the shoulders
    float theta = acos(v[0].X*v[1].X + v[0].Y*v[1].Y + v[0].Z*v[1].Z); 
    float angle_in_degrees = theta * 180 / my_pi;

    return angle_in_degrees;
  }
  else return -997.0; // Error code (though, it's not used yet)
}

void checkPose(TeleopNaoNi &obj) {
    XnUserID users[2];
    XnUInt16 users_count = 2;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
      XnUserID user = users[i];
      if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
	continue;

      /*  If right and left shoulder - elbow - hand make 90 (+/-5) degrees 
       *  with a certain confidence tell Nao to switch to teleoperation mode
       */
      
      // Angle between Right leg vectors (Right_Hip --> Knee --> Foot)
      float right_limb_angle = getLimbAngle(user, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT);
      //printf("Right limb angle is %f\n", right_limb_angle);


      // Angle between Left leg vectors (Left_Hip --> Knee --> Foot)
      float left_limb_angle = getLimbAngle(user, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT);
      //printf("Left limb angle is %f\n", left_limb_angle);

      // Angle between Left arm vectors (Left_Shoulder --> Elbow --> Hand)
      float left_elbow_angle = getLimbAngle(user, XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);
      
      // Get the current body (torso) location
      XnPoint3D current_bl = getBodyLoc(user, XN_SKEL_TORSO);

      // If the motors are enabled and in BODY CONTROL  mode, then move the robot
      if(obj.m_enabled && obj.m_teleopEnabled){
	if( (left_control_line <=current_bl.X) && (current_bl.X<=left_safe_line )){
	  // Step left
	  obj.m_motion.linear.y = walking_speed;
	}
	else if( (right_control_line>=current_bl.X) && (current_bl.X>=right_safe_line) ){
	  // Step right
	  obj.m_motion.linear.y = -walking_speed;
	}
	else if( (left_safe_line<=current_bl.X) && (current_bl.X<=right_safe_line)){
	  // Stop lateral motion
	  obj.m_motion.linear.y = 0.0;
	}
	
	if( (anterior_control_line<=current_bl.Z) && (current_bl.Z<=anterior_safe_line)){
	  // Step fwd
	  obj.m_motion.linear.x = walking_speed;
	}
	else if( (posterior_safe_line<=current_bl.Z) && (current_bl.Z<=posterior_control_line)){
	  // Step bwd
	  obj.m_motion.linear.x = -walking_speed;
	}
	else if( (anterior_safe_line<=current_bl.Z) && (current_bl.Z<=posterior_safe_line)){
	  // Stop fwd/bwd motion
	  obj.m_motion.linear.x = 0.0;
	}
	  
	
	// Get the current left and right shoulder locations
	XnPoint3D current_ls = getBodyLoc(user, XN_SKEL_LEFT_SHOULDER);
	XnPoint3D current_rs = getBodyLoc(user, XN_SKEL_RIGHT_SHOULDER);

	// if positive rotate right, else rotate left       
	float current_angle = getTorsoRotation(user, XN_SKEL_LEFT_SHOULDER, XN_SKEL_RIGHT_SHOULDER)*my_pi/180.0; 

	// If the user's body is rotated enough to control the robot then rotate the robot
	if( (control_rot>=current_angle) && (current_angle>=safety_rot)){
	  if((current_rs.Z+100.0) < current_ls.Z){
	    obj.m_motion.angular.z = rotating_speed;
	    //printf("Rot Left\n");
	  }
	  else if(current_rs.Z > (current_ls.Z+100.0)){
	    obj.m_motion.angular.z = -1.0*rotating_speed;
	    //printf("Rot Right\n");
	  }
	}
	else if(safety_rot>=current_angle && current_angle>=0.0 ){
	  obj.m_motion.angular.z = 0.0;
	}

	// Get shoulder pitch angles in Radians and publish
	float lsap = getLimbAngle(user, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW)*my_pi/180.0;
	lsap = lsap - 1.6;
        if(lsap>=-2.0 && lsap<=2.0){
	  obj.m_lShoulderAngles.pitch = lsap;
	}
	//printf("Left Shoulder Pitch: %f\n", lsap);

	// Left elbow angle roll
	float lear = getLimbAngle(user, XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND)*my_pi/180.0;
	lear = -1*lear;
	if(lear>=-1.55 && lear<=-0.01){
	  obj.m_lElbowAngles.pitch = lear;
	}

	// Left elbow angle yaw
	float leay = getAngleBetweenLimbs(user, XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND)*my_pi/180.0;
	// Hand down: leay = 0; Nao: 119.5 (~2.0 rad.)
	//            leay = 90; Nao: 0
	//            leay = 180; Nao: -119.5
	leay = 1.57 - leay;
	if(2.0>=leay && leay>=-2.0){
	  obj.m_lElbowAngles.yaw = leay;
	}

	// Right shoulder angles pitch
	float rsap = getLimbAngle(user, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW)*my_pi/180.0;
	rsap = rsap - 1.6;
	if(rsap>=-2.0 && rsap<=2.0){
	  obj.m_rShoulderAngles.pitch = rsap;
	}
	//printf("Right Shoulder Pitch: %f\n", rsap);

	// Right elbow angle roll
	float rear = getLimbAngle(user, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND)*my_pi/180.0;
	if(1.55>=rear && rear>=0.01){
	  obj.m_rElbowAngles.pitch = rear;
	}

	// Right elbow angle yaw
	float reay = getAngleBetweenLimbs(user, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND)*my_pi/180.0;
	reay = reay - 1.57;
	if(2.0>=reay && reay>=-2.0){
	  obj.m_rElbowAngles.yaw = reay;
	}

	// Right shoulder angle roll
	float rsar = getAngleBetweenLimbs(user, XN_SKEL_RIGHT_HIP, XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW)*my_pi/180.0;
	// In front: 90 (1.57) --> -0.5
	// Wide open: 180 (3.14) --> -94.5
	
	rsar = 1.63 - rsar;
	if(-0.01>=rsar && rsar>=-1.63){
	  obj.m_rShoulderAngles.yaw = rsar;
	}

	// Left shoulder angle roll
	float lsar = getAngleBetweenLimbs(user, XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP, XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW)*my_pi/180.0;
	// In front: 90 (1.57) --> 0.5
	// Wide open: 180 (3.14) --> 94.5
	lsar = lsar - 1.57;
	if(1.63>=lsar && lsar>=0.01){
	  obj.m_lShoulderAngles.yaw = lsar;
	}
      }

      // If motors are enabled and GAZE DIRECTION CONTROL mode is true
      if(obj.m_enabled && obj.m_attentionEnabled){

	// Left elbow angle yaw
	float leay = getAngleBetweenLimbs(user, XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);

	// If the user made APPLY COMMAND gesture in GAZE DIRECTION CONTROL mode
	if(left_elbow_angle>=87.0 && left_elbow_angle<=93.0 && leay >= 135.0){

	  // If the command was being already executed then stop
	  if(obj.m_commandEnabled){
	    obj.m_commandEnabled = false;
	    stopWalking(obj); // Stop Walking
	    printf("Command Mode OFF, Back to attention mode\n");
	    naosay.data="mission accomplished"; // let the user know what you're up to
	  }
	  // If the command was not being executed then execute the command
	  else{
	    obj.m_commandEnabled = true;
	    printf("Command Mode ON, Walking towards target\n");
	    naosay.data="as you wish"; // let the user know that you finished executing
	  }
	  ros::Duration(2.0).sleep(); // Sleep for a while, skip switching until noise cancelling.
	} 
      }

      // If the motors are enabled and the command is being executed, then
      if(obj.m_enabled && obj.m_commandEnabled){
	
	// Rotate until your head is looking at 0.0 degrees
	// Then start walking in that direction.
	float counter = floor(obj.m_headAngles.yaw / rot_constant);
	if (counter>0.0){
	  obj.m_motion.angular.z = rotating_speed;
	  obj.m_headAngles.yaw -= rot_constant;
	}
	else if (counter<0.0){
	  obj.m_motion.angular.z = -rotating_speed;
	  obj.m_headAngles.yaw += rot_constant;
	}
	else if (counter == 0.0){
	  obj.m_motion.angular.z = 0.0;
	  obj.m_motion.linear.x = walking_speed;
	} 
      }

      // If the motors are enabled, GAZE DIRECTION MODE is on, and the command is not being executed,
      // then change the Gaze Direction (head angles) of Nao, with respect to the right arm of the user.
      if(obj.m_enabled && obj.m_attentionEnabled && !obj.m_commandEnabled){

	// Head Angle Yaw
	float hay = getAngleBetweenLimbs(user, XN_SKEL_RIGHT_HIP, XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND)*my_pi/180.0;
	hay = hay - 1.57;
	if(2.0>=hay && hay>=-2.0){
	  obj.m_headAngles.yaw = hay;
	  }

	// Head Angle Pitch
	float hap = getAngleBetweenLimbs(user, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND)*my_pi/180.0;
	hap = 0.5 - hap;
	if(0.5>=hay && hay>=-0.66){
	  obj.m_headAngles.pitch = hap;
	  }
	obj.m_headAngles.absolute = 1.0;	
      } 
      

      // If the angle of the right Knee is in the desired interval
      // Then switch between Gaze Direction Mode (attention)  and Body Control Mode (teleoperation)
      if ((right_limb_angle>=87.0) && (right_limb_angle<=93) ){
	if(obj.m_teleopEnabled){
	  obj.m_teleopEnabled = false;
	  obj.m_attentionEnabled = true;
	  naosay.data = "Attention mode";
	  printf("Teleop OFF, Attention ON\n");
	}
	else{
	  obj.m_teleopEnabled = true;
	  obj.m_attentionEnabled = false;
	  naosay.data = "Tele operation mode";
	  printf("Teleop ON, Attention OFF\n");
	}
	ros::Duration(2.0).sleep(); // Sleep for a while, skip switching until noise cancelling.
	
      }


      // If the angle of the left Knee is in the desired interval
      // Then turn the motors ON / OFF
      if((left_limb_angle>=87.0) && (left_limb_angle<=93)){
	if(obj.m_enabled){
	  obj.m_enabled = false;
	  naosay.data = "Motors off";
	  printf("Motors OFF\n");

	  nao_ctrl::MotionCommandBtn motionCmd;
	  motionCmd.button = nao_ctrl::MotionCommandBtn::crouchNoStiff; // Stop walking and remove stiffness
	  obj.m_moveBtnPub.publish(motionCmd);
	}
	else{
	  obj.m_enabled = true;
	  
	  naosay.data = "Motors on";
	  printf("Motors ON\n");
	
	  ros::Duration(0.2).sleep();

	  nao_ctrl::MotionCommandBtn motionCmd;
	  motionCmd.button = nao_ctrl::MotionCommandBtn::stiffnessOn; // Power up the motors
	  obj.m_moveBtnPub.publish(motionCmd);

	  ros::Duration(0.2).sleep();

	  motionCmd.button = nao_ctrl::MotionCommandBtn::initPose; // Stand up and take initial position
	  obj.m_moveBtnPub.publish(motionCmd);
	  
	  init_usr_loc = getBodyLoc(user, XN_SKEL_TORSO); // Initial location of the user right after enabling the motors

	  // Draw safe zone's lines and angle (for rotation of the torso)
	  anterior_safe_line = init_usr_loc.Z - safety_dist;
	  posterior_safe_line = init_usr_loc.Z + safety_dist;
	  
	  right_safe_line = init_usr_loc.X + safety_dist;
	  left_safe_line = init_usr_loc.X - safety_dist;

	  // Draw control zone's lines and angle (for rotation of the torso)
	  anterior_control_line = anterior_safe_line - control_dist;
	  posterior_control_line = posterior_safe_line + control_dist;

	  right_control_line = right_safe_line + control_dist;
	  left_control_line = left_safe_line - control_dist;

	  // Initial location of the user's Right / Left shoulders
	  init_usr_RS = getBodyLoc(user,XN_SKEL_RIGHT_SHOULDER);
	  init_usr_LS = getBodyLoc(user,XN_SKEL_LEFT_SHOULDER);

	}
	ros::Duration(2.0).sleep(); // Sleep for half a second, skip switching until noise cancelling.
      }
     
      if (naosay.data != ""){
	obj.m_speechPub.publish(naosay);
	naosay.data=""; // Reset the speech string of Nao
      }

    }
 
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "teleop_nao_ni");// Initialize ROS

  TeleopNaoNi teleopNao;// Declare and initialize the variables and object(s)
  naosay.data=""; // Initialize the string variable (This string defines what Nao will say)
  string configFilename = ros::package::getPath("nao_openni") + "/openni_tracker.xml";

 
  double publishRate = 5.0;  // rate of publishing motion commands (too high stresses the Nao's CPU)
  teleopNao.privateNh.param("motion_publish_rate", publishRate, publishRate);
  ros::Rate pubRate(publishRate); // Set the publishing rate

  unsigned char state=0; // State variable for initializing (resetting) the control.

  // OpenNI functions begin -------------------------------------------------
  // Refer to OpenNI Manual and Discussion Group to get help about these functions and variables
  XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
  CHECK_RC(nRetVal, "InitFromXml");
  
  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
  CHECK_RC(nRetVal, "Find depth generator");
  
  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
  if (nRetVal != XN_STATUS_OK) {
    nRetVal = g_UserGenerator.Create(g_Context);
    CHECK_RC(nRetVal, "Find user generator");
  }
  
  if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
    printf("Supplied user generator doesn't support skeleton\n");
    return 1;
  }
  
  XnCallbackHandle hUserCallbacks;
  g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
  
  XnCallbackHandle hCalibrationCallbacks;
  g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);
  
  if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
    g_bNeedPose = TRUE;
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
      printf("Pose required, but not supported\n");
      return 1;
    }
    
    XnCallbackHandle hPoseCallbacks;
    g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
    
    g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
  }
  
  g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
  
  nRetVal = g_Context.StartGeneratingAll();
  CHECK_RC(nRetVal, "StartGenerating");
  // OpenNI functions end-------------------------------------------------

  // Endless loop to publish messages
  while(teleopNao.nh.ok()){

    // We don't need to use ros::spin() or ros::spinOnce() because we don't subscribe to any topic.

    // For the very first cycle test if everything works ok, and reset the control.
    // 0. Enable the remote control
    // 1. Then in the second pass remove the stiffness
    // 2. Then finally disable the remote control again until teleoperation.

    if (state == 0){

      teleopNao.m_enabled = true;
      naosay.data = "Control enabled";
      nao_ctrl::MotionCommandBtn motionCmd;
      motionCmd.button = nao_ctrl::MotionCommandBtn::stiffnessOn;
      teleopNao.m_moveBtnPub.publish(motionCmd);
      state++;
    }
    else if (state == 1){
      
      naosay.data = "Stiffness removed";
      nao_ctrl::MotionCommandBtn motionCmd;
      motionCmd.button = nao_ctrl::MotionCommandBtn::stiffnessOff;
      teleopNao.m_moveBtnPub.publish(motionCmd);
      state++;

    }
    else if (state == 2){
      
      teleopNao.m_enabled = false;
      naosay.data = "Control disabled";
      state++;
    }

    if (naosay.data != ""){
      teleopNao.m_speechPub.publish(naosay);
      naosay.data=""; // Reset the string to nothing.
    }  

    g_Context.WaitAndUpdateAll(); // Update skeleton recognition (OpenNI's callbacks)
    
    checkPose(teleopNao); // Check user's pose and assign according values to the messages that will be published

    teleopNao.pubMsg(); // Publish messages to Nao

    pubRate.sleep(); // Sleep to get to the rate of publishing
  }

  g_Context.Shutdown(); // Close OpenNI
  return 0;

}
