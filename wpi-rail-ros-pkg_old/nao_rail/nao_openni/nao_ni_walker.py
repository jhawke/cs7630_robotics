#!/usr/bin/python
#
# This code is a modified version of Armin Hornung, University of Freiburg 
# ROS node to send out movement commands to the Nao Aldebaran API (teleoperated)
#
# Copyright 2010 Halit Bener SUAY, Worcester Polytechnic Institute
# benersuay@wpi.edu
#
# https://github.com/wpi-ros-pkg-git/nao_openni
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the <ORGANIZATION> nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# This code has arm angle control in addition to its original

import roslib
roslib.load_manifest('nao_ctrl')
import rospy

import math
from math import fabs

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from nao_ctrl.msg import MotionCommandBtn
from nao_ctrl.msg import HeadAngles

try:
	import naoqi
	import motion
	from naoqi import ALProxy
except ImportError:
	rospy.logerr("Error importing NaoQI. Please make sure that Aldebaran's NaoQI API is in your PYTHONPATH.")
	exit(1)

from crouch import crouch
from init_pose import initPose

class NaoWalker():
	def __init__(self, ip, port):
		
		# ROS initialization:
		rospy.init_node('nao_walker')
		  
		self.connectNaoQi(ip, port)

		# walking pattern params:
		self.stepFrequency = rospy.get_param('~step_frequency', 0.5)  
        
        # other params
		self.maxHeadSpeed = rospy.get_param('~max_head_speed', 0.2)
		self.headYawChange = rospy.get_param('~head_yaw_change',0.5)
		self.HeadPitchChange = rospy.get_param('~head_pitch_change', 0.5)
		
		self.motionProxy.stiffnessInterpolation('Body', 1.0, 0.5)
		
		# last: ROS subscriptions (after all vars are initialized)
		rospy.Subscriber("head_angles", HeadAngles, self.handleHead, queue_size=1)
		rospy.Subscriber("cmd_vel", Twist, self.handleCmdVel, queue_size=1)
		rospy.Subscriber("motion_command_btn", MotionCommandBtn, self.handleMotionBtn, queue_size=1)
		rospy.Subscriber("speech", String, self.handleSpeech)

		rospy.Subscriber("lshoulder_angles", HeadAngles, self.handleLArm, queue_size=1)
		rospy.Subscriber("lelbow_angles", HeadAngles, self.handleLElbow, queue_size=1)
		rospy.Subscriber("rshoulder_angles", HeadAngles, self.handleRArm, queue_size=1)
		rospy.Subscriber("relbow_angles", HeadAngles, self.handleRElbow, queue_size=1)
		
		rospy.loginfo("nao_walker initialized")

	# (re-) connect to NaoQI:
	def connectNaoQi(self, ip, port):
		rospy.loginfo("Connecting to NaoQi at %s:%d", ip, port)
		
		self.ttsProxy = None
		self.motionProxy = None
		
		try:
			self.ttsProxy = ALProxy("ALTextToSpeech",ip,port)
		except RuntimeError,e:
			rospy.logwarn("No Proxy to TTS available, disabling speech output.")

		try:
			self.motionProxy = ALProxy("ALMotion",ip,port)
			self.motionProxy.setWalkArmsEnable(False,False)
		except RuntimeError,e:
			rospy.logerr("Error creating Proxy to motion, exiting...")
			print e
			exit(1)

	def stopWalk(self):
		""" Stops the current walking bahavior and blocks until the clearing is complete. """
		try:
			self.motionProxy.setWalkTargetVelocity(0.0, 0.0, 0.0, self.stepFrequency)
			self.motionProxy.waitUntilWalkIsFinished()

						
		except RuntimeError,e:
			print "An error has been caught"
			print e
			
	
	def handleSpeech(self,data):
		if (not self.ttsProxy is None):
			try:
				self.ttsProxy.say(data.data)
			except RuntimeError,e:
				rospy.logerr("Exception caught:\n%s", e)
	
	
	# Head needs to be rewritten:
	def handleHead(self,data):
		"""handles head movement callback """
		
		#headChange = [0.5*data.yaw,0.5*data.pitch]  
		
		try:
			#self.motionProxy.changeAngles(["HeadYaw", "HeadPitch"], headChange, self.maxHeadSpeed)
			self.motionProxy.setAngles(["HeadYaw", "HeadPitch"], [data.yaw, data.pitch], self.maxHeadSpeed)
		except RuntimeError,e:
			rospy.logerr("Exception caught:\n%s", e)
	
	def handleCmdVel(self, data):
		rospy.logdebug("Walk cmd_vel: %d %d %d, frequency %d", data.linear.x, data.linear.y, data.angular.z, self.stepFrequency)
		try:
			self.motionProxy.setWalkTargetVelocity(data.linear.x, data.linear.y, data.angular.z, self.stepFrequency)
		except RuntimeError,e:
			rospy.logerr("Exception caught:\n%s", e)
		
	
	def handleMotionBtn(self,data):
#		global walkLock
#		walkLock.acquire()
			
		if (data.button == MotionCommandBtn.crouchNoStiff):
			self.stopWalk()
			crouch(self.motionProxy)
			self.motionProxy.stiffnessInterpolation('Body',0.0, 0.5)
			if (not self.ttsProxy is None):
				self.ttsProxy.say("Stiffness removed")
		elif (data.button == MotionCommandBtn.initPose):
			initPose(self.motionProxy)
		elif (data.button == MotionCommandBtn.stiffnessOn):
			self.motionProxy.stiffnessInterpolation('Body',1.0, 0.5)
		elif (data.button == MotionCommandBtn.stiffnessOff):
			self.motionProxy.stiffnessInterpolation('Body',0.0, 0.5)

	def handleLArm(self,data):
		try:
			self.motionProxy.setAngles(["LShoulderPitch", "LShoulderRoll"], [data.pitch, data.yaw], self.maxHeadSpeed)
		except RuntimeError,e:
			rospy.logerr("Exception caught:\n%s", e)

	def handleRArm(self,data):
		try:
			self.motionProxy.setAngles(["RShoulderPitch", "RShoulderRoll"], [data.pitch, data.yaw], self.maxHeadSpeed)
		except RuntimeError,e:
			rospy.logerr("Exception caught:\n%s", e)
	
	def handleLElbow(self,data):
		try:
			self.motionProxy.setAngles(["LElbowRoll","LElbowYaw" ], [data.pitch, data.yaw], self.maxHeadSpeed)
		except RuntimeError,e:
			rospy.logerr("Exception caught:\n%s", e)

	def handleRElbow(self,data):
		try:
			self.motionProxy.setAngles(["RElbowRoll","RElbowYaw"], [data.pitch, data.yaw], self.maxHeadSpeed)
		except RuntimeError,e:
			rospy.logerr("Exception caught:\n%s", e)		


if __name__ == '__main__':
	
	# get connection from command line:
	from optparse import OptionParser

	parser = OptionParser()
	parser.add_option("--pip", dest="pip", default="127.0.0.1",
	                  help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
	parser.add_option("--pport", dest="pport", default=9559,
	                  help="port of parent broker. Default is 9559.", metavar="PORT")
		
	(options, args) = parser.parse_args()

	walker = NaoWalker(options.pip, int(options.pport))
	rospy.spin()
	rospy.loginfo("nao_walker NaoWalker...")
	walker.stopWalk()
	
	rospy.loginfo("nao_walker stopped.")
	exit(0)
