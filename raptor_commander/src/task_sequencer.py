#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: talker.py 5263 2009-07-17 23:30:38Z sfkwc $

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import roslib; roslib.load_manifest('raptor_commander')
import rospy
import raptor_commander.srv
import std_msgs

import deliberator

class Enum(set):
    def __getattr__(self, name):
        if name in self:
            return name
        raise AttributeError



class TaskSequencer():
   
    # Main update rate
    publishRate = 1 # Hz
    
    # FSM States
    RobotStates = Enum(["INIT", \
                        "FIND_DARK", \
                        "REL_MOVE", \
                        "ABS_MOVE", \
                        "ALIGN_BRIGHT", \
                        "WAIT_ALIGN", \
                        "SCAN_FOR_PREY", \
                        "STALK_PREY", \
                        "STALK_LOST_PREY", \
                        "PLAY_DEAD", \
                        "PLAY_DEAD_LOST_PREY", \
                        "FLEE", \
                        "KILL"])
    currentState = RobotStates.INIT
    
    # Parameters used by the functions to determine state transition conditions. 
    # Pulled from subscriptions to other ros nodes 
    preyFront = -1
    preyBack = 1
    preyGone = 0
    
    # TIMERS
    currentTime = 0
    
    scanForPreyStartTime = 0
    maxScanForPreyTime = 10
    
    stalkLostPreyStartTime = 0
    maxStalkLostPreyTime = 10
    
    playDeadLostPreyStartTime = 0
    maxPlayDeadLostPreyTime = 10
    
    fleeStartTime = 0
    maxFleeTime = 10
    
    fdStartTime = 0
    maxFDTime = 10
    
    
    darkness = 0
    darknessThreshold = 1000
    
    blobX = None
    blobY = None
    blobColour = preyBack    
    
    # NB: ALL GAINS BETWEEN -10 - +10
    behaviourGains = {"FIND_DARK": 0.0, "FIND_BLOB":0.0, "ABS_MOVE":0.0, "REL_MOVE":0.0}
    
    # SET BEHAVIOUR GAINS
    def zeroBehaviourGains(self):
        self.behaviourGains["FIND_DARK"] = 0.0
        self.behaviourGains["FIND_BLOB"] = 0.0
        self.behaviourGains["ABS_MOVE"] = 0.0
        self.behaviourGains["REL_MOVE"] = 0.0
    
    def setBehaviourGains(self, fd, fb, am, rm):
        self.behaviourGains["FIND_DARK"] = fd
        self.behaviourGains["FIND_BLOB"] = fb
        self.behaviourGains["ABS_MOVE"] = am
        self.behaviourGains["REL_MOVE"] = rm
    
    # Functions to set the appropriate gains for that state
    def detected(self):
        self.setBehaviourGains(0.0, 0.0, 0.0, 0.0)
        rospy.logdebug("ts.detected")
    
    def flee(self):
        self.setBehaviourGains(1.0, 0.0, 0.0, 0.0)
        rospy.logdebug("ts.flee")
    
    def stalkPrey(self):
        rospy.logdebug("ts.stalkPrey")
        self.setBehaviourGains(0.0, 1.0, 0.0, 0.0)
        
    def scanForPrey(self):
        rospy.logdebug("ts.scanForPrey")
        self.setBehaviourGains(0.0, 0.0, 0.0, 0.0)
        
    def findDark(self):
        rospy.logdebug("ts.findDark")
        self.setBehaviourGains(1.0, 0.0, 0.0, 0.0)
    
    def wait(self):
        rospy.logdebug("ts.wait")
        self.setBehaviourGains(0.0, 0.0, 0.0, 0.0)
    
    
    # CONDITIONAL FUNCTIONS - check if certain transition conditions are met..
    def checkIfDarkFound(self):
        rospy.logdebug("ts.checkIfDarkFound")
        if self.darkness > self.darknessThreshold:
            return True
        else:
            return False
    
    def checkIfDetected(self):
        rospy.logdebug("ts.checkIfDetected")
        if self.blobColour == self.preyFront:
            return True
        else:
            return False
    
    def threatGone(self):
        rospy.logdebug("ts.threatGone")
        if self.blobColour == self.preyGone:
            return True
        else:
            return False
        
    def targetLost(self):
        rospy.logdebug("ts.targetLost")
        if self.blobColour == self.preyGone:
            return True
        else:
            return False
            
    def scanTimeout(self):
        rospy.logdebug("ts.scanTimeout curr %s start %s thres %s" % (self.currentTime, self.timeLookingForPrey, self.scanForPreyTimeout))
        if (self.currentTime - self.timeLookingForPrey) > self.scanForPreyTimeout:
            return True
        else:
            return False
    
    def waitTimeout(self):
        if (self.currentTime - self.waitStartTime) > self.maxWaitTime:
            return True
        else:
            return False
        
    def blobGreen(self): # Prey looking away... can stalk
        if self.blobColour == self.preyBack:
            return True
        else:
            return False
    
    def blobRed(self): # Prey looking at robot... detected
        if self.blobColour == self.preyFront:
            return True
        else:
            return False
    
    # FUNCTIONS FOR EACH STATE RESPONSE (INCLUDING TRANSITION CONDITIONS)
    def findDarkResponse(self):
        if self.checkIfDarkFound():
            self.currentState = self.RobotStates.SCAN_FOR_PREY
            self.timeLookingForPrey = self.currentTime
            return
        
        if (self.currentTime - self.fdStartTime) > self.maxFDTime:
            # ASK FOR ADVICE FROM DELIBERATOR.
            self.adviceFDClient()
        
        self.findDark();
                
    def scanForPreyResponse(self):
        if self.blobGreen() or self.blobRed():
            self.currentState = self.RobotStates.STALK_PREY
        elif self.scanTimeout():
            self.currentState = self.RobotStates.FORCE_MOVE
        else:
            self.scanForPrey()
            
    def forceMoveResponse(self):
        if self.forceMoveTimeout():
            self.currentState = self.RobotStates.FIND_DARK
        else:
            self.setBehaviourGains(0.0, 0.0, 0.0, 1.0)
            # TODO: Set relMove parameters as well
    
    def stalkPreyResponse(self):
        if self.targetLost():
            self.currentState = self.RobotStates.WAIT
            self.waitStartTime = self.currentTime
        elif self.checkIfDetected():
            self.currentState = self.RobotStates.DETECTED
        elif self.checkIfCanKillPrey():
            self.currentState = self.RobotStates.KILL
        else:
            self.stalkPrey()
    
    def waitResponse(self):
        if self.blobGreen(): # Passive prey
            self.currentState = self.RobotStates.STALK
        elif self.blobRed(): # Prey looking at the robot
            self.currentState = self.RobotStates.DETECTED            
        elif self.waitTimeout():
            self.currentState = self.RobotStates.FIND_DARK
        else:
            self.wait()
            
    def detectedResponse(self):
        # TODO: Deliberator world model here.
        if self.blobGreen(): # AND ADVICE FROM DELIVERATOR
            self.currentState = self.RobotStates.STALK
        elif self.blobRed(): 
            # TODO: AND ADVICE FROM DELIBERATOR
            self.currentState = self.RobotStates.FLEE
        else: # IF PREY DOESN'T EXIST ANY MORE
            self.currentState = self.RobotStates.FIND_DARK
    
    def fleeResponse(self):
        if self.blobRed(): # If we still see red for some reason.
            self.fleeStartTime = self.currentTime
            
        if (self.currentTime - self.fleeStartTime) > self.maxFleeTime:
            self.currentState = self.RobotStates.FIND_DARK
        else:
            self.flee()
    
    
    def checkForStateChange(self):
        if self.currentState == self.RobotStates.INIT:
            self.currentState = self.RobotStates.FIND_DARK
            self.zeroBehaviourGains()
        
        elif self.currentState == self.RobotStates.FIND_DARK:
            self.findDarkResponse()
              
        elif self.currentState == self.RobotStates.SCAN_FOR_PREY:
            self.scanForPreyResponse()
                
        elif self.currentState == self.RobotStates.FORCE_MOVE:
            self.forceMoveResponse()
            
        elif self.currentState == self.RobotStates.STALK_PREY:
            self.stalkPreyResponse()
        
        elif self.currentState == self.RobotStates.WAIT:
            self.waitResponse()
                
        elif self.currentState == self.RobotStates.DETECTED:
            self.detectedResponse()
        
        elif self.currentState == self.RobotStates.FLEE:
            self.fleeResponse()
            
        else:
            rospy.logdebug("ts broken state")
    
    # SUBSCRIBED DATA - PUSH DATA INTO INTERNAL VARIABLES
    def darknessCallback(self, data):
        rospy.loginfo(rospy.get_caller_id()+" tsDarkness heard %f",data.data)
        self.darkness = data.data
        
    def darknessHistoCallback(self, data): # TODO: REMOVE THIS - only used by deliberator?
        rospy.loginfo(rospy.get_caller_id()+" tsDarknessHisto heard %f",data.data)
        
    def preyCallback(self, data):
        #rospy.loginfo(rospy.get_caller_id()+"I heard %f",data.data)
        print 'preyCallback'
        x = data.data[0]
        y = data.data[1]
        colour = data.data[2]
        print "x %f, y %f, colour %f" % (x,y,colour)
    
    
    # ADVICE REQUESTS
    def adviceFDClient(self):
        rospy.wait_for_service('adviceFD')
        try:
            adviceFDProxy = rospy.ServiceProxy('adviceFD', raptor_commander.srv.getAdviceFD)
            response = adviceFDProxy(0)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def adviceFleeClient(self):
        rospy.wait_for_service('adviceFLEE')
        try:
            adviceFLEEProxy = rospy.ServiceProxy('adviceFLEE', raptor_commander.srv.getAdviceFLEE)
            response = adviceFLEEProxy()
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def adviceDETClient(self):
        rospy.wait_for_service('adviceDET')
        try:
            adviceDETProxy = rospy.ServiceProxy('adviceDET', raptor_commander.srv.getAdviceDET)
            response = adviceDETProxy()
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    # MAIN FUNCTION
    def execute(self):
        pub = rospy.Publisher('taskSequencer', std_msgs.msg.String)
        
        stalkPub = rospy.Publisher('STALK_GAIN', std_msgs.msg.Float32)
        findDarkPub = rospy.Publisher('FIND_DARK_GAIN', std_msgs.msg.Float32)
        scanForPreyPub = rospy.Publisher('SCAN_FOR_PREY_GAIN', std_msgs.msg.Float32)
        playDeadPub = rospy.Publisher('PLAY_DEAD_GAIN', std_msgs.msg.Float32)
        
        # Darkness parameter used by the task sequencer
        rospy.Subscriber("DARKNESS_PARAM", std_msgs.msg.Float32, self.darknessCallback)
        # Darkness dataset used by the deliberator
        rospy.Subscriber("DARKNESS_HISTOGRAM", std_msgs.msg.Float32MultiArray, self.darknessHistoCallback)
        # Prey data used by the task sequencer
        rospy.Subscriber("PREY_PARAM", std_msgs.msg.Float32MultiArray, self.preyCallback)
        
        
        
        rospy.init_node('TaskSequencer')
        r = rospy.Rate(self.publishRate)
        while not rospy.is_shutdown():
            self.currentTime = rospy.get_time() 
            str = "TS %s"%self.currentTime
            
            # RUN THE SEQUENCER
            self.checkForStateChange()
            
            # Publish behavioural primitive gains
            rospy.loginfo(str) # Timestamp string
            stalkPub.publish(self.behaviourGains['FIND_DARK'])
            findDarkPub.publish(self.behaviourGains['FIND_BLOB'])
            scanForPreyPub.publish(self.behaviourGains['ABS_MOVE'])
            playDeadPub.publish(self.behaviourGains['REL_MOVE'])
            
            pub.publish(str)
            r.sleep()
        
if __name__ == '__main__':
    try:
        t = TaskSequencer()
        t.execute()
    except rospy.ROSInterruptException: pass
