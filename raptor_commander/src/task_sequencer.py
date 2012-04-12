#!/usr/bin/env python

'''
Created on 9/04/2012

@author: jdh
'''

import roslib; roslib.load_manifest('raptor_commander')
import rospy
import raptor_commander.srv
import raptor_commander.msg
import std_msgs

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
    
    waitForAlignStartTime = 0
    maxWaitForAlignTime = 10
    
    relMoveStartTime = 0
    maxRelMoveTime = 10
    absMoveStartTime = 0
    maxAbsMoveTime = 10
    
    findDarkStartTime = 0
    maxFindDarkTime = 10
    
    killSoundStartTime = 0
    maxKillSoundTime = 10
    
    # THRESHOLDS
    darkness = 0
    darknessThreshold = 1000
    
    # PERCENTAGES for advice acceptance
    fdAdviceAcceptThreshold = 0.9 # Above this, accept the advice outright
    fdAdviceRejectThreshold = 0.5 # Below this, reject the advice outright. Between these, use the midpoint
    
    blobX = None
    blobY = None
    blobColour = preyBack    
    
    # NB: ALL GAINS BETWEEN -10 - +10
    behaviourGains = {"FIND_DARK": 0.0, "FIND_BLOB":0.0, "ABS_MOVE":0.0, "REL_MOVE":0.0}
    
    relMoveParams = {"theta_rad":0.0, "val":0.0, "time_sec":0.0}
    absMoveParams= {"x":0, "y":0, "val":0.0, "time_sec":0.0}
    
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
    
    def relMove(self):
        msg = raptor_commander.msg.rel_pos_req()
        msg.theta_rad = self.relMoveParams["theta_rad"]
        msg.val = self.relMoveParams["val"]
        msg.time_sec = self.relMoveParams["time_sec"]
        self.relMoveParamsPub.publish(msg)
    
    def absMove(self):
        msg = raptor_commander.msg.abs_pos_req()
        msg.x = self.absMoveParams["x"]
        msg.y = self.absMoveParams["y"]
        msg.val = self.absMoveParams["val"]
        msg.time_sec = self.absMoveParams["time_sec"]
        self.absMoveParamsPub.publish(msg)
        
    def alignBright(self):
        print "alignBright"
    
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
        self.setBehaviourGains(1.0, 0.0, 0.0, 0.0)
    
    def wait(self):
        rospy.logdebug("ts.wait")
        self.setBehaviourGains(0.0, 0.0, 0.0, 0.0)
    
    def playKillSound(self):
        print "playKillSound"
    
    
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
    
    def isBrightAligned(self):
        print "isBrightAligned"
    
    
    # FUNCTIONS FOR EACH STATE RESPONSE (INCLUDING TRANSITION CONDITIONS)
    def findDarkResponse(self):
        # 1) Check if we have a good enough spot here
        if self.checkIfDarkFound():
            self.setState(self.RobotStates.ALIGN_BRIGHT)
            return
        # 2) If not, check with the timeout. If it's passed, ask the deliberator for advice.
        if (self.currentTime - self.findDarkStartTime) > self.maxFindDarkTime:
            fdAdvice = self.adviceFDClient() # TODO - BROKE!!!
            
            if fdAdvice.newThreshold > self.fdAdviceAcceptThreshold*self.darknessThreshold:
                rospy.loginfo("TS: FD failed... Accepting new Threshold. Absmove to given position")
                self.darknessThreshold = fdAdvice.newThreshold
                self.setState(self.RobotStates.ABS_MOVE)
            elif fdAdvice.newThreshold < self.fdAdviceRejectThreshold*self.darknessThreshold:
                rospy.loginfo("TS: FD failed... Rejecting new Theshold. Relmove randomly")
                self.setState(self.RobotStates.REL_MOVE)
            else: # Advice isn't great or horrible... update threshold to a midpoint
                rospy.loginfo("TS: FD failed... Lowering threshold. Relmove randomly")
                self.darknessThreshold = (self.darknessThreshold + fdAdvice.newThreshold)/2
                self.setState(self.RobotStates.REL_MOVE)
                
            # TODO - FINISH THIS
            # Deliberator provides a suggested new darkness threshold, and the best seen darkness location + value
            # If threshold is >50% lower than current... reject it outright. 
            # If it's between 50% and 10%, take the midpoint
            # If it's within 10% of the current threshold, just accept it and update the stored threshold. =
            
            return
        self.findDark()
    
    def relMoveResponse(self):
        if (self.currentTime - self.relMoveStartTime) > self.maxRelMoveTime:
            self.setState(self.RobotStates.FIND_DARK)
            return
        
        self.relMove()
        
    def absMoveResponse(self):
        self.absMove() # abs move terminates immediately
        self.setState(self.RobotStates.FIND_DARK)
    
    def alignBrightResponse(self):
        self.alignBright() # State only does one action then transitions
        self.setState(self.RobotStates.WAIT_ALIGN)
    
    def waitAlignResponse(self):
        if self.isBrightAligned():
            self.setState(self.RobotStates.SCAN_FOR_PREY)
                    
        elif (self.currentTime - self.waitForAlignStartTime) > self.maxWaitForAlignTime:
            self.setState(self.RobotStates.FIND_DARK)
                
    def scanForPreyResponse(self):
        if self.blobGreen() or self.blobRed(): # If any prey object is seen
            self.setState(self.RobotStates.STALK_PREY)
        elif (self.currentTime - self.scanForPreyStartTime) > self.maxScanForPreyTime: # timeout
            self.setState(self.RobotStates.FIND_DARK)
        else:
            self.scanForPrey()
    
    def stalkPreyResponse(self):
        if self.preyLost(): # Failure condition
            self.setState(self.RobotStates.STALK_LOST_PREY)
        elif self.checkIfDetected(): # Failure condition
            self.setState(self.RobotStates.PLAY_DEAD)
        elif self.checkIfCanKillPrey(): # Success!
            self.setState(self.RobotStates.KILL)
        else:
            self.stalkPrey()
    
    def stalkLostPreyResponse(self):
        if self.blobGreen() or self.blobRed(): # Reacquired prey
            self.setState(self.RobotStates.STALK)         
        elif (self.currentTime - self.stalkLostPreyStartTime) > self.maxStalkLostPreyTime:
            self.setState(self.RobotStates.FIND_DARK)
        else:
            self.wait()
            
    def playDeadResponse(self):
        # TODO: Deliberator world model here.
        if self.blobGreen(): # Prey looks away - immediately stalk
            self.setState(self.RobotStates.STALK)
        elif self.preyLost(): # IF PREY DOESN'T EXIST ANY MORE. Go into wait state
            self.setState(self.PLAY_DEAD_LOST_PREY)
        else: # A red blob must still be visible
            # If object centroid is growing in area, or the centroid is moving downwards in the FOV (ie towards robot) 
            if True: # TODO - MAKE THIS THE FLEE CONDITION
                self.setState(self.RobotStates.FLEE)
            
    def playDeadLostPreyResponse(self):
        if (self.currentTime - self.playDeadLostPreyStartTime) > self.maxPlayDeadLostPreyTime:
            self.setState(self.RobotStates.FIND_DARK)
        else:
            self.wait()
    
    def fleeResponse(self):
        if self.blobGreen(): # If we see Green - stalk again
            self.setState(self.RobotStates.STALK_PREY)        
        elif (self.currentTime - self.fleeStartTime) > self.maxFleeTime:
            self.setState(self.RobotStates.FIND_DARK)
        else:
            self.flee()
    
    def killResponse(self):
        if (self.currentTime - self.killSoundStartTime) > self.maxKillSoundTime:
            self.setState(self.RobotStates.FIND_DARK)
            return
        self.playKillSound()
    
    def setState(self, state):
        if self.currentState == self.RobotStates.INIT:
            rospy.logdebug("ts.INIT")
            self.currentState = self.RobotStates.INIT
        
        elif state == self.RobotStates.FIND_DARK:
            rospy.logdebug("ts.FIND_DARK")
            self.findDarkStartTime = self.currentTime
            self.currentState = self.RobotStates.FIND_DARK
        
        elif state == self.RobotStates.REL_MOVE:
            rospy.logdebug("ts.REL_MOVE")
            self.currentState = self.RobotStates.REL_MOVE
            self.relMoveStartTime = self.currentTime
        
        elif state == self.RobotStates.ABS_MOVE:
            rospy.logdebug("ts.ABS_MOVE")
            self.currentState = self.RobotStates.ABS_MOVE
            self.absMoveStartTime = self.currentTime
        
        elif state == self.RobotStates.ALIGN_BRIGHT:
            rospy.logdebug("ts.ALIGN_BRIGHT")
            self.currentState = self.RobotStates.ALIGN_BRIGHT
        
        elif state == self.RobotStates.WAIT_ALIGN:
            rospy.logdebug("ts.WAIT_ALIGN")
            self.waitForAlignStartTime = self.currentTime
            self.currentState = self.RobotStates.WAIT_ALIGN
              
        elif state == self.RobotStates.SCAN_FOR_PREY:
            rospy.logdebug("ts.SCAN_FOR_PREY")
            self.scanForPreyStartTime = self.currentTime
            self.currentState = self.RobotStates.SCAN_FOR_PREY
            
        elif state == self.RobotStates.STALK_PREY:
            rospy.logdebug("ts.STALK_PREY")
            self.currentState = self.RobotStates.STALK_PREY
        
        elif state == self.RobotStates.STALK_LOST_PREY:
            rospy.logdebug("ts.STALK_LOST_PREY")
            self.currentState = self.RobotStates.STALK_LOST_PREY
            self.stalkLostPreyStartTime = self.currentTime
                
        elif state == self.RobotStates.PLAY_DEAD:
            rospy.logdebug("ts.PLAY_DEAD")
            self.currentState = self.RobotStates.PLAY_DEAD
        
        elif state == self.RobotStates.PLAY_DEAD_LOST_PREY:
            rospy.logdebug("ts.PLAY_DEAD_LOST_PREY")
            self.currentState = self.RobotStates.PLAY_DEAD_LOST_PREY
            self.playDeadLostPreyStartTime = self.currentTime
        
        elif state == self.RobotStates.FLEE:
            rospy.logdebug("ts.FLEE")
            self.currentState = self.RobotStates.FLEE
            self.fleeStartTime = self.currentTime
        
        elif state == self.RobotStates.KILL:
            rospy.logdebug("ts.KILL")
            self.currentState = self.RobotStates.KILL
            self.killSoundStartTime = self.currentTime
            
        else:
            rospy.logdebug("ts broken state") 
    
    # MAIN STATE ENGINE
    def checkForStateChange(self):
        if self.currentState == self.RobotStates.INIT:
            rospy.logdebug("ts.INIT")
            self.currentState = self.RobotStates.FIND_DARK
            self.findDarkStartTime = self.currentTime
            rospy.logdebug("ts.FIND_DARK")
            self.zeroBehaviourGains()
        
        elif self.currentState == self.RobotStates.FIND_DARK:
            self.findDarkResponse()
        
        elif self.currentState == self.RobotStates.REL_MOVE:
            self.relMoveResponse()
        
        elif self.currentState == self.RobotStates.ABS_MOVE:
            self.absMoveResponse()
        
        elif self.currentState == self.RobotStates.ALIGN_BRIGHT:
            self.alignBrightResponse()
        
        elif self.currentState == self.RobotStates.WAIT_ALIGN:
            self.waitAlignResponse()
              
        elif self.currentState == self.RobotStates.SCAN_FOR_PREY:
            self.scanForPreyResponse()
            
        elif self.currentState == self.RobotStates.STALK_PREY:
            self.stalkPreyResponse()
        
        elif self.currentState == self.RobotStates.STALK_LOST_PREY:
            self.stalkLostPreyResponse()
                
        elif self.currentState == self.RobotStates.PLAY_DEAD:
            self.playDeadResponse()
        
        elif self.currentState == self.RobotStates.PLAY_DEAD_LOST_PREY:
            self.playDeadLostPreyResponse()
        
        elif self.currentState == self.RobotStates.FLEE:
            self.fleeResponse()
        
        elif self.currentState == self.RobotStates.KILL:
            self.killResponse()
            
        else:
            rospy.logdebug("ts broken state")
    
    # SUBSCRIBED DATA - PUSH DATA INTO INTERNAL VARIABLES
    def darknessCallback(self, data):
        rospy.loginfo(rospy.get_caller_id()+" tsDarkness heard %f",data.data)
        self.darkness = data.data
        
    def preyCallback(self, data):
        #rospy.loginfo(rospy.get_caller_id()+"I heard %f",data.data)
        print 'preyCallback'
        x = data.data.x
        y = data.data.y
        colour = data.data.colourID
        print "x %s, y %s, colour %s" % (x,y,colour)
    
    
    # ADVICE REQUESTS FROM DELIBERATOR
    def adviceFDClient(self):
        rospy.wait_for_service('adviceFD')
        try:
            adviceFDProxy = rospy.ServiceProxy('adviceFD', raptor_commander.srv.getAdviceFD)
            response = adviceFDProxy()
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def advicePDClient(self):
        rospy.wait_for_service('advicePD')
        try:
            advicePDProxy = rospy.ServiceProxy('advicePD', raptor_commander.srv.getAdvicePD)
            response = advicePDProxy()
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    # MAIN FUNCTION
    def execute(self):
        pub = rospy.Publisher('taskSequencer', std_msgs.msg.String)
        
        # behaviour gain parameters
        self.stalkPub = rospy.Publisher('STALK_GAIN', std_msgs.msg.Float32)
        self.findDarkPub = rospy.Publisher('FIND_DARK_GAIN', std_msgs.msg.Float32)
        self.relMovePub = rospy.Publisher('REL_MOVE_GAIN', std_msgs.msg.Float32)
        self.absMovePub = rospy.Publisher('ABS_MOVE_GAIN', std_msgs.msg.Float32)
        # move parameter sets.
        self.relMoveParamsPub = rospy.Publisher('REL_MOVE_PARAMS', raptor_commander.msg.rel_pos_req)
        self.absMoveParamsPub = rospy.Publisher('ABS_MOVE_PARAMS', raptor_commander.msg.abs_pos_req)
        
        # Darkness parameter used by the task sequencer
        rospy.Subscriber("DARKNESS_COEFF", std_msgs.msg.Float32, self.darknessCallback)
        # Prey data used by the task sequencer - colour of blobs in vision.
        rospy.Subscriber("BLOB_COLOUR", raptor_commander.msg.blob_colour, self.preyCallback)
        
        
        
        rospy.init_node('TaskSequencer')
        r = rospy.Rate(self.publishRate)
        while not rospy.is_shutdown():
            self.currentTime = rospy.get_time() 
            str = "TS %s"%self.currentTime
            
            # RUN THE SEQUENCER
            self.checkForStateChange()
            
            # Publish behavioural primitive gains
            rospy.loginfo(str) # Timestamp string
            self.stalkPub.publish(self.behaviourGains['FIND_DARK'])
            self.findDarkPub.publish(self.behaviourGains['FIND_BLOB'])
            self.absMovePub.publish(self.behaviourGains['ABS_MOVE'])
            self.relMovePub.publish(self.behaviourGains['REL_MOVE'])
            
            pub.publish(str)
            r.sleep()
        
if __name__ == '__main__':
    try:
        t = TaskSequencer()
        t.execute()
    except rospy.ROSInterruptException: pass
