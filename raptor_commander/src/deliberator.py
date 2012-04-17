#!/usr/bin/env python

'''
Created on 9/04/2012

@author: jdh
'''
import roslib; roslib.load_manifest('raptor_commander')
import rospy
import raptor_commander.srv
import raptor_commander.msg
import rovio_shared.srv
import std_msgs
import math

class Deliberator():
    
    preyFront = -1
    preyBack = 1
    preyGone = 0
    
    # Previous best dark spot
    x = 0
    y = 0
    darkness = 255
    newThreshold = 255
    
    # Approach speed
    lastX = 0
    lastY = 0
    approachSpeed = 0
    
    # Provided service functions.
    def giveAdviceFindDark(self, req):
        rospy.loginfo("deliberator.giveAdviceFindDark")
        return raptor_commander.srv.getAdviceFDResponse(self.newThreshold, self.x, self.y, self.darkness)
        
    def giveAdvicePlayDead(self, req):
        rospy.loginfo("deliberator.giveAdvicePlayDead")
        return raptor_commander.srv.getAdvicePDResponse(self.approachSpeed) 
    

# SUBSCRIBED DATA - PUSH DATA INTO INTERNAL VARIABLES
    def darknessCallback(self, data):
        # If this darkness is better than the previous best, store the value and current location
        # rospy.loginfo(rospy.get_caller_id()+" d_Darkness heard %f",data.data)
        darknessCoefficient = data.data
        if darknessCoefficient <= self.darkness:
            self.darkness = darknessCoefficient
            #resp = self.getRovioPosition()
            #self.x = response.

    def darknessRegionCallback(self, data):
        self.darknessRegion = data.data
        #print data.data
        
    def preyCallback(self, data):
        #rospy.loginfo(rospy.get_caller_id()+"I heard %f",data.data)
        #print 'd_preyCallback'
        self.colour = data.colourID
        
        self.lastX = self.x
        self.lastY = self.y
        
        self.x = data.x
        self.y = data.y
        
        self.approachSpeed = self.y - self.lastY
        
        if self.approachSpeed < 0:
	    self.approachSpeed = 0
        
        #print "x %s, y %s, colour %s" % (self.x,self.y,self.colour)

    def getRovioPosition(self):
        rospy.wait_for_service('rovio_position')
        try:
            rovioPositionProxy = rospy.ServiceProxy('rovio_position', rovio_shared.srv.rovio_position)
            response = rovioPositionProxy()
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Main server function - called on startup.
    def server(self):
        rospy.init_node('Deliberator')
        
        # Darkness coefficient for the area near the robot
        rospy.Subscriber("DARKNESS_COEFF", std_msgs.msg.Float32, self.darknessCallback)
        # Darkness dataset used by the deliberator
        rospy.Subscriber("DARKNESS_REGION", raptor_commander.msg.darkness_region, self.darknessRegionCallback)
        # Prey data used by the deliberator - colour of blobs in vision.
        rospy.Subscriber("BLOB_COLOUR", raptor_commander.msg.blob_colour, self.preyCallback)
        
        # (serviceName, serviceType, function) 
        self.fdAdvice = rospy.Service('adviceFD', raptor_commander.srv.getAdviceFD, self.giveAdviceFindDark)
        self.detAdvice = rospy.Service('advicePD', raptor_commander.srv.getAdvicePD, self.giveAdvicePlayDead)
        
        
        rospy.loginfo("Deliberator Ready.")
        rospy.spin()

if __name__ == "__main__":
    deliberator = Deliberator()
    deliberator.server()
