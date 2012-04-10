'''
Created on 9/04/2012

@author: jdh
'''
import roslib; roslib.load_manifest('raptor_commander')
import rospy
import raptor_commander.srv
import std_msgs

class Deliberator():
    
    # Previous best dark spot
    x = 0
    y = 0
    theta = 0
    darkness = 0
    
    # Approach speed
    approachSpeed = 0
    
    
    def giveAdviceDark(self, req):
        rospy.loginfo("deliberator.giveAdviceDark")
        return raptor_commander.srv.getAdviceFDResponse(self.x, self.y, self.theta, self.darkness)
        
    def giveAdviceDetected(self, req):
        rospy.loginfo("deliberator.giveAdviceDetected")
        return raptor_commander.srv.getAdviceDETResponse(self.approachSpeed) 
    
    def giveAdviceFlee(self, req):
        rospy.loginfo("deliberator.giveAdviceFlee")
        return raptor_commander.srv.getAdviceFLEEResponse(self.approachSpeed)

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


    def server(self):
        rospy.init_node('Deliberator')
        
         # Darkness parameter used by the task sequencer
        rospy.Subscriber("DARKNESS_PARAM", std_msgs.msg.Float32, self.darknessCallback)
        # Darkness dataset used by the deliberator
        rospy.Subscriber("DARKNESS_HISTOGRAM", std_msgs.msg.Float32MultiArray, self.darknessHistoCallback)
        # Prey data used by the task sequencer
        rospy.Subscriber("PREY_PARAM", std_msgs.msg.Float32MultiArray, self.preyCallback)
        
        # (serviceName, serviceType, function) 
        fdAdvice = rospy.Service('adviceFD', raptor_commander.srv.getAdviceFD, self.giveAdviceDark)
        detAdvice = rospy.Service('adviceDET', raptor_commander.srv.getAdviceDET, self.giveAdviceDetected)
        fleeAdvice = rospy.Service('adviceFLEE', raptor_commander.srv.getAdviceFLEE, self.giveAdviceFlee)
        rospy.loginfo("Deliberator Ready.")
        rospy.spin()

if __name__ == "__main__":
    deliberator = Deliberator()
    deliberator.server()
