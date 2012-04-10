#!/usr/bin/env python

# setup ROS
import roslib;
roslib.load_manifest('nao_vision')
import rospy
from std_msgs.msg import String 
from std_msgs.msg import Float32
import sys

 # the path parameter is optional
path = rospy.get_param("/naoqi/path", "")
sys.path.append(path)

# attempt to load NAOqi
try:
    from naoqi import ALProxy
except ImportError:
    rospy.logerr("NAOqi not found - Please check to see if it is in your PYTHONPATH variable.")
    exit(1)
    
class NaoSpeech():
    def __init__(self):
        rospy.init_node("nao_speech")
        
        # attempt to get the host and port
        try:
            host = rospy.get_param("/naoqi/host")
        except KeyError:
            rospy.logerr("Unable to find parameter /naoqi/host")
            exit(1)
        try:
            port = rospy.get_param("/naoqi/port")
        except KeyError:
            rospy.logerr("Unable to find parameter /naoqi/port")
            exit(1)
        
        #connect to the Nao
        try:
            self.tts = ALProxy("ALTextToSpeech", host, port)
        except Exception:
            rospy.logerr("Unable to create speech proxy.")
            exit(1)
            
        # get any optional parameters
        volume = rospy.get_param("~volume", 0.5)
        lang = rospy.get_param("~language", "English")
        
        #setup the node and the TTS module
        self.tts.post.setLanguage(lang)
        self.tts.post.setVolume(volume)
        rospy.Subscriber("nao_say", String, self.nao_say_callback)
        rospy.Subscriber("nao_set_volume", Float32, self.nao_set_volume_callback)
        rospy.Subscriber("nao_set_lang", String, self.nao_set_lang_callback)

        rospy.loginfo("NAO Speech Node Initialized")
        
    def nao_say_callback(self, data):
        self.tts.post.say(data.data)
        
    def nao_set_volume_callback(self, data):
        self.tts.post.setVolume(data.data)
        
    def nao_set_lang_callback(self, data):
        self.tts.post.setLanguage(data.data)

if __name__ == '__main__':
    NaoSpeech = NaoSpeech()
    rospy.spin()

