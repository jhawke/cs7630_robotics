#!/usr/bin/env python

# setup ROS
import roslib;
roslib.load_manifest('nao_vision')
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_msgs.msg import Time
import sys

 # the path parameter is optional
path = rospy.get_param("/naoqi/path", "")
sys.path.append(path)

# attempt to load NAOqi
try:
    from naoqi import ALProxy
    from vision_definitions import*
except ImportError:
    rospy.logerr("NAOqi not found - Please check to see if it is in your PYTHONPATH variable.")
    exit(1)
    
class NaoVision():
    def __init__(self):
        rospy.init_node("nao_vision")
        
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
            self.nao_cam = ALProxy("ALVideoDevice", host, port)
        except Exception:
            rospy.logerr("Unable to create vision proxy.")
            exit(1)
            
        # get any optional parameters
        resolution = rospy.get_param("~resolution", 1)
        cam = rospy.get_param("~camera", 0)
        
        #setup the camera stream
        self.proxy_name = self.nao_cam.subscribe("nao_image", resolution, kRGBColorSpace, 30)
        self.nao_cam_pub = rospy.Publisher("nao_camera", Image)
        self.nao_cam.setParam(kCameraSelectID, cam)

        rospy.loginfo("NAO Vision Node Initialized")
    
    def publish_image(self):
        # get the image from the Nao
        img = self.nao_cam.getImageRemote(self.proxy_name)
        # copy the data into the ROS Image
        ros_img = Image()
        ros_img.width = img[0]
        ros_img.height = img[1]
        ros_img.step = img[2] * img[0]
        ros_img.header.stamp.secs = img[5]
        ros_img.data = img[6]
        ros_img.is_bigendian = False
        ros_img.encoding = "rgb8"
        ros_img.data = img[6]
        # publish the image
        self.nao_cam_pub.publish(ros_img)
        
    def disconnect(self):
        self.nao_cam.unsubscribe(self.proxy_name)

if __name__ == '__main__':
    vision = NaoVision()
    #operate at 30 Hz
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        vision.publish_image()
        rate.sleep()
    vision.disconnect()

