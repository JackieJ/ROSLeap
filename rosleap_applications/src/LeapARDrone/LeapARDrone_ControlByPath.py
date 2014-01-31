#!/usr/bin/python
#Author: Jackie Jin
#Leapmotion,Inc. reserves all the rights to the Leapmotion Python SDK
#used as part of the code

from numpy import *

import rospy
import rosbag
import roslib; roslib.load_manifest("geometry_msgs")
import roslib; roslib.load_manifest("rosleap_msg")
from std_msgs.msg import *
from geometry_msgs.msg import *
from rosleap_msg.msg import *
import math

#helper functions
def rosprint(string):
    rospy.loginfo(string)

class LeapARDroneControlByPath:
    def __init__(self):
        rospy.init_node('LeapARNode')
        rosprint('[ControlByPath]starting leapmotion-ardrone control...')
        #Topic Handlers
        self.LeapARPub = rospy.Publisher('/cmd_vel', Twist)
        self.TwistMsgs = [] #list of twist msgs from path drown
        self.bag = rosbag.Bag('path.bag', 'rw')
    
    def run(self):
        #test
        rospy.Subscriber('/leap', LeapmotionMsg, self.pathVizTest)
        rospy.spin()
    
    #helper functions
    def isVectorAboveThreshhold(self, vector, threshhold):
        for mem in vector:
            if mem > threshhold:
                return True
        return False
    
    def pathVizTest(self, data):
        #record coords, do regression and display the playback data
        
        pass

    def publish(self):
        self.LeapARPub.publish(self.TwistMsg)        
            

if __name__ == "__main__":
    try:
        LeapAR = LeapARDroneControlByPath()
        LeapAR.run()
    except rospy.ROSInterruptException:
        rosprint('leapmotion control interrupted!')
