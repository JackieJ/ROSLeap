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
from collections import deque

#helper functions
def rosprint(string):
    rospy.loginfo(string)

class LeapARDroneControlByPath:
    def __init__(self):
        rospy.init_node('LeapARNode')
        rosprint('[ControlByPath]starting leapmotion-ardrone control...')
        #Topic Handlers
        self.LeapARPub = rospy.Publisher('/cmd_vel', Twist)
        self.pathMsgs = deque() #queue of path drawn by the finger
        self.twistMsgs = deque() #queue of twist msgs from path drawn
        self.pathBag = rosbag.Bag('path.bag', 'w')
        self.twistBag = rosbag.Bag('twist.bag', 'w')
    
        #constants
        self.QUEUESIZE = 10
        self.POSTHRESHHOLD = 5
        
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
    
    def twistMsgGenerator(self, bag):
        pass
    
    def pathVizTest(self, data):
        twistMsg = Twist()
        #record coords, do regression and display the playback data
        hands = data.hands
        #activate drawing when 
        if len(hands) == 1:
            fingers = hands[0].fingers
            numberofFinger = len(fingers)
            if numberofFinger != 0:
                #linear twist, x,y,z
                averageLinearTwist = [0,0,0]
                for finger in fingers:
                    pos_vector = finger.tip_position.cartesian
                    averageLinearTwist[0] += pos_vector[0]
                    averageLinearTwist[1] += pos_vector[1]
                    averageLinearTwist[2] += pos_vector[2]
                #average twist
                for index in range(0,3):
                    averageLinearTwist[index] = averageLinearTwist[index]/numberofFinger
                #write to the twist bag
                
                
        else:
            #replay data to generate twist msg
            twistMsgGenerator(self.pathBag)
    
    def publish(self, data):
        self.LeapARPub.publish(data)        
            

if __name__ == "__main__":
    try:
        LeapAR = LeapARDroneControlByPath()
        LeapAR.run()
    except rospy.ROSInterruptException:
        rosprint('leapmotion control interrupted!')
