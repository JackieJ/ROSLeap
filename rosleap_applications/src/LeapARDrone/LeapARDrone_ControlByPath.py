#!/usr/bin/python
#Author: Jackie Jin
#Leapmotion,Inc. reserves all the rights to the Leapmotion Python SDK
#used as part of the code

import rospy
import roslib; roslib.load_manifest("geometry_msgs")
import roslib; roslib.load_manifest("rosleap_msg")
from std_msgs.msg import *
from geometry_msgs.msg import *
from rosleap_msg.msg import *
import math
import numpy
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
       
        #constants
        self.QUEUESIZE = 10
        self.VELTHRESHHOLD = 1500
        self.MAXVAL = 99999

        #previous position
        self.prevPos = [self.MAXVAL, self.MAXVAL, self.MAXVAL]
        
    def run(self):
        #test
        rospy.Subscriber('/leap', LeapmotionMsg, self.pathVizTest)
        rospy.spin()
    
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
                averageLinearVel = [0,0,0]
                for finger in fingers:
                    vel_vector = finger.tip_velocity.cartesian                    
                    pos_vector = finger.tip_position.cartesian
                    
                    averageLinearTwist[0] += pos_vector[0]
                    averageLinearTwist[1] += pos_vector[1]
                    averageLinearTwist[2] += pos_vector[2]
                    
                    averageLinearVel[0] += vel_vector[0]
                    averageLinearVel[1] += vel_vector[1]
                    averageLinearVel[2] += vel_vector[2]
                    
                currentTwist = [0,0,0]
                for index in range(0,3):
                    averageLinearTwist[index] = averageLinearTwist[index]/numberofFinger
                    averageLinearVel[index] = averageLinearVel[index]/numberofFinger
                    if numpy.mean(self.prevPos) == self.MAXVAL:
                        self.prevPos[index] = averageLinearTwist[index]
                    else:
                        currentTwist[index] = averageLinearTwist[index] - self.prevPos[index]
            else:
                self.prevPos = [self.MAXVAL, self.MAXVAL, self.MAXVAL]
        else:
            #reset prevPos
            self.prevPos = [self.MAXVAL, self.MAXVAL, self.MAXVAL]
            
    def velPath(self, vel_vector):
        #map: x --> -y ; y --> z ; z --> -x
        
        pass
            
    def publish(self, data):
        self.LeapARPub.publish(data)        
            

if __name__ == "__main__":
    try:
        LeapAR = LeapARDroneControlByPath()
        LeapAR.run()
    except rospy.ROSInterruptException:
        rosprint('leapmotion control interrupted!')
