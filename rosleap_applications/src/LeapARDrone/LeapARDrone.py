#!/usr/bin/python
#Author: Jackie Jin
#Leapmotion,Inc. reserves all the rights to the Leapmotion Python SDK
#used as part of the code

from numpy import *

import rospy
import roslib; roslib.load_manifest("geometry_msgs")
import roslib; roslib.load_manifest("rosleap_msg")
from geometry_msgs.msg import *
from rosleap_msg.msg import *

#helper functions
def rosprint(string):
    rospy.loginfo(string)

class LeapARDrone:
    def __init__(self):
        rospy.init_node('LeapARNode')
        rosprint('starting leapmotion-ardrone control...')
        #Topic Handlers
        self.LeapARPub = rospy.Publisher('/cmd_vel', Twist)
        self.TwistMsg = Twist()
        self.VELTHRESH = 200
        self.XTHRESH = 100
        self.YTHRESH = 100
        self.ZTHRESH = 100
        self.prevPos = [0,0,0]
        
    def run(self):
        #linear control
        rospy.Subscriber('/leap', LeapmotionMsg, self.linearControl)
        rospy.spin()
    
    #helper functions
    def isVectorAboveThreshhold(self, vector, threshhold):
        for mem in vector:
            if mem > threshhold:
                return True
        return False
    
    #control callbacks
    def linearControl(self, data):
        #rosprint('[LeapAR][LinearControl] processing data...')
        #obtain the hand 
        hands = data.hands
        if len(hands) != 0:
            fingers = hands[0].fingers
            numberofFinger = len(fingers)
            if numberofFinger != 0:
                #linear twist, x,y,z
                averageLinearTwist = [0,0,0]
                #velocity needs to be beyond the threshhold to qualify for publishing
                averageLinearVel = [0,0,0]
                isReadyToGo = False
                for finger in fingers:
                    vel_vector = finger.tip_velocity.cartesian
                    pos_vector = finger.tip_position.cartesian
                    #add up vel values and average after loop
                    averageLinearVel[0] += vel_vector[0]
                    averageLinearVel[1] += vel_vector[1]
                    averageLinearVel[2] += vel_vector[2]
                    #add up pos values and average after loop
                    averageLinearTwist[0] += pos_vector[0]
                    averageLinearTwist[1] += pos_vector[1]
                    averageLinearTwist[2] += pos_vector[2]
                
                #average
                for index in range(0,3):
                    averageLinearTwist[index] = averageLinearTwist[index]/numberofFinger
                    averageLinearVel[index] = averageLinearVel[index]/numberofFinger
                
                isReadyToGo = self.isVectorAboveThreshhold(averageLinearVel, self.VELTHRESH)
                if isReadyToGo:
                    if averageLinearTwist[0] > 0:
                        self.TwistMsg.linear.x = 1
                    else:
                        self.TwistMsg.linear.x = -1
                    
                    #if averageLinearTwist[1] > 0:
                    #    self.TwistMsg.linear.y = 1
                    #else:
                    #    self.TwistMsg.linear.y = -1
                    
                    if averageLinearTwist[2] > 0:
                        self.TwistMsg.linear.z = 1
                    else:
                        self.TwistMsg.linear.z = -1
                    
                    self.publish()
                

    def angularControl(self, data):
        pass
    
    def fusedControl(self, data):
        pass
    
    def publish(self):
        self.LeapARPub.publish(self.TwistMsg)        
            

if __name__ == "__main__":
    try:
        LeapAR = LeapARDrone()
        LeapAR.run()
    except rospy.ROSInterruptException:
        rosprint('leapmotion control interrupted!')
