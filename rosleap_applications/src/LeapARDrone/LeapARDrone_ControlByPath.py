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
import math

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
        self.VELTHRESH = 150
        self.XTHRESH = 50
        self.YTHRESH = 300
        self.ZTHRESH = 30
        self.XVELTHRESH = 100
        self.YVELTHRESH = 300
        self.ZVELTHRESH = 100
        self.YAWTHRESH = 0.5
        self.prevPos = [0,0,0]
        
    def run(self):
        rospy.Subscriber('/leap', LeapmotionMsg, self.linearControlByFingerPos)
        #rospy.Subscriber('/leap', LeapmotionMsg, self.restrictedControl)
        rospy.spin()
    
    #helper functions
    def isVectorAboveThreshhold(self, vector, threshhold):
        for mem in vector:
            if mem > threshhold:
                return True
        return False
    
    #control callbacks
    def linearControlByFingerPos(self, data):
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
                #direction to yaw
                averageLinearDirection = [0,0,0]
                for finger in fingers:
                    vel_vector = finger.tip_velocity.cartesian
                    pos_vector = finger.tip_position.cartesian
                    direction_vector = finger.direction.cartesian
                    #add up vel values and average after loop
                    averageLinearVel[0] += vel_vector[0]
                    averageLinearVel[1] += vel_vector[1]
                    averageLinearVel[2] += vel_vector[2]
                    #add up pos values and average after loop
                    averageLinearTwist[0] += pos_vector[0]
                    averageLinearTwist[1] += pos_vector[1]
                    averageLinearTwist[2] += pos_vector[2]
                    #direction
                    averageLinearDirection[0] += direction_vector[0]
                    averageLinearDirection[1] += direction_vector[1]
                    averageLinearDirection[2] += direction_vector[2]
                    
                #average vel and twist
                for index in range(0,3):
                    averageLinearTwist[index] = averageLinearTwist[index]/numberofFinger
                    averageLinearVel[index] = averageLinearVel[index]/numberofFinger
                    averageLinearDirection[index] = averageLinearDirection[index]/numberofFinger
                    
                isDirectionChanged = False
                yaw = averageLinearDirection[0]
                if math.fabs(yaw) > self.YAWTHRESH:
                    isDirectionChanged = True
                    if yaw > 0:
                        self.TwistMsg.angular.z = -1
                    else:
                        self.TwistMsg.angular.z = 1
                
                if isDirectionChanged == False:
                    x = averageLinearTwist[0] - self.prevPos[0]
                    if math.fabs(x) > self.XTHRESH:
                        if x > 0:
                            self.TwistMsg.linear.y = -1
                        else:
                            self.TwistMsg.linear.y = 1
                
                    y = averageLinearVel[1]
                    if math.fabs(y) > self.YTHRESH:
                        if y > 0:
                            self.TwistMsg.linear.z = 1
                        else:
                            self.TwistMsg.linear.z = -1
                    
                    z = averageLinearTwist[2] - self.prevPos[2]
                    if math.fabs(z) > self.ZTHRESH:
                        if z > 0:
                            self.TwistMsg.linear.x = -1
                        else:
                            self.TwistMsg.linear.x = 1
                    
                self.publish()
                self.TwistMsg.linear.x = 0
                self.TwistMsg.linear.y = 0
                self.TwistMsg.linear.z = 0
                self.TwistMsg.angular.z = 0
                

    def restrictedControl(self, data):
        #rosprint('[LeapAR][LinearControl] processing data...')
        #obtain the hand 
        hands = data.hands
        if len(hands) != 0:
            fingers = hands[0].fingers
            numberofFinger = len(fingers)
            if numberofFinger != 0:
                #linear twist, x,y,z
                averageLinearDirection = [0,0,0]
                #velocity needs to be beyond the threshhold to qualify for publishing
                averageLinearVel = [0,0,0]
                for finger in fingers:
                    vel_vector = finger.tip_velocity.cartesian
                    direction_vector = finger.direction.cartesian
                    #add up vel values and average after loop
                    averageLinearVel[0] += vel_vector[0]
                    averageLinearVel[1] += vel_vector[1]
                    averageLinearVel[2] += vel_vector[2]
                    #add up pos values and average after loop
                    averageLinearDirection[0] += direction_vector[0]
                    averageLinearDirection[1] += direction_vector[1]
                    averageLinearDirection[2] += direction_vector[2]
                
                #average vel and twist
                for index in range(0,3):
                    averageLinearDirection[index] = averageLinearDirection[index]/numberofFinger
                    averageLinearVel[index] = averageLinearVel[index]/numberofFinger
                
                isDirectionChanged = False
                yaw = averageLinearDirection[0]
                if math.fabs(yaw) > self.YAWTHRESH:
                    isDirectionChanged = True
                    if yaw > 0:
                        self.TwistMsg.angular.z = -1
                    else:
                        self.TwistMsg.angular.z = 1
                    
                if isDirectionChanged == False:
                    x = averageLinearVel[0]
                    if math.fabs(x) > self.XVELTHRESH:
                        if x > 0:
                            self.TwistMsg.linear.y = -1
                        else:
                            self.TwistMsg.linear.y = 1
                            
                    y = averageLinearVel[1]
                    if math.fabs(y) > self.YVELTHRESH:
                        if y > 0:
                            self.TwistMsg.linear.z = 1
                        else:
                            self.TwistMsg.linear.z = -1
                            
                    z = averageLinearVel[2]
                    if math.fabs(z) > self.ZVELTHRESH:
                        if z > 0:
                            self.TwistMsg.linear.x = -1
                        else:
                            self.TwistMsg.linear.x = 1
                                                                    
                self.publish()
                self.TwistMsg.linear.x = 0
                self.TwistMsg.linear.y = 0
                self.TwistMsg.linear.z = 0
                self.TwistMsg.angular.z = 0
                
    def sphereControl(self, data):
        pass
                
    def publish(self):
        self.LeapARPub.publish(self.TwistMsg)        
            

if __name__ == "__main__":
    try:
        LeapAR = LeapARDrone()
        LeapAR.run()
    except rospy.ROSInterruptException:
        rosprint('leapmotion control interrupted!')
