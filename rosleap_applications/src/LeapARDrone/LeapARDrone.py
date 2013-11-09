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
        
        
    def run(self):
        #linear control
        rospy.Subscriber('/leap', LeapmotionMsg, self.linearControl)
        rospy.spin()
    
    def linearControl(self, data):
        #rosprint('[LeapAR][LinearControl] processing data...')
        
        pass
    
    def angularControl(self, data):
        pass
    
    def fusedControl(self, data):
        pass
    
    def publish(self):
        self.LeapARPub.publish(TwistMsg)        
        pass
    

if __name__ == "__main__":
    try:
        LeapAR = LeapARDrone()
        LeapAR.run()
    except rospy.ROSInterruptException:
        rosprint('leapmotion control interrupted!')
