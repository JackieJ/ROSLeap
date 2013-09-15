#!/usr/bin/python
#Author: Jackie Jin
#Leapmotion,Inc. reserves all the rights to the Leapmotion Python SDK
#used as part of the code

import Leap,sys
from Leap import *
from numpy import *

#ROS imports
import rospy
import roslib; roslib.load_manifest("rosleap_msg")
from rosleap_msg.msg import *



def rosprint(string):
    rospy.loginfo(string)

class ROSLeapListener(Leap.Listener):
    def on_init(self, controller):
        rospy.init_node('ROSLeapNode')
        rosprint("initialzing leapmotion...")
        #define msg
        
    def on_connect(self, controller):
        rosprint("leapmotion connected...ready to retrieve frame data...")
        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);
        
    #dispatched when the controller disconnects from the Leapmotion driver
    def on_disconnect(self, controller):
        rosprint("leapmotion disconnected...")
    
    #dispatched to the listener when it's removed from the controller
    def on_exit(self, controller):
        rosprint("leapmotion exited...")
    def on_frame(self, controller):
        frame = controller.frame()
        frameInfoStr = "[LEAP]FrameID:"+str(frame.id)
        rosprint(frameInfoStr)
        
    def publish(self):
        pass

if __name__ == "__main__":
    listener = ROSLeapListener()
    controller = Leap.Controller()
    try:
        controller.add_listener(listener)
        rospy.spin()
    except rospy.ROSInterruptException:
        #interrupt and detatch the listener
        controller.remove_listener(listener)
