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
        #ROS topic handle
        self.LeapMsg = LeapmotionMsg()
        self.LeapPub = rospy.Publisher('/leap', LeapmotionMsg)
        #define msg
        self.LeapFrameMsg = {
            'id':0,
            'timeStamp':0,
            'hands':[],
            'fingers':[],
            'tools':[],
            'gestures':[],
            'pointables':[],
            'conncted': False,
            'deviceVector':{
                'cartesian':[0,0,0], #x,y,z,
                'angular':[0,0,0] #pitch, yaw, roll
                }
            }
        
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
        #rosprint(str(frame.id))
        #frameId
        self.LeapFrameMsg['id'] = frame.id
        #device vector:x,y,z,yaw,pitch,roll 
        vector = Leap.Vector()
        #rosprint(str(vector.to_float_array()))
        #cartesian coordinates
        self.LeapFrameMsg['deviceVector']['cartesian'] = vector.to_float_array()
        #angular coordinates
        self.LeapFrameMsg['deviceVector']['angular'] = [vector.pitch, vector.yaw, vector.roll]
        #time stamp
        self.LeapFrameMsg['timeStamp'] = frame.timestamp
        #hands
        self.LeapFrameMsg['hands'] = frame.hands
        #fingers
        self.LeapFrameMsg['fingers'] = frame.fingers
        #tools
        self.LeapFrameMsg['tools'] = frame.tools
        #pointables
        self.LeapFrameMsg['pointables'] = frame.pointables
        #gestures
        self.LeapFrameMsg['gestures'] = frame.gestures()
        #rosprint(str(self.LeapFrameMsg))
        
        #publish
        self.publish()
    def publish(self):
        self.LeapMsg.frameID = self.LeapFrameMsg['id']
        self.LeapMsg.timeStamp = self.LeapFrameMsg['timeStamp']
        self.LeapMsg.vector.cartesian = self.LeapFrameMsg['deviceVector']['cartesian']
        self.LeapMsg.vector.angular = self.LeapFrameMsg['deviceVector']['angular']
        
        self.LeapPub.publish(self.LeapMsg)

if __name__ == "__main__":
    listener = ROSLeapListener()
    controller = Leap.Controller()
    try:
        controller.add_listener(listener)
        rospy.spin()
    except rospy.ROSInterruptException:
        #interrupt and detatch the listener
        controller.remove_listener(listener)
