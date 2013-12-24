#!/usr/bin/python
#Author: Jackie Jin
#Leapmotion,Inc. reserves all the rights to the Leapmotion Python SDK
#used as part of the code

import Leap,sys
from Leap import *
from numpy import *
from collections import deque

#ROS imports
import rospy
import roslib; roslib.load_manifest("rosleap_msg")
from rosleap_msg.msg import *


def rosprint(string):
    rospy.loginfo(string)


#################### ROSLeapMsg ####################


class ROSLeapMsg:

    def __init__(self, frame):
        self.frame = frame
        #define msg
        self.LeapFrameMsg = LeapmotionMsg()
        
    def getMsg(self, gestureProcessor):
        #frameId
        self.LeapFrameMsg.frameID = self.frame.id
        #device vector:x,y,z,yaw,pitch,roll 
        vector = Leap.Vector()
        self.LeapFrameMsg.vector.cartesian = vector.to_float_array()
        self.LeapFrameMsg.vector.angular = [vector.pitch, vector.yaw, vector.roll]
        #time stamp
        self.LeapFrameMsg.timeStamp = self.frame.timestamp
        
        #hands
        for hand in self.frame.hands:
            self.LeapFrameMsg.hands.append(self.getHand(hand))
        
        #fingers
        for finger in self.frame.fingers:
            self.LeapFrameMsg.fingers.append(self.getFinger(finger))
                
        #tools
        #self.LeapFrameMsg['tools'] = self.frame.tools
        
        #pointables
        #self.LeapFrameMsg['pointables'] = self.frame.pointables
        
        #gestures
        self.LeapFrameMsg.gestures = gestureProcessor.getGestures()
        rosprint('ROSLeap: Swipes: ' + str(len(self.LeapFrameMsg.gestures.swipes)) + ' | Taps: ' + str(len(self.LeapFrameMsg.gestures.key_taps)))
        
        return self.LeapFrameMsg
    
    def getHand(self, hand):
        roshand = LeapHand()
        roshand.id = hand.id
        
        #fingers, tools, pointables
        for finger in hand.fingers:
            roshand.fingers.append(self.getFinger(finger))
        
        #hand data
        roshand.palm_pos.cartesian = hand.palm_position.to_float_array()
        roshand.palm_pos.angular = [
            hand.palm_position.pitch,
            hand.palm_position.yaw,
            hand.palm_position.roll
            ]
        roshand.palm_vel.cartesian = hand.palm_velocity.to_float_array()
        roshand.palm_normal.cartesian = hand.palm_normal.to_float_array()
        roshand.direction.cartesian = hand.direction.to_float_array()
        roshand.sphere_center.cartesian = hand.sphere_center.to_float_array()
        roshand.translation.cartesian = hand.translation(self.frame).to_float_array()
        
        roshand.isValid = hand.is_valid
        roshand.sphere_radius = hand.sphere_radius
        return roshand
    
    def getFinger(self, finger):
        rosfinger = LeapFinger()
        rosfinger.id = finger.id
        rosfinger.tip_position.cartesian = finger.tip_position.to_float_array()
        rosfinger.tip_velocity.cartesian = finger.tip_velocity.to_float_array()
        rosfinger.direction.cartesian = finger.direction.to_float_array()
        rosfinger.is_finger = finger.is_finger
        rosfinger.is_valid = finger.is_valid
        return rosfinger
    
    def getTool(self, tool):
        pass
    
    def getPointable(self, pointable):
        pass

    def getGesture(self, gesture):
        pass


#################### ROSLeapGesture ####################


class ROSLeapGesture:

    def __init__(self, maxframes):
        self.MAXFRAMES = maxframes
        self.frameList = deque()

    def pushFrame(self, frame):
        if len(self.frameList) == MAXFRAMES:
            self.frameList.pop()
        self.frameList.appendleft(frame)

    def getGestures(self):
        gestures = LeapGestureList()
        gestures.swipes = self.getSwipes()
        gestures.key_taps = self.getKeyTaps()
        return gestures

    def getSwipes(self):
        swipeList = []
        for gesture in self.frameList[0].gestures():
            if gesture.type == Gesture.TYPE_SWIPE:
                swipe = SwipeGesture(gesture)
                swipeMsg = LeapSwipeGesture()
                swipeMsg.start_pos = swipe.startPosition.to_float_array()
                swipeMsg.cur_pos = swipe.position.to_float_array()
                swipeMsg.direction = swipe.direction.to_float_array()
                swipeMsg.speed = swipe.speed
                swipeList.append(swipeMsg)
        return swipeList

    def getKeyTaps(self):
        keyTapList = []
        for gesture in self.frameList[0].gestures():
            if gesture.type == Gesture.TYPE_KEY_TAP:
                keyTap = KeyTapGesture(gesture)
                keyTapMsg = LeapKeyTapGesture()
                keyTapMsg.position = keyTap.position.to_float_array()
                keyTapMsg.direction = keyTap.direction.to_float_array()
                keyTapList.append(keyTapMsg)
        return keyTapList


#################### ROSLeapListener ####################
    

class ROSLeapListener(Leap.Listener):

    def on_init(self, controller):
        rospy.init_node('ROSLeapNode')
        rosprint("initialzing leapmotion...")
        #ROS topic handle
        self.LeapPub = rospy.Publisher('/leap', LeapmotionMsg)
        self.gestureProcessor = ROSLeapGesture(1)
        
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
        
        #Allow ROSLeapGesture to also receive frame
        self.gestureProcessor.pushFrame(frame)
        #generate ROSLeap msg
        rosleapMsg = ROSLeapMsg(frame)
        #publish
        self.publish(rosleapMsg.getMsg(gestureProcessor))
    
    def publish(self, msg):
        self.LeapPub.publish(msg)

if __name__ == "__main__":
    listener = ROSLeapListener()
    controller = Leap.Controller()
    try:
        controller.add_listener(listener)
        rospy.spin()
    except rospy.ROSInterruptException:
        #interrupt and detatch the listener
        controller.remove_listener(listener)

