#!/usr/bin/python
#Author: Jackie Jin
#Leapmotion,Inc. reserves all the rights to the Leapmotion Python SDK
#used as part of the code

import Leap,sys
from Leap import *
from numpy import *
from collections import deque
import math

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

        if self.LeapFrameMsg.gestures.swipes:
            dataList = ['{0}|{1}'.format(g.cur_pos, g.speed) for g in self.LeapFrameMsg.gestures.swipes]
            printMsg = ' --- '.join(dataList)
            rosprint(printMsg)
        
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
        if len(self.frameList) == self.MAXFRAMES:
            self.frameList.pop()
        self.frameList.appendleft(frame)

    def getGestures(self):
        gestures = LeapGestureList()
        gestures.swipes = self.getSwipes()
        gestures.key_taps = self.getKeyTaps()
        self.getSpread()
        return gestures

    def average(self, vlist):
        if not vlist:
            return float('nan')
        return float(sum(vlist)) / len(vlist)

    def magnitude(self, vec):
        if not vec:
            return float('nan')
        vsum = 0.0
        for i in vec:
            vsum += i**2
        return math.sqrt(vsum)

    def getSpread(self):
        spreadList = []
        if len(self.frameList) < 3:
            return spreadList
        for i in range(3):
            f = self.frameList[i]
            if not f.hands or len(f.hands[0].fingers) != 5:
                return spreadList
        spreadMsg = LeapSpreadGesture()
        fingers = self.frameList[0].hands[0].fingers
        avg_spdsum = 0.0
        avg_dirsum = [0.0, 0.0, 0.0]
        avg_possum = [0.0, 0.0, 0.0]
        for f in fingers:
            avg_spdsum += self.magnitude(f.tip_velocity.to_float_array())
            for i in range(3):
                avg_dirsum[i] += f.direction[i]
                avg_possum[i] += f.tip_position[i]
        avg_spd = avg_spdsum / 5.0
        avg_dir = [0, 0, 0]
        avg_pos = [0, 0, 0]
        for i in range(3):
            avg_dir[i] = avg_dirsum[i] / 5.0
            avg_pos[i] = avg_possum[i] / 5.0
        rosprint(" | ".join([str(avg_spd), str(avg_dir), str(avg_pos)]))

    def getSwipes(self):
        swipeList = []
        for gesture in self.frameList[0].gestures():
            if gesture.type == Gesture.TYPE_SWIPE:
                swipe = SwipeGesture(gesture)
                swipeMsg = LeapSwipeGesture()
                swipeMsg.start_pos.cartesian = swipe.start_position.to_float_array()
                swipeMsg.cur_pos.cartesian = swipe.position.to_float_array()
                swipeMsg.direction.cartesian = swipe.direction.to_float_array()
                swipeMsg.speed = swipe.speed
                if (
                   swipe.speed > 3000 and
                   swipe.direction.x < -0.8 and
                   abs(swipe.direction.y) < 0.15 and
                   abs(swipe.direction.z) < 0.40 and
                   swipe.position.y > 100 and swipe.position.y < 400 and
                   abs(swipe.position.z) < 150
                   ):
                    swipeList.append(swipeMsg)
        return swipeList

    def getKeyTaps(self):
        keyTapList = []
        for gesture in self.frameList[0].gestures():
            if gesture.type == Gesture.TYPE_KEY_TAP:
                keyTap = KeyTapGesture(gesture)
                keyTapMsg = LeapKeyTapGesture()
                keyTapMsg.position.cartesian = keyTap.position.to_float_array()
                keyTapMsg.direction.cartesian = keyTap.direction.to_float_array()
                if (
                   keyTap.direction.y < -0.9 and
                   abs(keyTap.position.x) < 50 and
                   abs(keyTap.position.z) < 50 and
                   keyTap.position.y > 100 and keyTap.position.y < 400
                   ):
                    keyTapList.append(keyTapMsg)
        return keyTapList


#################### ROSLeapListener ####################
    

class ROSLeapListener(Leap.Listener):

    def on_init(self, controller):
        rospy.init_node('ROSLeapNode')
        rosprint("initialzing leapmotion...")
        #ROS topic handle
        self.LeapPub = rospy.Publisher('/leap', LeapmotionMsg)
        self.gestureProcessor = ROSLeapGesture(3)
        
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
        self.publish(rosleapMsg.getMsg(self.gestureProcessor))
    
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

