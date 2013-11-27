#!/usr/bin/python

from collections import deque

import Leap, sys
from Leap import *

import rospy
import roslib; roslib.load_manifest("rosleap_msg")
from rosleap_msg.msg import *

class ROSLeapGesture:
    def __init__(self):
        self.MAXFRAMES = 1
        self.frame_list = deque()

    def getGestures(frame):
        if len(frame_list) == MAXFRAMES:
            frame_list.pop()
        frame_list.appendleft(frame)

        gestureList = LeapGestureList()
        gestureList.swipes = getSwipes()
        gestureList.keytaps = getKeyTaps()
        return gestureList

    def getSwipes():
        swipeList = []
        for gesture in frame_list[0].gestures():
            if gesture.type == Gesture.TYPE_SWIPE:
                swipe = SwipeGesture(gesture)
                swipeMsg = LeapSwipeGesture()
                swipeMsg.start_pos = swipe.startPosition.to_float_array()
                swipeMsg.cur_pos = swipe.position.to_float_array()
                swipeMsg.direction = swipe.direction.to_float_array()
                swipeMsg.speed = swipe.speed
                swipeList.append(swipeMsg)
        return swipeList

    def getKeyTaps():
        keyTapList = []
        for gesture in frame_list[0].gestures():
            if gesture.type == Gesture.TYPE_KEY_TAP:
                keyTap = KeyTapGesture(gesture)
                keyTapMsg = LeapKeyTapGesture()
                keyTapMsg.position = keyTap.position.to_float_array()
                keyTapMsg.direction = keyTap.direction.to_float_array()
                keyTapList.append(keyTapMsg)
        return keyTapList
