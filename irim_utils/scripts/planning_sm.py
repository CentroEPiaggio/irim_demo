#!/usr/bin/env python

"""
    This node does is the main planning and execution calling state machine for I-RIM Franka demo

    TODO: give a brief explanation of the main states and what they do

    Autors: George Jose Pollayil, Mathew Jose Pollayil
"""

# Main ROS Imports
import roslib
import rospy

# SMACH Imports
import smach
import smach_ros

# Aruco Imports
from aruco_msgs.msg import Marker

DEBUG = False
VERBOSE = False

object_topic = "aruco_marker_publisher/markers"

# DEFINITION OF STATES

# State Wait
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['no_obj_in_view', 'obj_in_view'])

        # Subscriber to object pose and id state and the saved message
        self.obj_sub = ospy.Subscriber(object_topic, Marker, self.callback, queue_size=1)
        self.marker_msg = None

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("Executing state Wait")

        # According to the presence or absence of objects, change state
        if self.marker_msg is None:
            return 'no_obj_in_view'
        else:
            return 'obj_in_view'