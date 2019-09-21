#!/usr/bin/env python

"""
    This node does is the main planning and execution calling state machine for I-RIM Franka demo

    TODO: give a better explanation of the main states and what they do

    The state machine is made of some outer states and a sub state machine, the input point is 'Wait' and there is no
    output point.

    OUTER STATES: Wait, NothingToDo
    SUB SM : PlanAndExecute -> SUB SM STATES: GraspService, GraspCheck, PlaceService

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
VERBOSE = True

object_topic = "aruco_marker_publisher/markers"

# DEFINITION OF STATES

# State Wait
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['no_obj_in_view', 'obj_in_view'])

        # Subscriber to object pose and id state and the saved message
        self.obj_sub = rospy.Subscriber(object_topic, Marker, self.obj_callback, queue_size=1)
        self.last_marker_msg = None

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("Executing state Wait")

        # According to the presence or absence of objects, change state
        # The callback simply saves the message: checking its sequential no. to know if it's new
        if self.last_marker_msg is None:
            return 'no_obj_in_view'
        else:
            return 'obj_in_view'

    def obj_callback(self, data):
        if self.last_marker_msg is None:
            self.last_marker_msg = data
        else:
            if self.last_marker_msg.header.stamp == data.header.stamp:
                self.last_marker_msg = None
            else:
                self.last_marker_msg = data


# State NothingToDO: auxiliary state for Wait in order to avoid self transitions
class NothingToDo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_wait'])

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo('Nothing to do for now! Going back to wait.')

        return 'go_to_wait'

# DEFINITION OF STATES OF SUB STATE MACHINE PLAN AND EXECUTE (the service states need not to be defined here)

# State PrepareGrasp
class PrepareGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_wait', 'go_to_grasp'])

        # Subscriber to object pose and id state and the saved message
        self.obj_sub = rospy.Subscriber(object_topic, Marker, self.obj_callback, queue_size=1)
        self.last_marker_msg = None

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I'm preparing the grasp now.")

        # TODO: Listen to object_topic and set the object: if everything goes well go to grasp
        # TODO: Need to check if msg is None
        # TODO: Then the userdata should be passed to CheckGrasp

    def obj_callback(self, data):
        self.last_marker_msg = data