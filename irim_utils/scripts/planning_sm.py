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
from smach_ros import ServiceState

# Aruco Imports
from aruco_msgs.msg import Marker

# Custom imports (services for panda_softhand_control task_sequencer)
from std_srvs.srv import SetBool, SetBoolRequest
from panda_softhand_control.srv import set_object, set_objectRequest

DEBUG = False
VERBOSE = True

# Topic and service names
object_topic = "aruco_marker_publisher/markers"
set_obj_service_name = '/set_object'
grasp_service_name = '/grasp_task'
place_service_name = '/place_task'
home_service_name = '/home_task'

# A dictionary associating ids with object names
obj_dict = {
    1 : 'ball',
    2 : 'teddy',
    3 : 'two_cubes'
}


# TODO: pass userdata through states in order to check if object removed (timestamps) if so act accordingly

# DEFINITION OF STATES

# State Wait
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_obj_in_view', 'obj_in_view'])

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


# DEFINITION OF STATES OF SUB STATE MACHINE PLAN AND EXECUTE (also the service states are defined as generic states)

# State PrepareGrasp
class PrepareGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_wait', 'go_to_grasp'],
                             output_keys=['prepare_grasp_out'])

        # Subscriber to object pose and id state and the saved message
        self.obj_sub = rospy.Subscriber(object_topic, Marker, self.obj_callback, queue_size=1)
        self.last_marker_msg = None

        # Service Proxy to set object service
        self.set_obj_client = rospy.ServiceProxy(set_obj_service_name, set_object)

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I'm preparing the grasp now.")

        # If no objects in scene return to wait
        if self.last_marker_msg is None:
            rospy.loginfo("In PrepareGraps no objects in the scene. Going back to wait")
            return 'go_to_wait'

        # Call set object according to id
        set_obj_req = set_objectRequest()
        set_obj_req.object_name = obj_dict.get(self.last_marker_msg.id)
        set_obj_res = self.set_obj_client(set_obj_req)

        # The userdata passed to CheckGrasp
        userdata.prepare_grasp_out = self.last_marker_msg

        # Change state according to result
        if set_obj_res.result:
            return 'go_to_grasp'
        else:
            return 'go_to_wait'

    def obj_callback(self, data):
        if self.last_marker_msg is None:
            self.last_marker_msg = data
        else:
            if self.last_marker_msg.header.stamp == data.header.stamp:
                self.last_marker_msg = None
            else:
                self.last_marker_msg = data


# State GraspService
class GraspService(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_check', 'error_grasp'])

        # Service Proxy to grasp service
        self.grasp_client = rospy.ServiceProxy(grasp_service_name, SetBool)

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I will perform the grasp now.")

        # Creating a service request and sending
        set_bool_req = SetBoolRequest(True)
        set_bool_res = self.grasp_client(set_bool_req)

        # Changing states according to res
        if set_bool_res.success:
            return 'go_to_check'
        else:
            return 'error_grasp'


# State CheckGrasp
class CheckGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_place', 'try_regrasp'])

        # Subscriber to object pose and id state and the saved message
        self.obj_sub = rospy.Subscriber(object_topic, Marker, self.obj_callback, queue_size=1)
        self.last_marker_msg = None

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I'm checking if I really grasped.")

        # TODO: Get the userdata from PrepareGrasp
        # TODO: Listen to object_topic and check if grasped: if so go to place, else try to regrasp
        # TODO: Need to check if msg is None

    def obj_callback(self, data):
        if self.last_marker_msg is None:
            self.last_marker_msg = data
        else:
            if self.last_marker_msg.header.stamp == data.header.stamp:
                self.last_marker_msg = None
            else:
                self.last_marker_msg = data


# State PlaceService
class PlaceService(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_wait', 'error_place'])

        # Service Proxy to place service
        self.place_client = rospy.ServiceProxy(place_service_name, SetBool)

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I will perform placing now.")

        # Creating a service request and sending
        set_bool_req = SetBoolRequest(True)
        set_bool_res = self.place_client(set_bool_req)

        # Changing states according to res
        if set_bool_res.success:
            return 'go_to_wait'
        else:
            return 'error_place'


# State HomeService
class HomeService(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_wait'])

        # Service Proxy to home service
        self.home_client = rospy.ServiceProxy(home_service_name, SetBool)

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I will perform homing now.")

        # Creating a service request and sending
        set_bool_req = SetBoolRequest(True)
        set_bool_res = self.home_client(set_bool_req)

        # Anyways go to wait
        return 'go_to_wait'


# Defining the State Machine
def main():
    rospy.init_node('planning_sm')

    # Waiting for the necessary services
    rospy.wait_for_service(set_obj_service_name)
    rospy.wait_for_service(grasp_service_name)
    rospy.wait_for_service(place_service_name)
    rospy.wait_for_service(home_service_name)

    # Creating the top level state machine
    sm_top = smach.StateMachine(outcomes=[])

    # Open the container
    with sm_top:
        # The entering point is Wait
        smach.StateMachine.add('WAIT', Wait(),
                               transitions={'no_obj_in_view': 'NOTHING_TO_DO',
                                            'obj_in_view': 'PREPARE_GRASP'})

        # The second outer state is NotingToDo
        smach.StateMachine.add('NOTHING_TO_DO', NothingToDo(),
                               transitions={'go_to_wait': 'WAIT'})

        # Create the sub state machine which does all the planning
        sm_sub = smach.StateMachine(outcomes=['go_to_wait'])

        # Open the sub container
        with sm_sub:
            # The entering point is PrepareGrasp
            smach.StateMachine.add('PREPARE_GRASP', PrepareGrasp(),
                                   transitions={'go_to_wait': 'WAIT',
                                                'go_to_grasp': 'GRASP_SERVICE'})

            # The service state for grasping
            smach.StateMachine.add('GRASP_SERVICE', GraspService(),
                                   transitions={'go_to_check': 'CHECK_GRASP',
                                                'error_grasp': 'HOME_SERVICE'})

        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'outcome4': 'outcome5'})

    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == '__main__':
    main()
