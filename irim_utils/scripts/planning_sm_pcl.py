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
import actionlib

# General Imports
import math

# SMACH Imports
import smach
import smach_ros
from smach_ros import ServiceState

# Geometry Imports
import PyKDL
from geometry_msgs.msg import Pose
import tf_conversions.posemath as pm

# Trajectory Imports
from trajectory_msgs.msg import JointTrajectory

# Aruco Imports
from aruco_msgs.msg import Marker
from aruco_msgs.msg import MarkerArray

# Clusters Import
from irim_vision.msg import IdentifiedClustersArray
from irim_vision.msg import IdentifiedCluster

# Franka Imports
from franka_msgs.msg import Errors
from franka_control.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal

# Custom imports (services for panda_softhand_control task_sequencer)
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from panda_softhand_control.srv import set_object, set_objectRequest
from panda_softhand_control.srv import hand_plan, hand_planRequest
from panda_softhand_control.srv import hand_control, hand_controlRequest
from panda_softhand_control.srv import complex_grasp, complex_graspRequest, complex_graspResponse

DEBUG = False
VERBOSE = True

max_id = 999            # If you change this, change it also in object camera publisher
epsilon = 0.04          # The distance in cm for checking if the object has been grasped

# Topic and service names
object_topic = "irim_demo/cluster_chosen_object"
cam_pose_topic = "irim_demo/camera_pose"
identified_clusters_topic = "irim_vision/identified_clusters"
set_obj_service_name = '/set_object_service'
set_place_service_name = '/set_place_service'
grasp_service_name = '/complex_grasp_task_service'
place_service_name = '/place_task_service'
home_service_name = '/home_task_service'
hand_plan_service_name = '/hand_plan_service'
hand_control_service_name = '/hand_control_service'
rob_ns = "/panda_arm"
err_rec_topic = rob_ns + "/franka_control/error_recovery"
franka_states_topic = rob_ns + "/franka_state_controller/franka_states/current_errors"      # as of now seems not to be needed

# A dictionary associating ids with object names
obj_dict = {
    1: 'red',
    2: 'green',
    3: 'blue',
    4: 'black',
    5: 'white',
}


# TODO: pass userdata through states in order to check if object removed (timestamps) if so act accordingly

# DEFINITION OF STATES

# State Wait
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_obj_in_view', 'obj_in_view'])

        # Subscriber to object pose and id state and the saved message
        self.obj_sub = rospy.Subscriber(object_topic, IdentifiedCluster, self.obj_callback, queue_size=1)
        self.last_object_msg = None

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("Executing state Wait")

        rospy.sleep(0.5) # Sleeps for half a sec

        # According to the presence or absence of objects, change state
        # The callback simply saves the message: if it's empty don't do anything
        if self.last_object_msg is None or self.last_object_msg.obj_id == max_id + 1:
            return 'no_obj_in_view'
        else:
            self.last_object_msg = None         # Clearing the memory of wait
            return 'obj_in_view'

    def obj_callback(self, data):
        self.last_object_msg = data


# DEFINITION OF STATES OF SUB STATE MACHINE PLAN AND EXECUTE (also the service states are defined as generic states)

# State PrepareGrasp
class PrepareGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_wait', 'go_to_grasp'],
                             output_keys=['prepare_grasp_out'])

        # Subscriber to object pose and id state and the saved message
        self.obj_sub = rospy.Subscriber(object_topic, IdentifiedCluster, self.obj_callback, queue_size=1)
        self.last_object_msg = None

        # Service Proxy to set object service
        self.set_obj_client = rospy.ServiceProxy(set_obj_service_name, set_object)

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I'm preparing the grasp now.")

        rospy.sleep(0.5) # Sleeps for half a sec for not getting the placing hand cluster

        # If no objects in scene return to wait
        if self.last_object_msg is None:
            rospy.loginfo("In PrepareGraps no objects in the scene. Going back to wait")
            return 'go_to_wait'

        # Call set object according to obj_id
        set_obj_req = set_objectRequest()

        # Checking if the tag obj_id has an object associated in the dictionary
        if self.last_object_msg.obj_id <= len(obj_dict):
            set_obj_req.object_name = obj_dict.get(self.last_object_msg.obj_id)
        else:
            rospy.logerr("The obj_id of the object is not in the objects dictionary. Assigning the default...")
            set_obj_req.object_name = obj_dict.get(1)
        
        # Calling the service
        set_obj_res = self.set_obj_client(set_obj_req)

        # The userdata passed to CheckGrasp
        userdata.prepare_grasp_out = self.last_object_msg

        # Change state according to result
        if set_obj_res.result and self.last_object_msg is not None and self.last_object_msg.obj_id != max_id + 1:
            rospy.loginfo("In PrepareGrasp, no problems: going to grasp!")
            self.last_object_msg = None         # Clearing the memory of prepare grasp
            return 'go_to_grasp'
        else:
            rospy.loginfo("In PrepareGrasp, errors: going to wait!")
            self.last_object_msg = None         # Clearing the memory of prepare grasp
            return 'go_to_wait'

    def obj_callback(self, data):
        self.last_object_msg = data


# State ErrorService
class ErrorService(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_home', 'exit_sm'])

        # Subscriber to franka states (NOT USED NOW)
        # self.obj_sub = rospy.Subscriber(object_topic, Marker, self.obj_callback, queue_size=1)

        # Building the action client
        self.recovery_client = actionlib.SimpleActionClient(err_rec_topic, ErrorRecoveryAction)

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I will perform the error recovery now.")

        # Waiting for the error recovery server and sending goal
        if not self.recovery_client.wait_for_server(rospy.Duration.from_sec(5.0)):
            rospy.logerr("In ErrorService, could not contact recovery server: exiting!")
            return 'exit_sm'
        
        goal = ErrorRecoveryActionGoal()
        self.recovery_client.send_goal(goal)
        
        # Checking the result and changing state
        if self.recovery_client.wait_for_result(rospy.Duration.from_sec(5.0)):
            rospy.loginfo("In ErrorService, no problems: recovered, going to home!")
            return 'go_to_home'
        else:
            rospy.logerr("In ErrorService, errors: couldn't recover, exiting!")
            return 'exit_sm'
            

# State GraspService
class GraspService(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_check', 'error_grasp', 'go_to_prepare'],
                                            input_keys=['grasp_in'])

        # Service Proxy to grasp service
        self.complex_grasp_client = rospy.ServiceProxy(grasp_service_name, complex_grasp)

        # Subscriber to object pose and id state and the saved message
        self.obj_sub = rospy.Subscriber(object_topic, IdentifiedCluster, self.obj_callback, queue_size=1)
        self.last_object_msg = None

        # Subscriber to aruco markers and the saved message
        self.obj_sub = rospy.Subscriber(identified_clusters_topic, IdentifiedClustersArray, self.callback_clusters, queue_size=1)
        self.last_all_clusters_msg = None

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I will perform the grasp now.")

        # Check if the passed userdata obj_id from PrepareGrasp is in the clusters array
        cluster_found = None
        for ind in range(len(self.last_all_clusters_msg.ident_clusters)):
            if self.last_all_clusters_msg.ident_clusters[ind].obj_id == userdata.grasp_in.obj_id:
                cluster_found = self.last_all_clusters_msg.ident_clusters[ind]

        if cluster_found is None:
            rospy.loginfo("In GraspService, the object with obj_id given by PrepareGrasp was not found!")
            return 'go_to_prepare'

        if VERBOSE:
            print("I got the following marker obj_id from PrepareGrasp: ")
            print(userdata.grasp_in.obj_id)
            print("The obj_id_found is: ")
            print(cluster_found.obj_id)

        # Creating a service request and sending
        complex_grasp_req = complex_graspRequest()
        complex_grasp_req.data = True
        complex_grasp_req.object_pose = cluster_found.pose
        complex_grasp_res = complex_graspResponse()

        if VERBOSE:
            print("The computed pose of grasp is")
            print(complex_grasp_req.object_pose)

        try:
            complex_grasp_res = self.complex_grasp_client(complex_grasp_req)
        except rospy.ServiceException, e:
            print "In GraspService, Service call failed: %s"%e

        # Changing states according to res
        if complex_grasp_res.success:
            rospy.loginfo("In GraspService, no problems: going to check!")
            self.last_object_msg = None         # Clearing the memory of grasp
            return 'go_to_check'
        else:
            rospy.loginfo("In GraspService, errors: going to error state!")
            self.last_object_msg = None         # Clearing the memory of grasp
            return 'error_grasp'

    def obj_callback(self, data):
        self.last_object_msg = data
    
    def callback_clusters(self, data):
        self.last_all_clusters_msg = data


# State CheckGrasp
class CheckGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_place', 'try_regrasp', 'go_to_home'],
                             input_keys=['check_grasp_in'])

        # Subscriber to object pose and the saved message
        self.obj_sub = rospy.Subscriber(object_topic, IdentifiedCluster, self.obj_callback, queue_size=1)
        self.last_object_msg = None

        # Subscriber to aruco markers and the saved message
        self.obj_sub = rospy.Subscriber(identified_clusters_topic, IdentifiedClustersArray, self.callback_clusters, queue_size=1)
        self.last_all_clusters_msg = None

        # Service Proxy to hand opening service
        self.hand_plan_client = rospy.ServiceProxy(hand_plan_service_name, hand_plan)
        self.hand_control_client = rospy.ServiceProxy(hand_control_service_name, hand_control)

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I'm checking if I really grasped.")

        # Check if the passed userdata obj_id and rough position from PrepareGrasp is in the clusters array
        same_obj_found = False
        for ind in range(len(self.last_all_clusters_msg.ident_clusters)):
            if self.last_all_clusters_msg.ident_clusters[ind].obj_id == userdata.check_grasp_in.obj_id:
                # Check for position with a tolerance
                tmp1 = self.last_all_clusters_msg.ident_clusters[ind].pose.position
                tmp2 = userdata.check_grasp_in.pose.position
                if (math.sqrt((tmp1.x - tmp2.x)**2 + (tmp1.y - tmp2.y)**2 + (tmp1.z - tmp2.z)**2) < epsilon):
                    same_obj_found = True

        print("I got the following marker obj_id from PrepareGrasp: ")
        print(userdata.check_grasp_in.obj_id)
        print("The same_obj_found is: ")
        print(same_obj_found)


        # The userdata given by PrepareGrasp used to check grasp success
        if self.last_object_msg is None:
            rospy.loginfo("In CheckGrasp I found no objects in scene. Supposing grasp to be successful!")
            self.last_object_msg = None         # Clearing the memory of checkgrasp
            return 'go_to_place'
        elif not same_obj_found:
            rospy.loginfo("In CheckGrasp I did not find the same object in scene. Supposing grasp to be successful!")
            self.last_object_msg = None         # Clearing the memory of checkgrasp
            return 'go_to_place'
        else:
            rospy.loginfo("In CheckGrasp I found the same object in scene. Grasp unsuccessful! Opening hand and retying...")
            hand_plan_req = hand_planRequest()
            hand_plan_req.goal_syn = 0.0
            hand_plan_req.goal_duration = 2.0
            hand_plan_res = self.hand_plan_client(hand_plan_req)

            if not hand_plan_res.answer:
                rospy.logerr("In CheckGrasp I could not plan the reopening of hand for regrasing! Going to home...")
                self.last_object_msg = None         # Clearing the memory of checkgrasp
                return 'go_to_home'

            hand_control_req = hand_controlRequest()
            hand_control_req.computed_trajectory = hand_plan_res.computed_trajectory
            hand_control_res = self.hand_control_client(hand_control_req)

            if not hand_control_res:
                rospy.logerr("In CheckGrasp I could not reopen the hand for regrasing! Going to home...")
                self.last_object_msg = None         # Clearing the memory of checkgrasp
                return 'go_to_home'

            self.last_object_msg = None         # Clearing the memory of checkgrasp
            return 'try_regrasp'

    def obj_callback(self, data):
        self.last_object_msg = data

    def callback_clusters(self, data):
        self.last_all_clusters_msg = data


# State PlaceService
class PlaceService(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_home', 'error_place'])

        # Service Proxy to place service
        self.place_client = rospy.ServiceProxy(place_service_name, SetBool)

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I will perform placing now.")

        # Creating a service request and sending
        set_bool_req = SetBoolRequest(True)

        try:
            set_bool_res = self.place_client(set_bool_req)
        except rospy.ServiceException, e:
            print "In PlaceService, Service call failed: %s"%e

        # If ok go to home, else go to error state
        if set_bool_res.success:
            rospy.loginfo("In PlaceService,everything ok: going to home!")
            return 'go_to_home'
        else:
            rospy.logerr("In PlaceService error: going to error recovery!")
            return 'error_place'

        # Anyways go to home
        


# State HomeService
class HomeService(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_wait', 'exit_sm'])

        # Service Proxy to home service
        self.home_client = rospy.ServiceProxy(home_service_name, SetBool)

    def execute(self, userdata):
        if VERBOSE:
            rospy.loginfo("I will perform homing now.")

        # Creating a service request and sending
        set_bool_req = SetBoolRequest(True)
        set_bool_res = SetBoolResponse()

        try:
            set_bool_res = self.home_client(set_bool_req)
        except rospy.ServiceException, e:
            print "In HomeService, Service call failed: %s"%e

        # If ok go to wait, else exit state machine
        if set_bool_res.success:
            rospy.loginfo("In HomeService, no problems: going to wait!")
            return 'go_to_wait'
        else:
            rospy.logerr("In HomeService, errors happened: exiting state machine!")
            return 'exit_sm'


# DEFINING THE STATE MACHINE

def main():
    rospy.init_node('planning_sm')

    # Waiting for the necessary services
    rospy.loginfo("Before starting the state machine, waiting for the services!")
    rospy.wait_for_service(set_obj_service_name)
    rospy.wait_for_service(grasp_service_name)
    rospy.wait_for_service(place_service_name)
    rospy.wait_for_service(home_service_name)

    # Creating the top level state machine
    sm_top = smach.StateMachine(outcomes=['stop_everything'])

    # Open the container
    with sm_top:
        # The entering point is Wait
        smach.StateMachine.add('WAIT', Wait(),
                               transitions={'no_obj_in_view': 'WAIT',
                                            'obj_in_view': 'PICK_AND_PLACE'})

        # Create the sub state machine which does all the planning
        sm_sub = smach.StateMachine(outcomes=['exit_pick_and_place', 'exit_all'])
        sm_sub.userdata.passed_marker_msg = None

        # Open the sub container
        with sm_sub:
            # The entering point is PrepareGrasp
            smach.StateMachine.add('PREPARE_GRASP', PrepareGrasp(),
                                   transitions={'go_to_wait': 'exit_pick_and_place',
                                                'go_to_grasp': 'GRASP_SERVICE'},
                                    remapping={'prepare_grasp_out':'passed_marker_msg'})

            # The service state for grasping
            smach.StateMachine.add('GRASP_SERVICE', GraspService(),
                                   transitions={'go_to_check': 'CHECK_GRASP',
                                                'error_grasp': 'ERROR_SERVICE',
                                                'go_to_prepare': 'PREPARE_GRASP'},
                                    remapping={'grasp_in':'passed_marker_msg'})

            # The service state for error recovery
            smach.StateMachine.add('ERROR_SERVICE', ErrorService(),
                                   transitions={'go_to_home': 'HOME_SERVICE',
                                                'exit_sm': 'exit_all'})

            # The state for checking grasp success
            smach.StateMachine.add('CHECK_GRASP', CheckGrasp(),
                                   transitions={'go_to_place': 'PLACE_SERVICE',
                                                'try_regrasp': 'GRASP_SERVICE',
                                                'go_to_home' : 'HOME_SERVICE'},
                                    remapping={'check_grasp_in':'passed_marker_msg'})

            # The state for placing the grasped object
            smach.StateMachine.add('PLACE_SERVICE', PlaceService(),
                                   transitions={'go_to_home': 'HOME_SERVICE',
                                                'error_place': 'ERROR_SERVICE'})

            # The state for going to home
            smach.StateMachine.add('HOME_SERVICE', HomeService(),
                                   transitions={'go_to_wait': 'exit_pick_and_place',
                                                'exit_sm': 'exit_all'})

        smach.StateMachine.add('PICK_AND_PLACE', sm_sub,
                               transitions={'exit_pick_and_place': 'WAIT',
                                            'exit_all': 'stop_everything'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('planning_sm', sm_top, '/SM_TOP')
    sis.start()

    # Execute SMACH plan
    sm_top.execute()

    # Wait for ctrl-c to stop the application
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down IRIM Planning Module"
        sis.stop()

# THE MAIN

if __name__ == '__main__':
    main()
