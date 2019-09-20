#!/usr/bin/env python

"""
    This node does several important tasks:
    a) Publishes the tf of the camera wrt world (this will be used for object pose computation
    b) Chooses the object to be grasped and publishes the following
        i) an aruco message which will be used by the state machine
        ii) the tf in order to visualize the chosen object in RViz
        iii) A pose for testing with task_sequencer of panda_softhand_control

    All the above is done with some assumptions
    - the marker with id = robot_id is considered robot marker (used each transformation computation)
    - the other markers are objects to be grasped
    - the one with smallest id will be chosen
    - the pose will be changed for easing the grasping
"""

# Python libs
import sys
import time
import numpy

# Ros libraries
import roslib
import rospy
import tf
import PyKDL

from geometry_msgs.msg import Pose
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import UInt32MultiArray

VERBOSE = False

max_id = 999
robot_id = 25
rob_marker = [0, 0, 0, 0 ,0, 0]

world_frame_name = "/world"

input_topic = "aruco_marker_publisher/markers"
output_ns = "irim_demo/"
output_pose_topic = output_ns + "pose_chosen_object"            # maybe not needed
output_aruco_topic = output_ns + "aruco_chosen_object"


class ObjectPoseRemapper:

    def __init__(self):
        # Initializing the subscribers and publishers

        # Subscriber
        self.sub = rospy.Subscriber(input_topic, MarkerArray, self.callback, queue_size=1)

        # Publishers
        self.pose_pub = rospy.Publisher(output_pose_topic, Pose, queue_size=10)  # For task_sequencer (testing)
        self.aruco_pub = rospy.Publisher(output_aruco_topic, Pose, queue_size=10)  # For state machine

        # Other instances set as None
        self.rob_maker_frame_w = None
        self.rob_maker_frame_c = None
        self.cam_frame_w = None
        self.obj_maker_frame_c = None
        self.obj_frame_w = None

        if VERBOSE:
            print "Subscribed to " + input_topic
            print "Publishing to " + output_pose_topic
            print "Publishing to " + output_aruco_topic
            print "Broadcasting camera and object tfs"

        # Computing the fixed world to robot_marker transform
        rob_marker_tra_w = PyKDL.Vector(rob_marker[0], rob_marker[1], rob_marker[2])
        rob_marker_rot_w = PyKDL.Rotation.EulerZYX(rob_marker[3], rob_marker[4], rob_marker[5])
        self.rob_maker_frame_w = PyKDL.Frame(rob_marker_rot_w, rob_marker_tra_w)

    def callback(self, data):
        # Callback function of subscribed topic. Here the read message of pose is published to output topic

        if VERBOSE:
            rospy.loginfo(rospy.get_caller_id() + "I heard a new MarkerArray")

        # Anyways extract the poses of robot marker and publish camera tf
        self.compute_camera_frame(data)
        self.broadcast_tf(self.cam_frame_w, world_frame_name)

        # Now if any, choose the object an publish the relevant stuff
        if len(data.markers) > 1:                  # hp: if there's only one marker, it's the robot marker
            self.choose_publish_object(data)
            self.broadcast_tf(self.obj_frame_w, world_frame_name)

    def compute_camera_frame(self, data):
        # Simple function which computes the camera frame from object pose in camera

        rob_marker_pose_c = data.markers[0].pose.pose       # safety assignment
        for ind in range(len(data.markers)):
            if data.markers[ind].id == robot_id:
                rob_marker_pose_c = data.markers[ind].pose.pose     # wrt camera
                break
        else:
            rospy.logwarn(rospy.get_caller_id() + "Beware: I have found no marker with robot_id... Using default!")

        # Transform to PyKDL Frame
        rob_marker_tra_c = PyKDL.Vector(rob_marker_pose_c.position.x, rob_marker_pose_c.position.y,
                                        rob_marker_pose_c.position.z)
        rob_marker_rot_c = PyKDL.Rotation.Quaternion(rob_marker_pose_c.orientation.x, rob_marker_pose_c.orientation.y,
                                    rob_marker_pose_c.orientation.z, rob_marker_pose_c.orientation.w)
        self.rob_maker_frame_c = PyKDL.Frame(rob_marker_rot_c, rob_marker_tra_c)

        # Compute the camera frame in world
        self.cam_frame_w = self.rob_maker_frame_w * self.rob_maker_frame_c.Inverse()
        if VERBOSE:
            print "The camera in world is"
            print(self.cam_frame_w)

    def choose_publish_object(self, data):
        # Simple function which chooses the object if any and publishes the aruco message to topic

        # Choose the object with the lowest marker id
        obj_marker_pose_c = None
        lowest_ind = max_id
        for ind in range(len(data.markers)):
            print("The lowest id is " + str(lowest_ind) + " and the marker id is " + str(data.markers[ind].id))
            if data.markers[ind].id != robot_id and data.markers[ind].id <= lowest_ind:
                lowest_ind = data.markers[ind].id
                obj_marker_pose_c = data.markers[ind].pose.pose

        print("The chosen marker id is " + str(lowest_ind))
        
        # Transform it to PyKDL Frame
        obj_marker_tra_c = PyKDL.Vector(obj_marker_pose_c.position.x, obj_marker_pose_c.position.y,
                                        obj_marker_pose_c.position.z)
        obj_marker_rot_c = PyKDL.Rotation.Quaternion(obj_marker_pose_c.orientation.x, obj_marker_pose_c.orientation.y,
                                                     obj_marker_pose_c.orientation.z, obj_marker_pose_c.orientation.w)
        self.obj_maker_frame_c = PyKDL.Frame(obj_marker_rot_c, obj_marker_tra_c)

        # Compute the object frame in world
        self.obj_frame_w = self.cam_frame_w * self.obj_maker_frame_c

    def broadcast_tf(self, frame, ref):
        # Simple function which broadcasts a tf from a PyKDL frame expressed wrt a ref frame (string)
        br = tf.TransformBroadcaster()


def main():
    # Initializes ros node and creates object
    rospy.init_node('object_pose_remapper', anonymous=True)
    opr = ObjectPoseRemapper()

    # Start to spin
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down ROS Image feature detector module"


if __name__ == '__main__':
    main()


    #
    # # Compute the needed poses
    #
    # # Publish the camera tf always but the object stuff will be published only if there is more than one marker
    # # In fact, if there is only one marker, that will be the robot marker
    #
    # if len(data.markers) == 1 and data.markers[0].id == robot_id:
    #     print "No objects to be grasped! Only robot!"
    # else:
    #     obj_pose = data.markers[1].pose.pose
    #
    #     # Filling up the new message
    #     pose_stamped = Pose()
    #     pose_stamped.header.stamp = rospy.Time.now()
    #     pose_stamped.pose = obj_pose
    #
    #     # Publish to output_topic
    #     self.pub.publish(pose_stamped)