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

robot_id = 25
rob_marker = [0, 0, 0, 0 ,0, 0]

input_topic = "aruco_marker_publisher/markers"
output_ns = "irim_demo/"
output_pose_topic = output_ns + "pose_chosen_object"
output_aruco_topic = output_ns + "aruco_chosen_object"


class ObjectPoseRemapper:

    def __init__(self):
        # Initializing the subscribers and publishers

        # Subscriber
        self.sub = rospy.Subscriber(input_topic, MarkerArray, self.callback, queue_size=1)

        # Publishers
        self.pose_pub = rospy.Publisher(output_pose_topic, Pose, queue_size=10)  # For task_sequencer (testing)
        self.aruco_pub = rospy.Publisher(output_aruco_topic, Pose, queue_size=10)  # For state machine

        if VERBOSE:
            print "Subscribed to " + input_topic
            print "Publishing to " + output_pose_topic
            print "Publishing to " + output_aruco_topic
            print "Broadcasting camera and object tfs"

        # Computing the fixed world to robot_marker transform
        rob_marker_tra_w = PyKDL.Vector(rob_marker[0], rob_marker[1], rob_marker[2])
        rob_marker_rot_w = PyKDL.Rotation()
        rob_marker_rot_w.DoRotX(rob_marker[3])
        rob_marker_rot_w.DoRotY(rob_marker[4])
        rob_marker_rot_w.DoRotZ(rob_marker[5])
        rob_maker_frame_w = PyKDL.Frame(rob_marker_rot_w, rob_marker_tra_w)

    def callback(self, data):
        # Callback function of subscribed topic. Here the read message of pose is published to output topic

        if VERBOSE:
            rospy.loginfo(rospy.get_caller_id() + "I heard a new MarkerArray")

        # Anyways extract the poses of robot marker and publish camera tf
        self.compute_camera_frame(data)

        # lowest_ind = 0
        # for ind in data.markers:
        #     if data.markers[ind].id != robot_id and data.markers[ind].id <= lowest_ind:
        #         lowest_ind = data.markers[ind].id
        #         object_marker_pose = data.markers[ind].pose.pose
        #     elif data.markers[ind].id == robot_id:
        #         robot_marker_pose = data.markers[ind].pose.pose
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

    def compute_camera_frame(self, data):
        # Simple function which computes the camera frame from object pose in camera

        rob_marker_pose_c = data.markers[0].pose.pose       # safety assignment
        for ind in range(len(data.markers)):
            if data.markers[ind].id == robot_id:
                rob_marker_pose_c = data.markers[ind].pose.pose     # wrt camera

        # Transform to PyKDL Frame
        rob_marker_tra_c = PyKDL.Vector(rob_marker_pose_c.position.x, rob_marker_pose_c.position.y,
                                        rob_marker_pose_c.position.z)
        rob_marker_rot_c = PyKDL.Rotation()
        rob_marker_rot_c.Quaternion(rob_marker_pose_c.orientation.x, rob_marker_pose_c.orientation.y,
                                    rob_marker_pose_c.orientation.z, rob_marker_pose_c.orientation.w)
        rob_maker_frame_c = PyKDL.Frame(rob_marker_rot_c, rob_marker_tra_c)

    def broadcast_tf(self, pose):
        # Simple function which broadcasts a tf from a pose
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
