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

    Autors: George Jose Pollayil, Mathew Jose Pollayil
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
import tf_conversions.posemath as pm

from geometry_msgs.msg import Pose
from aruco_msgs.msg import MarkerArray
from irim_vision.msg import IdentifiedClustersArray
from irim_vision.msg import IdentifiedCluster
from std_msgs.msg import UInt32MultiArray

VERBOSE = False
DEBUG = False


max_id = 999                # If you change this, change it also in planning sm
robot_id = 0
# rob_marker = [0.17, 0, -0.025, -1.5707, -3.1415, 0]
# rob_marker = [0, 0, 0, 0, 0, 0]
rob_marker = [0.165, -0.285, -0.03, 1.5707, 0, 3.1415]
# cam_rgb_2_link = [0.015, -0.000, -0.000, -0.499, 0.502, -0.500, -0.499]        # For Intel Realsense
cam_rgb_2_link = [-0.045, 0.000, 0.000, 0.500, -0.500, 0.500, 0.500]      # For Asus Xtion

world_frame_name = "world"
object_frame_name = "object"
camera_frame_name = "camera_link"

input_topic = "aruco_marker_publisher/markers"
input_topic_pcl = "irim_vision/identified_clusters"
output_ns = "irim_demo/"
output_pose_topic = output_ns + "chosen_object"                 # This will be used by task_sequencer if simple grasp
output_cluster_topic = output_ns + "cluster_chosen_object"      # This will be used by state machine
world_to_cam_topic = output_ns + "camera_pose"                  # This will be used by state machine


class ObjectPoseRemapper:

    def __init__(self):
        # Initializing the subscribers and publishers

        # Subscriber
        self.sub = rospy.Subscriber(input_topic, MarkerArray, self.callback, queue_size=1)
        self.sub_clusters = rospy.Subscriber(input_topic_pcl, IdentifiedClustersArray, self.callback_clusters, queue_size=1)

        # Publishers
        self.cluster_pub = rospy.Publisher(output_cluster_topic, IdentifiedCluster, queue_size=10)      # For state machine
        self.pose_pub = rospy.Publisher(output_pose_topic, Pose, queue_size=10)                         # For task_sequencer
        self.cam_pose_pub = rospy.Publisher(world_to_cam_topic, Pose, queue_size=10)                    # For task_sequencer

        # tf Transform Broadcaster
        self.br = tf.TransformBroadcaster()

        # Other instances set as None
        self.rob_maker_frame_w = None
        self.rob_maker_frame_c = None
        self.cam_link_frame_w = None
        self.cam_rgb_frame_w = None
        self.cam_link_frame_rgb = None
        self.obj_frame_w = None
        self.chosen_obj_cluster = None

        if VERBOSE:
            print("Subscribed to " + input_topic)
            print("Subscribed to " + input_topic_pcl)
            print("Publishing to " + output_pose_topic)
            print("Publishing to " + output_cluster_topic)
            print("Broadcasting camera and object tfs")

        # Computing the fixed world to robot_marker transform
        rob_marker_tra_w = PyKDL.Vector(rob_marker[0], rob_marker[1], rob_marker[2])
        rob_marker_rot_w = PyKDL.Rotation.EulerZYX(rob_marker[5], rob_marker[4], rob_marker[3])
        self.rob_maker_frame_w = PyKDL.Frame(rob_marker_rot_w, rob_marker_tra_w)

        # Computing the fixed camera_rgb_optical_frame to camera_link transform
        cam_link_tra_rgb = PyKDL.Vector(cam_rgb_2_link[0], cam_rgb_2_link[1], cam_rgb_2_link[2])
        cam_link_rot_rgb = PyKDL.Rotation.Quaternion(cam_rgb_2_link[3], cam_rgb_2_link[4], cam_rgb_2_link[5],
                                                     cam_rgb_2_link[6])
        self.cam_link_frame_rgb = PyKDL.Frame(cam_link_rot_rgb, cam_link_tra_rgb)

    def callback(self, data):
        # Callback function of aruco subscribed topic. Here the cam transform is published

        if VERBOSE:
            rospy.loginfo(rospy.get_caller_id() + "I heard a new MarkerArray")

        # Anyways extract the poses of robot marker and publish camera tf
        self.compute_camera_frame(data)
        self.broadcast_tf(self.cam_link_frame_w, camera_frame_name, world_frame_name)

        # For debug purposes republish robot marker frame
        if DEBUG:
            self.broadcast_tf(self.rob_maker_frame_c, 'marker_debug', 'camera_rgb_optical_frame')

        # Now publish
        self.cam_pose_pub.publish(pm.toMsg(self.cam_rgb_frame_w))

    def callback_clusters(self, data):
        # Callback function of clusters subscribed topic. Here the object transform is published

        # Now if any, choose the object an publish the relevant stuff
        if len(data.ident_clusters) > 0:
            self.choose_object(data)
            self.broadcast_tf(self.obj_frame_w, object_frame_name, world_frame_name)
        else:
            # When no object is in scene publishing max id + 1 is required unless we
            # find a method for find callbacks not being called
            self.chosen_obj_cluster = IdentifiedCluster()
            self.chosen_obj_cluster.obj_id = max_id + 1

        # Publishing the object Identified Cluster message to the output_cluster_topic 
        # This publishing is important for wait in the state machine
        if self.chosen_obj_cluster is not None:            
            # Now publish
            self.cluster_pub.publish(self.chosen_obj_cluster)
            self.pose_pub.publish(self.chosen_obj_cluster.pose)

    def compute_camera_frame(self, data):
        # Simple function which computes the camera frame from object pose in camera

        rob_marker_pose_c = data.markers[0].pose.pose  # safety assignment
        for ind in range(len(data.markers)):
            if data.markers[ind].id == robot_id:
                rob_marker_pose_c = data.markers[ind].pose.pose  # wrt camera
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
        self.cam_rgb_frame_w = self.rob_maker_frame_w * self.rob_maker_frame_c.Inverse()
        self.cam_link_frame_w = self.cam_rgb_frame_w * self.cam_link_frame_rgb

        if DEBUG:
            print("rob_maker_frame_c is")
            print(self.rob_maker_frame_c)
            print("rob_maker_frame_w is")
            print(self.rob_maker_frame_w)

            print("cam_rgb_frame_w is")
            print(self.cam_rgb_frame_w)
            print("cam_link_frame_w is")
            print(self.cam_link_frame_w)

    def choose_object(self, data):
        # Simple function which chooses the object if any

        # Choose the object which is nearest
        nearest_object = None
        lowest_ind = max_id
        lowest_dist = 10        # never gonna be higher than 10m

        if DEBUG:
            print("The length of the ident_cluster is " + str(len(data.ident_clusters)))
        
        for ind in range(len(data.ident_clusters)):
            if data.ident_clusters[ind].pose.position.x < lowest_dist:
                lowest_dist = data.ident_clusters[ind].pose.position.x
                lowest_ind = ind

        if lowest_ind != max_id:
            nearest_object = data.ident_clusters[lowest_ind]

        if DEBUG:
            print("The chosen objects id is " + str(lowest_ind))

        # IF NEEDED FORCE THE Z HERE

        # Compute the object frame in world
        self.obj_frame_w = pm.fromMsg(nearest_object.pose)

        # Correct the cluster msg to be in world frame
        self.chosen_obj_cluster = nearest_object
        

    def broadcast_tf(self, frame, name, ref):
        # Simple function which broadcasts a tf from a PyKDL frame expressed wrt a ref frame (string)
        self.br.sendTransform((frame.p.x(), frame.p.y(), frame.p.z()), frame.M.GetQuaternion(),
                         rospy.Time.now(), name, ref)


def main():
    # Initializes ros node and creates object
    rospy.init_node('object_pose_remapper', anonymous=True)
    ObjectPoseRemapper()

    # Start to spin
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down IRIM Image feature detection module")


if __name__ == '__main__':
    main()
