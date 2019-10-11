#!/usr/bin/env python

""" This node chooses the object to be grasped:
    - the marker with id = 0 is considered robot marker
    - the other markers are objects to be grasped
    - the one with smallest id will be chosen
    - the pose will be changed for easing the grasping 
    
    Autors: George Jose Pollayil, Mathew Jose Pollayil
    
"""

# Python libs
import sys, time
import numpy

# Ros libraries
import roslib
import rospy

from geometry_msgs.msg import PoseStamped
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import UInt32MultiArray

VERBOSE=False
input_topic = "aruco_marker_publisher/markers"
output_topic = "irim_demo/chosen_object"
robot_id = 25

class object_pose_remapper:

    def __init__(self):
        ''' Initializing the subscriber to aruco 
        and the publisher to object_pose '''

        # Subscriber
        self.sub = rospy.Subscriber(input_topic, MarkerArray, self.callback, queue_size = 1)

        # Publisher
        self.pub = rospy.Publisher(output_topic, PoseStamped, queue_size=10)

        if VERBOSE :
            print "subscribed to " + input_topic
            print "subscribed to " + output_topic

    def callback(self, data):
        ''' Callback function of subscribed topic. 
        Here the read message of pose is published to output topic '''
        
        rospy.loginfo(rospy.get_caller_id() + "I heard the MarkerArray")

        # Checking if there is only the robot marker: if so don't publish anything
        if len(data.markers) == 1 and data.markers[0].id == robot_id:
            print "No objects to be grasped! Only robot!"
        else:
            obj_pose = data.markers[1].pose.pose
            
            # Filling up the new message
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = obj_pose

            # Publish to output_topic
            self.pub.publish(pose_stamped)

def main(args):
    '''I nitializes and cleanup ros node '''
    rospy.init_node('object_pose_remapper', anonymous=True)

    opr = object_pose_remapper()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)
