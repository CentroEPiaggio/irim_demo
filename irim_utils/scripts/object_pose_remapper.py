#!/usr/bin/env python
# license removed for brevity

# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.

''' This node is for republishing an objects pose to an output_topic '''

# Python libs
import sys, time
import numpy

# Ros libraries
import roslib
import rospy

from geometry_msgs.msg import Pose
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import UInt32MultiArray

VERBOSE=False
input_topic = "aruco_marker_publisher/markers"
output_topic = "object_pose"

class object_pose_remapper:

    def __init__(self):
        ''' Initializing the subscriber to aruco 
        and the publisher to object_pose '''

        # Subsciber
        self.sub = rospy.Subscriber(input_topic, MarkerArray, self.callback, queue_size = 1)

        # Publisher
        self.pub = rospy.Publisher(output_topic, Pose, queue_size=10)

        if VERBOSE :
            print "subscribed to " + input_topic
            print "subscribed to " + output_topic

    def callback(self, data):
        ''' Callback function of subscribed topic. 
        Here the read message of pose is published to output topic '''
        
        # Extractind the needed data to be published
        rospy.loginfo(rospy.get_caller_id() + "I heard the MarkerArray")
        obj_pose = data.markers[0].pose.pose

        # Publish to output_topic
        self.pub.publish(obj_pose)

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('object_pose_remapper', anonymous=True)

    opr = object_pose_remapper()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)
