#!/usr/bin/env python
# license removed for brevity

# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.

import rospy
from std_msgs.msg import String

global obj_pose

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard the MarkerArray")
    obj_pose = data.pose.pose

def object_pose_remapper():
    rospy.init_node('object_pose_remapper', anonymous=True)
    rospy.Subscriber("aruco_marker_publisher/markers_list", UInt32MultiArray, callback)
    pub = rospy.Publisher('object_pose', Pose, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(obj_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        object_pose_remapper()
    except rospy.ROSInterruptException:
        pass
