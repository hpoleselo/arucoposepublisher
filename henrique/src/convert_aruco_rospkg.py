#!/usr/bin/env python
from math import pi
import numpy as np
import rospy
import cv2
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point, TransformStamped
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
import tf2_ros



class PoseProcessor(object):

    def __init__(self):
        rospy.init_node("MarkerPoseHandler")
        self.sub = rospy.Subscriber("aruco_single/pose", PoseStamped, self.broadcaster)
        self.pub = rospy.Publisher("GraspingPose", PoseStamped, queue_size=1)
        self.rate = rospy.Rate(1)

        # Setting default Header for the message that will be published
        self.hdr = Header()
        self.hdr.frame_id = "world"
        self.hdr.stamp = rospy.Time.now()

        # Creates the broadcaster
        self.brd = tf2_ros.TransformBroadcaster()

        # Creates the buffer/listener to check the transformation
        self.buff = tf2_ros.Buffer(rospy.Duration(5))
        self.listener = tf2_ros.TransformListener(self.buff)

        # Fixed rotation to overwrite the detected rotation
        fixed_rot = [pi, 0, 3.84]
        fixed_q = quaternion_from_euler(fixed_rot[0], fixed_rot[1], fixed_rot[2])
        self.quat = Quaternion(fixed_q[0], fixed_q[1], fixed_q[2], fixed_q[3])


    def broadcaster(self, msg):
        """ Broadcaster to create the tf relationship between marker -> camera."""
        t = TransformStamped()

        # Pack the transformation in one message (transformStamped, new from tf2.)
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera"
        t.child_frame_id = "marker"
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w
        self.brd.sendTransform(t)
        self.check_and_send_pose()

    def check_and_send_pose(self):
        """ Extracts the pose from the transformation Camera -> Marker."""
   
        try:
            transform = self.buff.lookup_transform("world", "marker", rospy.Time(0), rospy.Duration(4.0))
            ps = PoseStamped(self.hdr, Pose(transform.transform.translation, transform.transform.rotation))
            # If we wanted to send a fixed rotation:
            #ps = PoseStamped(hdr, Pose(transform.transform.translation, self.quat))
            self.pub.publish(ps)
        except ():
            print ("Could not transform.")
            self.marker_sub.unregister()


if __name__ == '__main__':
    try:
        pc = PoseProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        print ('Program could not be correctly ran.')
