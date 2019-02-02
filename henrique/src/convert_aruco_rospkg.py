#!/usr/bin/env python
from math import pi
import numpy as np
import rospy
import cv2
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point, TransformStamped
from std_msgs.msg import Header, Bool
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
import tf2_ros

## For later, JTs advice ##
#import scipy.signal.fir_filter_design
#import scipy.signal
#scipy.signal.convolve()


class PoseProcessor(object):

    def __init__(self):
        rospy.init_node("MarkerPoseHandler")
        self.sub = rospy.Subscriber("aruco_single/pose", PoseStamped, self.broadcaster)
        self.pub = rospy.Publisher("GraspingPose", PoseStamped, queue_size=1)
        self.rate = rospy.Rate(1)

        self.ps = PoseStamped()
        # Setting default Header for all messages
        self.hdr = Header()
        self.hdr.frame_id = "marker"
        self.hdr.stamp = rospy.Time.now()

        # Creates the broadcaster
        self.brd = tf2_ros.TransformBroadcaster()

        # Creates the buffer/listener to check the transformation
        self.buff = tf2_ros.Buffer(rospy.Duration(5))
        self.listener = tf2_ros.TransformListener(self.buff)

        # Quaternion to rotate 180 degrees
        fixed_rot = [pi, 0, 3.84]
        fixed_q = quaternion_from_euler(fixed_rot[0], fixed_rot[1], fixed_rot[2])
        self.quat = Quaternion(fixed_q[0], fixed_q[1], fixed_q[2], fixed_q[3])

        # Offset
        self.offset_x = 0.00
        self.offset_y = 0.00

    def broadcaster(self, ps):
        """ Broadcaster to create the tf relationship between marker -> camera."""
        t = TransformStamped()

        # Pack the transformation in one message
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera"
        t.child_frame_id = "marker"
        t.transform.translation.x = ps.pose.position.x
        t.transform.translation.y = ps.pose.position.y
        t.transform.translation.z = ps.pose.position.z
        t.transform.rotation.x = ps.pose.orientation.x
        t.transform.rotation.y = ps.pose.orientation.y
        t.transform.rotation.z = ps.pose.orientation.z
        t.transform.rotation.w = ps.pose.orientation.w
        self.brd.sendTransform(t)
        self.check_and_send_pose()

    def check_and_send_pose(self):
        """ Extracts the pose from the transformation Camera -> Marker and then check if """
   
        try:
            transform = self.buff.lookup_transform("world", "marker", rospy.Time(0), rospy.Duration(4.0))
            hdr = Header()
            hdr.frame_id = "world"
            hdr.stamp = rospy.Time.now()
            if transform.transform.translation.x <= -0.75:
                # implementa uma funcao q pegue o maximo
                print ("Correct ziel"), transform.transform.translation.x
                transform.transform.translation.x -= self.offset_x
                transform.transform.translation.y += self.offset_y
                ps = PoseStamped(hdr, Pose(transform.transform.translation, self.quat))
                self.pub.publish(ps)
                print ("X sent:"), transform.transform.translation.x
            else:
                print ("Try to place the cube further on the X-Axis, the robot cannot reach this position.")
                count = count + 1
                #print count
        except ():
            print ("Could not transform.")
            self.marker_sub.unregister()


if __name__ == '__main__':
    try:
        pc = PoseProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        print ('Program could not be correctly ran.')
