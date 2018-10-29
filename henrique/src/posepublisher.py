#!/usr/bin/env python
import rospy
import std_msgs.msg
import arucointerface
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from tf.transformations import quaternion_from_euler

def convert_to_quaternion(roll, pitch, yaw):
    _quaternion = quaternion_from_euler(roll, pitch, yaw)
    real_quaternion = Quaternion(_quaternion[0],_quaternion[1],_quaternion[2],_quaternion[3])
    print 'Quaternion: ', _quaternion
    return real_quaternion

def pose_publisher():
    rospy.init_node('MarkerPose')
    rate = rospy.Rate(15)
    pub = rospy.Publisher('MarkerPose', PoseStamped, queue_size=15)
    while not rospy.is_shutdown():
        aruco = arucointerface.ArucoInterface()
        _rvec, _tvec = aruco.trackAruco()

        list1 = []
        list2 = []
        for i in range(3):
            list1.append(_rvec[0, 0, i])
            list2.append(_tvec[0, 0, i])

        roll = list1[0]
        pitch = list1[1]
        yaw = list1[2]
        quaternion = convert_to_quaternion(roll, pitch, yaw)

        point = Point(list2[0], list2[1], list2[2])

        # We don't have to set the sequence field, it is set when we publish a message containing the Header.
        hdr = std_msgs.msg.Header()
        hdr.stamp = rospy.Time.now()
        hdr.frame_id = 'camera'
        posestamped = PoseStamped(hdr, Pose(point, quaternion))
        pub.publish(posestamped)
        rospy.loginfo(posestamped)
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass
