import rospy
import tf
from geometry_msgs.msg import PoseStamped

def handleMessage(data):
    ps = PoseStamped()
    ps = data
    transformation(ps)

def transformation(ps):
    # The Method sendTransform requires the translation and rotation vectors as tuples, so we have to convert it
    x = ps.pose.position.x
    y = ps.pose.position.y
    z = ps.pose.position.z
    translation = x, y, z

    rx = ps.pose.orientation.x
    ry = ps.pose.orientation.y
    rz = ps.pose.orientation.z
    rw = ps.pose.orientation.w
    rotation = rx, ry, rz, rw
    broadcaster = tf.TransformBroadcaster()
    broadcaster.sendTransform(translation, rotation, rospy.Time.now(), 'marker', 'camera')

def rossetup():
    rospy.init_node('TfBroadcaster')
    sub = rospy.Subscriber('MarkerPose', PoseStamped, handleMessage)
    rospy.spin()

if __name__ == '__main__':
    rossetup()
