import rospy
from geometry_msgs.msg import PoseStamped

''' This function just retrieves the data from the publisher'''
def callback(data):
    rospy.loginfo("Pose received: %s", data)


def move_robot():
    rospy.init_node('PoseReceived',)
    sub = rospy.Subscriber('MarkerPose', PoseStamped, callback)


    
    #<function callback at 0x7f964a3fd410>
    print "------- vamo --------"
    oi = callback(data)
    print oi
    print "------- acabou ---------"

    


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    move_robot()
