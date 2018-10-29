import rospy
from std_msgs.msg import String

# acho que entendi:
# vc define um node e o node que vc definiu vai ter um nome
# e dps quando vc definir um topico entre seu node o topico vai ter o nome
# que vc deu pro seu node (nome e caracteristica do seu topico)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
