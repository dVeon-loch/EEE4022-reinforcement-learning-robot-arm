import rospy
from control_msgs.msg import JointTrajectoryControllerState

def callback(data):
    rospy.loginfo("The current state is: %s",data.actual.positions)

def listener():
    rospy.init_node("Mossie_is_Gay")
    state_sub = rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, callback)
    rospy.sleep(2)
    rospy.spin()

if __name__ == "__main__":
    listener()