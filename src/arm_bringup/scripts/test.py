from arm_env import AllJoints
import rospy

rospy.init_node('test_node')
joint_names = ['plat_joint','shoulder_joint','forearm_joint','wrist_joint']
jt = AllJoints(joint_names)
# TODO test joint ranges
zero = [0,0,0,0]

limit_pos = [1.4,1.4,1.4,2.4]
limit_neg = [-1.4,-1.4,-1.4,-2.4]

jt.move(zero)
next = raw_input("Press enter to move... ")
jt.move(limit_pos)
next = raw_input("Press enter to move... ")
jt.move(limit_neg)

