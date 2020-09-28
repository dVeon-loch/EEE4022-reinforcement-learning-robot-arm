import rospy
from arm_env import ArmEnvironment, AllJoints
import position_command
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelState
import numpy as np

zero = np.array([0,0,0,0])

testpos = np.array([0,0,0,1])


""" rospy.init_node('testing_node')

jointboi = AllJoints(['plat_joint','shoulder_joint','forearm_joint','wrist_joint'])

jointboi.move(testpos) """

# TODO fix reset function

envboi = ArmEnvironment()

envboi.reset()


#position_command.pos_commander(testpos)


