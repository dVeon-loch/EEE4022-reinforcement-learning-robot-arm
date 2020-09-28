import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelState
import numpy as np

class AllJoints:
    def __init__(self,joint_names):
        self.jta = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action')
        self.jtp = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)
        self.joint_names = joint_names
        self.jtp_zeros = np.zeros(len(joint_names))

    def move(self, pos):
        msg = FollowJointTrajectoryActionGoal()
        msg.goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = pos
        point.time_from_start = rospy.Duration(1.0/60.0)
        msg.goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(msg.goal)

    def move_jtp(self, pos):
        jtp_msg = JointTrajectory()
        jtp_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = pos
        point.velocities = self.jtp_zeros
        point.accelerations = self.jtp_zeros
        point.effort = self.jtp_zeros
        point.time_from_start = rospy.Duration(1.0/60.0)
        jtp_msg.points.append(point)
        self.jtp.publish(jtp_msg)

    def reset_move(self, pos):
        jtp_msg = JointTrajectory()
        self.jtp.publish(jtp_msg)
        msg = FollowJointTrajectoryActionGoal()
        msg.goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = pos
        point.time_from_start = rospy.Duration(0.0001)
        msg.goal.trajectory.points.append(point)
        self.jta.send_goal(msg.goal)

    def reset_move_jtp(self, pos):
        jtp_msg = JointTrajectory()
        self.jtp.publish(jtp_msg)
        jtp_msg = JointTrajectory()
        jtp_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = pos
        point.velocities = self.jtp_zeros
        point.accelerations = self.jtp_zeros
        point.effort = self.jtp_zeros
        point.time_from_start = rospy.Duration(0.0001)
        jtp_msg.points.append(point)
        self.jtp.publish(jtp_msg)

class ArmEnvironment:
    def __init__(self):
        rospy.init_node('joint_position_node')
        self.num_joints = 4
        self.state_shape = self.num_joints
        self.action_shape = self.num_joints
        self.joint_names = ['plat_joint','shoulder_joint','forearm_joint','wrist_joint']
        self.all_joints = AllJoints(self.joint_names)
        self.joint_pos_high = np.array([1.5, 1.5, 1.5, 2.5])
        self.joint_pos_low = np.array([-1.5, -1.5, -1.5, -2.5])
        self.joint_pos_range = self.joint_pos_high-self.joint_pos_low
        self.joint_pos_mid = self.joint_pos_range/2.0
        self.joint_pos = self.joint_pos_mid
        self.joint_state = np.zeros(1,self.num_joints)
        self.joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_subscriber_callback)
        