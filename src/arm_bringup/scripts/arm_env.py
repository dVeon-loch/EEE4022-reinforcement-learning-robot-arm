import os
import rospy
import actionlib
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, LoadController, LoadControllerRequest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel, SpawnModelRequest

import numpy as np
import time

goal_model_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..', '..', 'arm_description',
                              'urdf', 'arm_gazebo.urdf')


class AllJoints:
    def __init__(self,joint_names):
        self.jta = actionlib.SimpleActionClient('arm/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action')
        self.jtp = rospy.Publisher('arm/arm_controller/command', JointTrajectory, queue_size=1)
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
        point.time_from_start = rospy.Duration(0.0001)
        jtp_msg.points.append(point)
        self.jtp.publish(jtp_msg)

class ArmEnvironment:
    def __init__(self):
        self.zero = np.array([0,0,0,0])
        rospy.init_node('joint_position_node')
        self.num_joints = 4
        self.state_shape = self.num_joints
        self.action_shape = self.num_joints
        self.joint_names = ['plat_joint','shoulder_joint','forearm_joint','wrist_joint']
        self.all_joints = AllJoints(self.joint_names)
        self.starting_pos = np.array([0, 0, 0, 0])

        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        # self.model_config_proxy = rospy.ServiceProxy('/gazebo/set_model_configuration',SetModelConfiguration)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        # self.model_config_req = SetModelConfigurationRequest()
        # self.model_config_req.model_name = 'arm'
        # self.model_config_req.urdf_param_name = 'robot_description'
        # self.model_config_req.joint_names = self.joint_names
        # self.model_config_req.joint_positions = self.zero
        
        self.load_controller_proxy = rospy.ServiceProxy('/arm/controller_manager/load_controller', LoadController)
        self.joint_state_controller_load = LoadControllerRequest()
        self.joint_state_controller_load.name = 'joint_state_controller'
        self.arm_controller_load = LoadControllerRequest()
        self.arm_controller_load.name = 'arm_controller'

        self.switch_controller_proxy = rospy.ServiceProxy('/arm/controller_manager/switch_controller', SwitchController)
        self.switch_controller = SwitchControllerRequest()
        self.switch_controller.start_controllers.append('joint_state_controller')
        self.switch_controller.start_controllers.append('arm_controller')
        self.switch_controller.strictness = 2

        self.del_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        self.goal_urdf = open(goal_model_dir, "r").read()
        self.model = SpawnModelRequest()
        self.model.model_name = 'arm'  # the same with sdf name
        self.model.model_xml = self.goal_urdf
        self.model.robot_namespace = 'arm'
        self.initial_pose = Pose()
        self.initial_pose.position.z = 0.0305
        self.model.initial_pose = self.initial_pose 
        self.model.reference_frame = 'world'


        self.joint_pos_high = np.array([1.5, 1.5, 1.5, 2.5])
        self.joint_pos_low = np.array([-1.5, -1.5, -1.5, -2.5])
        self.joint_pos_range = self.joint_pos_high-self.joint_pos_low
        self.joint_pos_mid = self.joint_pos_range/2.0
        self.joint_pos = self.joint_pos_mid
        self.joint_state = np.zeros(self.num_joints)
        self.joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_subscriber_callback)
        self.normed_sp = self.normalize_joint_state(self.starting_pos)
        self.state = np.zeros(self.state_shape)
        self.diff_state_coeff = 4.0 # TODO find out what this is
        self.reward_coeff = 20.0 # TODO find out what this is
        self.reward = 0.0
        self.done = False
        self.episode_start_time = 0.0
        self.max_sim_time = 15.0

    
    def normalize_joint_state(self, joint_pos):
        # TODO implement normalization
        return joint_pos
    
    def joint_state_subscriber_callback(self, joint_state):
        self.joint_state = np.array(joint_state.position)

    def pause_physics(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_proxy()
            return True
        except rospy.ServiceException, e:
            print('/gazebo/pause_physics service call failed')
            return False
    def unpause_physics(self):
        print("in unpause physics")
        rospy.wait_for_service('/gazebo/unpause_physics')
        print("done waiting for service")
        try:
            self.unpause_proxy()
            print("physics unpaused")
            return True

        except rospy.ServiceException, e:
            print('/gazebo/unpause_physics service call failed')
            return False
    def reset_position(self):
        rospy.wait_for_service('/gazebo/set_model_configuration')
        try:
            self.model_config_proxy(self.model_config_req)
            return True
        except rospy.ServiceException, e:
            print('/gazebo/set_model_configuration call failed')
            return(False)
            
    def reset(self):

        rospy.wait_for_service('/gazebo/delete_model')
        self.del_model('arm')

        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            self.spawn_model(self.model.model_name, self.model.model_xml, self.model.robot_namespace, self.model.initial_pose, 'world')
            #self.spawn_model(self.sp)
        except (rospy.ServiceException) as e:
            print("/gazebo/failed to build the target")
        
        rospy.wait_for_service('arm/controller_manager/load_controller')
        try:
            self.load_controller_proxy(self.joint_state_controller_load)
        except (rospy.ServiceException) as e:
            print('arm/controller_manager/load_controller service call failed')
        
        rospy.wait_for_service('arm/controller_manager/load_controller')
        try:
            self.load_controller_proxy(self.arm_controller_load)
        except (rospy.ServiceException) as e:
            print('arm/controller_manager/load_controller service call failed')

        rospy.wait_for_service('arm/controller_manager/switch_controller')
        try:
            self.switch_controller_proxy(self.switch_controller)
        except (rospy.ServiceException) as e:
            print('arm/controller_manager/switch_controller service call failed')

        
        # nonzero = np.array([0,0,0,1])
        # self.joint_pos = self.starting_pos
        # self.all_joints.move(nonzero)
        # rospy.sleep(1)
        # self.all_joints.move(zero) 
        # print("in reset")
        # if(self.pause_physics()):            
        #     #set model's joint config
        #     self.reset_position()
        # else:
        #     print("Physics pause failure")
        # print("REEE")
        
        #self.unpause_physics()
        rospy.sleep(1)
       # rospy.wait_for_service('/gazebo/get_model_state')
       # model_state = self.get_model_state_proxy(self.get_model_state_req)
        #pos = np.array([model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z])
        done = False
        self.reward = 0.0
        self.state = np.zeros(self.state_shape)
        self.last_joint = self.joint_state
        #self.last_pos = pos
        diff_joint = np.zeros(self.num_joints)
        normed_js = self.normalize_joint_state(self.joint_state)
       # self.state = np.concatenate((normed_js,diff_joint,self.orientation,self.angular_vel,self.linear_acc_coeff*self.linear_acc)).reshape(1,-1)
        self.episode_start_time = rospy.get_time()
        self.last_action = np.zeros(self.num_joints)
        return self.state, done