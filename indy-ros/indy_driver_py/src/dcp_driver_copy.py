#!/usr/bin/python
#-*- coding: utf-8 -*-

import sys
import json

from indy_utils import indydcp_client
from indy_utils import indy_program_maker
import threading
import rospy

import time
from moveit_msgs.msg import MoveGroupActionGoal
from sensor_msgs.msg import JointState



from std_msgs.msg import Header, Bool, Empty, Int32MultiArray
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryFeedback
import utils_transf
import logging

from std_srvs.srv import Trigger, TriggerRequest
from indy_driver_py.srv import robotis_gripper, robotis_gripperResponse, indy_collision_stop

ROBOT_STATE = {
    0: "robot_ready", 
    1: "busy",
    2: "direct_teaching_mode",
    3: "collided",
    4: "emergency_stop",
}

logging.basicConfig(level=logging.INFO)
LOG = logging.getLogger(__name__)

class MyThread(threading.Thread):
    def __init__(self, parent=None):
        self.parent = parent
        self.pos_msg = Int32MultiArray()
        self.msg_alive = False
        super(MyThread, self).__init__()

    def robotis_gripper_init(self):
        # Position Control Mode
        self.parent.indy.set_gripper_mode(48)
        #time.sleep(2)
        # disconnect to the robot
        self.parent.indy.set_gripper_cmd(255)
        time.sleep(1)
        self.parent.indy.set_gripper_cmd(0)

    def robotis_gripper_publisher(self):
        self.parent.indy.set_gripper_mode(160)
        cur = self.parent.indy.get_gripper_curdata()
        self.parent.indy.set_gripper_mode(176)
        pos = self.parent.indy.get_gripper_posdata()        
        gripper_pos = Int32MultiArray()
        gripper_pos.data = [cur,pos]      
        if self.parent.rospy.is_shutdown() == False:
            self.parent.robotis_gripper_pub.publish(gripper_pos)
    def service_robotis_gripper(self,req):
        req_type = req.type
        ret = robotis_gripperResponse()

        self.parent.indy.set_gripper_mode(160)
        cur = self.parent.indy.get_gripper_curdata()
        self.parent.indy.set_gripper_mode(176)
        pos = self.parent.indy.get_gripper_posdata()       

        ret.current = cur
        ret.position = pos
        return ret

    def execute_robotis_gripper_cb(self,msg):        
        #self.pos_msg = msg
        #self.msg_alive = True
        grip_pos = msg.data
        LOG.info("-----------set gripper /robotis/pos=")
        self.parent.indy.set_gripper_mode(48)
        self.parent.indy.set_gripper_cmd(grip_pos[0])

    '''def run(self):
        while not self.parent.rospy.is_shutdown():
            time.sleep(0.5)
            self.robotis_gripper_publisher()
            if self.msg_alive == True:
                grip_pos = self.pos_msg.data
                #LOG.info("-----------set gripper /robotis/pos=")
                self.parent.indy.set_gripper_mode(48)
                self.parent.indy.set_gripper_cmd(grip_pos[0])
                self.msg_alive = False'''
                
        # ...
        #self.parent and self.parent.on_thread_finished(self, 42)

class IndyROSConnector:
    def __init__(self, robot_ip, robot_name):
        self.robot_name = robot_name
        LOG.info("--------------------hjarpjk--------------------------")
        # Connect to Robot
        self.indy = indydcp_client.IndyDCPClient(robot_ip, robot_name)

        # Initialize ROS node
        rospy.init_node('indy_driver_py')
        self.rospy = rospy
        self.rate = rospy.Rate(10) # hz
        self.robotis_thread = MyThread(parent=self)

        s = rospy.Service('Srv_Robotis_Gripper_States', robotis_gripper, self.robotis_thread.service_robotis_gripper)

        # self.stop_service = rospy.Service('Srv_indy_stop', indy_collision_stop ,self.set_robot_restore)
        
        # Publish current robot state
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.indy_state_pub = rospy.Publisher("/indy/status", GoalStatusArray, queue_size=10)
        self.control_state_pub = rospy.Publisher("/feedback_states", FollowJointTrajectoryFeedback, queue_size=1)

        # Subscribe desired joint position
        self.joint_execute_plan_sub = rospy.Subscriber("/joint_path_command", JointTrajectory, self.execute_plan_result_cb, queue_size=1)

        # Subscribe command
        self.execute_joint_state_sub = rospy.Subscriber("/indy/execute_joint_state", JointState, self.execute_joint_state_cb, queue_size=1)
        self.stop_sub = rospy.Subscriber("/stop_motion", Bool, self.stop_robot_cb, queue_size=1)
        self.set_motion_param_sub = rospy.Subscriber("/indy/motion_parameter", Int32MultiArray, self.set_motion_param_cb, queue_size=1)
        

        self.set_robotis_gripper = rospy.Subscriber("/robotis/pos", Int32MultiArray, self.robotis_thread.execute_robotis_gripper_cb, queue_size=1)
        #self.robotis_gripper_pub = rospy.Publisher("/robotis/pos_states", Int32MultiArray, queue_size=1)

        # Misc variable
        self.joint_state_list = []
        self.execute = False
        self.vel = 2
        self.blend = 5
        self.move_group_goal = MoveGroupActionGoal()
        self.move_group_goal_sub = rospy.Subscriber("/move_group/goal", MoveGroupActionGoal, self.move_group_goal_cb, queue_size=1)

    def handle_add_two_ints(self, req):
        data = Int32MultiArray()
        data.data = [10,20]
        return data

    def move_group_goal_cb(self, msg):        
        print ("test")
        self.move_group_goal = msg
        print (self.move_group_goal.goal.request.max_velocity_scaling_factor)
        VelMin = 1
        VelMax = 9
        VelFactorMin = 0.0
        VelFactorMax = 1.0
        plan_vel_scaling_factor = msg.goal.request.max_velocity_scaling_factor
        indy_vel = (((plan_vel_scaling_factor - VelFactorMin) * (VelMax - VelMin)) / (VelFactorMax - VelFactorMin)) + VelMin
        rospy.loginfo("converted indy7 vel %d",indy_vel)
        self.vel = indy_vel

    def __del__(self):
        self.indy.disconnect()

    def execute_joint_state_cb(self, msg):
        # 들어오는 좌표들 다 여기다 계속 저장하나봠.
        self.joint_state_list = [msg.position]
        
        print("joint_state_list : ", self.joint_state_list)

        if self.execute == False:
            self.execute = True

    def execute_plan_result_cb(self, msg):
        # download planned path from ros moveit
        self.joint_state_list = []
        if msg.points:
            self.joint_state_list = [p.positions for p in msg.points]
        else:
            self.indy.stop_motion()

        if self.execute == False:
            self.execute = True
    
    def stop_robot_cb(self, msg):
        if msg.data == True:
            self.indy.stop_motion()

    def set_motion_param_cb(self, msg):
        param_array = msg.data
        self.vel = param_array[0]
        self.blend = param_array[1]

    def move_robot(self):
        if self.joint_state_list:
            LOG.info("joint move gogo!")
            prog = indy_program_maker.JsonProgramComponent(policy=0, resume_time=2)
                
            for j_pos in self.joint_state_list:
                prog.add_joint_move_to(utils_transf.rads2degs(j_pos), vel= 1, blend=5)
            
            json_string = json.dumps(prog.json_program)
            # 이게 시작하는 프로그램 같음.
            self.indy.set_and_start_json_program(json_string)
             # 좌표 값 다시 초기화
            self.joint_state_list = []

    def joint_state_publisher(self):
        LOG.info("QWERQWERQWERQWEQWRQWERQWERWQERQWERQWERQWERQWERQWERERQWQREWQWREQWERQWER!")
        print('comecomecomecomecomecomecomecomecomecomecome')
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()

        if self.robot_name == 'NRMK-IndyRP2':
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        else:
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        cur_joint_pos = self.indy.get_joint_pos()

        joint_state_msg.position = utils_transf.degs2rads(cur_joint_pos)
        joint_state_msg.velocity = [self.vel]
        joint_state_msg.effort = []

        control_state_msg = FollowJointTrajectoryFeedback()
        control_state_msg.header.stamp = rospy.Time.now()
        if self.robot_name == 'NRMK-IndyRP2':
            control_state_msg.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        else:
            control_state_msg.joint_names = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        control_state_msg.actual.positions = utils_transf.degs2rads(cur_joint_pos)
        control_state_msg.desired.positions = utils_transf.degs2rads(cur_joint_pos)
        control_state_msg.error.positions = [0 for i in control_state_msg.joint_names]


        self.joint_state_pub.publish(joint_state_msg)
        self.control_state_pub.publish(control_state_msg)
        
    def robot_state_publisher(self):
        if self.current_robot_status['ready']:
            state_num = 0

        if self.current_robot_status['busy']:
            state_num = 1

        if self.current_robot_status['direct_teaching']:
            state_num = 2

        if self.current_robot_status['collision']:
            state_num = 3

        if self.current_robot_status['emergency']:
            state_num = 4

        status_msg = GoalStatusArray()
        status_msg.header.stamp = rospy.Time.now()

        status = GoalStatus()
        status.goal_id.stamp = rospy.Time.now()
        status.goal_id.id = ""
        status.status = state_num
        status.text = ROBOT_STATE[state_num]

        status_msg.status_list=[status]

        self.indy_state_pub.publish(status_msg)


    # def set_robot_restore(self):

    def run(self):
        self.indy.connect()
        self.robotis_thread.robotis_gripper_init()        

       # self.robotis_thread.start()
        while not rospy.is_shutdown():
            time.sleep(0.01)
            self.current_robot_status = self.indy.get_robot_status()
            self.joint_state_publisher()
            self.robot_state_publisher()
            #self.robotis_gripper_publisher()

            # 조인트 무브 들어왔을 때 self.execute 가 false면 true로
            if self.execute:
                # 한번 실행되면 다시 초기화 시키나봄.
                self.execute = False
                if self.current_robot_status['busy']:
                    continue
                if self.current_robot_status['direct_teaching']:
                    continue
                if self.current_robot_status['collision']:
                    continue
                if self.current_robot_status['emergency']:
                    continue
                    #self.reset_robot()
                if self.current_robot_status['ready']:
                    self.move_robot()

        self.indy.disconnect()
        #self.robotis_thread.terminate()

def main():
    robot_ip = rospy.get_param("robot_ip_address")
    robot_name = rospy.get_param("robot_name")
    t = IndyROSConnector(robot_ip, robot_name)
    t.run()
                
if __name__ == '__main__':
    main()
        
