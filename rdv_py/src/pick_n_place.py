#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import MoveGroupActionResult, RobotState, Constraints, OrientationConstraint, MoveGroupActionGoal
import geometry_msgs.msg
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
import std_msgs.msg
from moveit_commander.conversions import pose_to_list, list_to_pose
import time

import moveit_msgs.msg

#from moveit_msgs.srv import GetMotionPlan

# moveit_msgs.msg import MoveGroupGoal
#import moveit_msgs.msg import MoveGroupGoal

#from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from std_msgs.msg import Int32MultiArray

from indy_driver_py.srv import *

class PickNPlaceTutorial():
    """PickNPlaceTutorial"""
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_n_place_tutorial', anonymous=True)

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot_group = moveit_commander.MoveGroupCommander('indy7')
        #self.hand_group = moveit_commander.MoveGroupCommander('hand')

        self.plan_result_pub = rospy.Publisher("/move_group/result", MoveGroupActionResult, queue_size=1)

        self.gripper_pub = rospy.Publisher("/robotis/pos",Int32MultiArray, queue_size = 1)

        self.robot_group.set_planner_id("RRTConnectkConfigDefault")
        self.robot_group.set_num_planning_attempts(5)
        self.robot_group.set_planning_time(5)
        #self.robot_group.allow_replanning(True)
        # Misc variables
        self.box_name = ''
        self.move_group_goal = MoveGroupActionGoal()

        self.move_group_goal_sub = rospy.Subscriber("/move_group/goal", MoveGroupActionGoal, self.move_group_goal_cb, queue_size=1)
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
        print ("converted indy7 vel %d",indy_vel)
    def hold_hand(self, target_name):
        touch_links = ['left_inner_finger', 'right_inner_finger']
        self.hand_group.attach_object(target_name, 'robotiq_85_base_link', touch_links=touch_links)

    def release_hand(self, target_name):
        self.hand_group.detach_object(target_name)

    def jmove_to_pose_goal(self, pose_goal):
        # self.robot_group.set_pose_target(pose_goal)
        plan = self.robot_group.plan(pose_goal)
        self.robot_group.execute(plan, wait=True)
        # self.robot_group.go(wait=True)

    def jmove_to_joint_goal(self, joint_goal):
        # self.robot_group.go(joint_goal, wait=True)
        plan = self.robot_group.plan(joint_goal)
        self.robot_group.execute(plan, wait=True)
        
    def tmove_to_pose_goal(self, pose_goal):
        waypoints = []
        waypoints.append(self.robot_group.get_current_pose().pose)
        waypoints.append(pose_goal)

        plan, _ = self.robot_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

        msg = MoveGroupActionResult()
        msg.result.planned_trajectory = plan
        self.plan_result_pub.publish(msg)
        self.robot_group.execute(plan)
    
    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def add_box(self, name, pose_stamp, size=(0.05, 0.05, 0.05)):
        self.scene.add_box(name, pose_stamp, size=size)
        self.wait_for_state_update(box_name=name, box_is_known=True)

    def show_current_joint(self):
        joint_values = self.robot_group.get_current_joint_values()
        print joint_values
    def show_current_pos(self):
        pos_values = self.robot_group.get_current_pose()
        print pos_values
    def go_cup_ready(self):

        joint_goal = [1.4966198335851, -1.326799297366056,
         -2.3646593370644577, -0.04153883619746379, 
         2.091894065899511, 1.56272327949214]
        self.jmove_to_pose_goal(joint_goal)

    def joint_move(self,joint_goal):
        self.jmove_to_pose_goal(joint_goal)
    def go_cup_pick(self):
        joint_goal = [1.5204237004310464, -1.2234713661491383, -2.060448448441854, -0.012849023353027792, 1.6853771921882825, 1.581778165977047]
        self.jmove_to_pose_goal(joint_goal)
    def go_cup_pick_up(self):
        joint_goal = [1.5213124031006353, -1.0696250320846978, -2.13191968131102, -0.010384709049366059, 1.601077789316958, 1.5834105228422943]
        self.jmove_to_pose_goal(joint_goal)
    def go_cup_pick_up_back(self):
        joint_goal = [1.485013394059338, -1.2205087459196038, -2.558914482811422, -0.05925392810520602, 2.176512843699474, 1.5491935569119]
        self.jmove_to_pose_goal(joint_goal)              
    
    def go_cup_juice_plate_1(self):
        joint_goal = [1.1585109273853935, -1.1412947340268882, -1.9651534712829661, 1.174693853054754, 1.5802267648312642, 1.5647516568196982]
        self.jmove_to_pose_goal(joint_goal)              
    
    def go_cup_juice_plate_2(self):
        joint_goal = [0.92240650967898, -1.172250392101962, -1.778228708394378, 0.9456193887305044, 1.4795156069155557, 1.4431953051824347]
        self.jmove_to_pose_goal(joint_goal)                         
    
    def go_gripper_pos(self,pos):
        print (pos)
        cup_hold = Int32MultiArray()
        cup_hold.data = [pos]
        self.gripper_pub.publish(cup_hold)

    def go_cup_hold(self):
        cup_hold = Int32MultiArray()
        cup_hold.data = [100]
        self.gripper_pub.publish(cup_hold)
    def go_cup_release(self):
        cup_hold = Int32MultiArray()
        cup_hold.data = [50]
        self.gripper_pub.publish(cup_hold)
    def go_testing_pos_move(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.0174523581312
        pose_goal.orientation.y = -0.999828809961
        pose_goal.orientation.z = 0.00613744673102
        pose_goal.orientation.w = 0.000312593759324

        pose_goal.position.x = 0.18
        pose_goal.position.y = 0.47
        pose_goal.position.z = 0.80

        move_group.set_pose_target(pose_goal)
        move_group.go()
    def go_orange_to_juice_pot2(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.620015838475
        pose_goal.orientation.y = -0.373841701158
        pose_goal.orientation.z = -0.332026067318
        pose_goal.orientation.w = 0.604633304687

        pose_goal.position.x = -0.561062699341
        pose_goal.position.y = -0.133818775573
        pose_goal.position.z = 1.62573164543

        move_group.set_pose_target(pose_goal)
        move_group.go()
    def go_orange_pick_pos(self):
        joint_goal = [-1.525941365018604, -0.14172073526193607, -0.9297368925373561, 0.0006108652381979773, -2.083661327493378, 1.5812857556003057]
        self.jmove_to_pose_goal(joint_goal)              
    
    def go_orange_to_juice_pot1(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.620121216554
        pose_goal.orientation.y = -0.374091217336
        pose_goal.orientation.z = -0.331611714704
        pose_goal.orientation.w = 0.604598303473

        pose_goal.position.x = -0.448826582349
        pose_goal.position.y = 0.0282399813568
        pose_goal.position.z = 1.68930662465

        move_group.set_pose_target(pose_goal)
        move_group.go()
    def go_juicer_pick_cup_ok(self):
        joint_goal = [1.1844676968659216, -1.2657127735462559, -1.8955148341283938, 1.187522023056912, 1.601077789316958, 1.4844723419921242]
        self.jmove_to_pose_goal(joint_goal)   

    def test_constraints(self):


        constraint = Constraints()
        constraint.name = "tilt constraint"
        ocm = OrientationConstraint()
        # 'base_link' is equal to the world link
        ocm.header.frame_id = "world"
        # The link that must be oriented upwards
        ocm.link_name = "tcp"
        # ocm.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
        ocm.orientation.x =  0.683951325248
        ocm.orientation.y =  -0.209429903965
        ocm.orientation.z =  -0.202265206016
        ocm.orientation.w =  0.668908429049

        # Allow rotation of 45 degrees around the x and y axis
        ocm.absolute_x_axis_tolerance = 0.5 #Allow max rotation of 45 degrees
        ocm.absolute_y_axis_tolerance = 1.5 #Allow max rotation of 360 degrees
        ocm.absolute_z_axis_tolerance = 0.5 #Allow max rotation of 45 degrees
        # The tilt constraint is the only constraint
        ocm.weight = 1
        constraint.orientation_constraints = [ocm]

        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.683951325248
        pose_goal.orientation.y = -0.209429903965
        pose_goal.orientation.z = -0.202265206016
        pose_goal.orientation.w = 0.668908429049

        pose_goal.position.x = -0.461741097024
        pose_goal.position.y = -0.433851093412
        pose_goal.position.z = 1.26149858128
        move_group.set_pose_target(pose_goal)

        move_group.set_path_constraints(constraint)
        move_group.plan()
       

    def test_constraints2(self):
        move_group = self.robot_group
        move_group.clear_path_constraints()
        self.go_juicer_pick_cup_ok()
        self.test_constraints()
        # self.go_sealing_step1()
    def go_juicer_constraint_test(self):

        
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.683951325248
        pose_goal.orientation.y = -0.209429903965
        pose_goal.orientation.z = -0.202265206016
        pose_goal.orientation.w = 0.668908429049

        pose_goal.position.x = -0.461741097024
        pose_goal.position.y = -0.433851093412
        pose_goal.position.z = 1.26149858128
        move_group.set_pose_target(pose_goal)
        move_group.go()

    def go_sealing_step1(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.683951325248
        pose_goal.orientation.y = -0.209429903965
        pose_goal.orientation.z = -0.202265206016
        pose_goal.orientation.w = 0.668908429049

        pose_goal.position.x = -0.461741097024
        pose_goal.position.y = -0.433851093412
        pose_goal.position.z = 1.26149858128

        move_group.set_pose_target(pose_goal)
        move_group.go()
    def go_sealing_step2(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.683951325248
        pose_goal.orientation.y = -0.209429903965
        pose_goal.orientation.z = -0.202265206016
        pose_goal.orientation.w = 0.668908429049

        pose_goal.position.x = -0.545566440256
        pose_goal.position.y = -0.515964899805
        pose_goal.position.z = 1.26149858128

        move_group.set_pose_target(pose_goal)
        move_group.go()

    def go_sealing_goal(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.683951325248
        pose_goal.orientation.y = -0.209429903965
        pose_goal.orientation.z = -0.202265206016
        pose_goal.orientation.w = 0.668908429049

        pose_goal.position.x = -0.545566440256
        pose_goal.position.y = -0.515964899805
        pose_goal.position.z = 1.05706463542

        move_group.set_pose_target(pose_goal)
        move_group.go()

    def go_juicer_push_ready(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = -0.489606100026
        pose_goal.orientation.y = -0.500430269326
        pose_goal.orientation.z = 0.49569733542
        pose_goal.orientation.w = 0.513945098252

        pose_goal.position.x = -0.398840350759
        pose_goal.position.y = -0.201061446823
        pose_goal.position.z = 1.04976587286

        move_group.set_pose_target(pose_goal)
        move_group.go()

    def go_juicer_pop(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = -0.489606100026
        pose_goal.orientation.y = -0.500430269326
        pose_goal.orientation.z = 0.49569733542
        pose_goal.orientation.w = 0.513945098252

        pose_goal.position.x = -0.418840350759
        pose_goal.position.y = -0.201061446823
        pose_goal.position.z = 1.04976587286

        move_group.set_pose_target(pose_goal)
        move_group.go()
    def go_juicer_push(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = -0.489606100026
        pose_goal.orientation.y = -0.500430269326
        pose_goal.orientation.z = 0.49569733542
        pose_goal.orientation.w = 0.513945098252

        pose_goal.position.x = -0.418840350759
        pose_goal.position.y = -0.201061446823
        pose_goal.position.z = 1.02876587286

        move_group.set_pose_target(pose_goal)
        move_group.go()        
    def service_test(self):
        while False:
            ctrl_rg = rospy.ServiceProxy('Srv_Robotis_Gripper_States', robotis_gripper)
            resp = ctrl_rg(10)
            print resp
            time.sleep(0.1)
    def cup_5(self):
        cup_ready_joint = [-0.391565,-0.898583,-1.68416,1.24128,1.37323,-2.09087]
        self.joint_move(cup_ready_joint)
    def cup_6(self):   
        self.go_gripper_pos(0)
        cup_tophold_joint = [-0.3871867344150098, -1.1022436382952863, -1.6139942082902186, 1.2301102050538066, 1.405831691137374, -1.9886570599445468]
        self.joint_move(cup_tophold_joint)     
    def cup_7(self):
        self.go_gripper_pos(255)
        time.sleep(1)
        cup_topholdup_joint = [-0.38681591674521093, -1.071543421555916, -1.6347108723811592, 1.2342365754033977, 1.3927529861277888, -2.01096433006541]
        self.joint_move(cup_topholdup_joint)
    def cup_8(self):    
        self.go_gripper_pos(0)
        cup_hold_joint = [-0.3798179217459306, -1.277356736439644, -1.5299580500447836, 1.2160490309133662, 1.4174580504752605, -1.8313129674584225]
        self.joint_move(cup_hold_joint)
        self.go_gripper_pos(170)
        time.sleep(1)
    def cup_9(self):
        cup_holdup_1_joint = [-0.3811292534628473, -1.1970350936557836, -1.5791856804014743, 1.2196200926237786, 1.4061392466913807, -1.8609085448128169]
        self.joint_move(cup_holdup_1_joint)
        cup_holdup_joint = [-0.39156523536224974, -0.8909837408000261, -1.7027337291166913, 1.2414546384671867, 1.3415383416668982, -2.0982607466354124]
        self.joint_move(cup_holdup_joint)
    def cup_release(self):
        cup_release_joint = [-0.3319967045599512, -0.8275517335918747, -1.3078667902714252, 1.3919185993007148, 1.3002100894744664, -2.5479344947139215]
        self.joint_move(cup_release_joint)
        self.go_gripper_pos(0)
    def cup_init(self):
        self.cup_5()
        cup_reset_joint = [-0.38217911118182907, -1.2281512917555053, -1.5429144828151904, 1.2207867153888228, 1.4404820009766026, -1.9118241769295343]
        self.go_gripper_pos(255)
        time.sleep(1)
        self.joint_move(cup_reset_joint)
        self.cup_5()
def main():
    try:
        pnp = PickNPlaceTutorial()
        pnp.show_current_joint()
        pnp.show_current_pos()
        pnp.service_test()
        while True:            
            input = raw_input()
            if( input == '1'):
                pnp.go_cup_ready()
            elif( input == '2'):
                pnp.go_cup_pick()
                pnp.go_gripper_pos(60)

                joint_goal = [1.5111731241567696, -1.2281306907738498, -2.068684813172132, -0.035447676090357226, 1.7236447924176177, 1.543485583448304]
                pnp.joint_move(joint_goal)
                pnp.go_gripper_pos(65)

                joint_goal2 = [1.5112634515378744, -1.177508532428172, -2.096267942917937, -0.035507478559191875, 1.699514970865603, 1.5443550923602491]
                pnp.joint_move(joint_goal2)
                

                ##pnp.go_cup_pick_up()
                #pnp.go_cup_pick_up_back()
            elif( input == '3'):
                pnp.go_cup_juice_plate_1()

                juicer_cup_pos1 = [0.9358011763915453, -1.1023561513489433, -1.8302015107645895, 0.9564246810547786, 1.460044053004294, 1.4085133097795628]
                pnp.joint_move(juicer_cup_pos1)                

                juicer_cup_pos2 = [0.9359556837539614, -1.2160442533056632, -1.7734759154307487, 0.95329122153695, 1.4924541433760656, 1.4542251778621282]
                pnp.joint_move(juicer_cup_pos2)
                
                pnp.go_gripper_pos(0)
                juicer_cup_pos3 = [1.2041448093632727, -1.2862674533510918, -1.9188213874283713, 1.2196466714988161, 1.6036563595513371, 1.6373251495052477]
                pnp.joint_move(juicer_cup_pos3)
            elif( input == '3.1'):
                pnp.go_orange_pick_pos()
            elif( input == '4'):
                pnp.go_orange_to_juice_pot1()                
                pnp.go_orange_to_juice_pot2()
                pnp.go_cup_release()
                time.sleep(0.5)
                pnp.go_orange_to_juice_pot1()             
            elif( input == '5'):
                pnp.cup_5()
            elif( input == '6'):
                pnp.cup_6()
            elif( input == '7'):
                pnp.cup_7()
            elif( input == '8'):
                pnp.cup_8()
            elif( input == '9'):
                pnp.cup_9()
            elif( input == 'all'):
                pnp.cup_5()
                pnp.cup_6()
                pnp.cup_7()
                pnp.cup_8()
                pnp.cup_9()
                pnp.cup_release()
                pnp.cup_init()
            elif( input == 'loop'):
                pnp.cup_5()
                pnp.cup_6()
                pnp.cup_7()
                pnp.cup_8()
                pnp.cup_9()
                pnp.cup_release()
                pnp.cup_init()
            elif( input == 'r'):
                pnp.go_gripper_pos(0)       
            elif( input == 't'):
                pnp.go_testing_pos_move()
            elif( input == 'q'):
                break
            elif( input == 'a1'):
                pnp.test_constraints()
            elif( input == 'a2'):
                pnp.test_constraints2()
            elif( input == 'g'):
                gripper_pos_input = raw_input("0~255 gripper pos input:")
                print(gripper_pos_input)
                if gripper_pos_input.isdigit() == True:
                    print("number ok")
                    gripper_val = int(gripper_pos_input)
                    if gripper_val >= 0 and gripper_val <= 255:
                        pnp.go_gripper_pos(gripper_val)
                    else:
                        print("not available gripper pos area ( 0 ~ 255 ")
                else:
                    print("not number")
            else:
                pnp.show_current_joint()
                pnp.show_current_pos()
            
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    else:
        print("============ Python pick place demo complete!")
if __name__ == '__main__':
  main()

