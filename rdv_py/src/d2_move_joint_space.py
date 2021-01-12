#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import math

if __name__ == "__main__":
    # Initialize moveit_commander and node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('d2_move_joint_space', anonymous=False)

    # Get instance from moveit_commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Get group_commander from MoveGroupCommander
    group_name = "indy7"
    move_group = moveit_commander.MoveGroupCommander(group_name)


    # Move using joint space
    joint_goal = move_group.get_current_joint_values()
    print (joint_goal)
    
    # joint_goal[0] = -1.0115490297051097
    # joint_goal[1] = -0.8238991149337542
    # joint_goal[2] = 0.8239614788332459
    # joint_goal[3] = -0.7933920325240735
    # joint_goal[4] = -3.945688361841749e-05
    # joint_goal[5] = 1.8050059279057975
    # joint_goal[0] = 0
    # joint_goal[1] = -math.pi/2
    # joint_goal[2] = math.pi/2 # degree
    # joint_goal[3] = 0
    # joint_goal[4] = math.pi/2
    # joint_goal[5] = 0

    # for item in joint_goal:
    #     print math.degrees(item, end = '')
    # print math.cos(math.radians(joint_goal[2]))
    # print math.cos(math.radians(joint_goal[3]))
    # print math.cos(math.radians(joint_goal[4]))
    # print math.cos(math.radians(joint_goal[5]))
    # joint_goal = [2.2411126525922413, -1.3436602035523912, -2.0697909274179422, 0.6622724740312428, 1.7868588497124587, 1.7113539612037907]
    # joint_goal2 = [1.5204237004310464, -1.2234713661491383, -2.060448448441854, -0.012849023353027792, 1.6853771921882825, 1.581778165977047]

    # 
    # for item2 in joint_goal2:
    #     print(round(math.degrees(item2),2))  

    joint_goal = [-0.3611086222376268, -0.8412486994612669, -1.772032789549843, 1.2960815025309893, 1.3639748104335687, -2.063153708782497]
    move_group.go(joint_goal, wait=True)

    joint_goal = [-0.4445353604829557, -1.125911900461542, -1.5700981950940986, 1.2007865253720986, 1.3550736312483975, -1.9805996351631652]
    move_group.go(joint_goal, wait=True)

    joint_goal = [-0.45919612619970807, -0.8293804605477054, -1.6674875673553826, 1.2362167091875835, 1.2791518087866443, -2.1715386553313447]
    move_group.go(joint_goal, wait=True)


    joint_goal = [-0.13770647798235258, -0.8157668923821496, -1.6913985781077048, 1.0732029570513133, 1.2056734472776829, -1.048942880448592]
    move_group.go(joint_goal, wait=True)

    joint_goal = [0.37437312455278365, -0.7867944267990438, -1.73293741430517, 0.5295328950550796, 0.9892526200303859, 1.2952088379049917]
    move_group.go(joint_goal, wait=True)
    
#     joint_goal =  [-0.38681591674521093, -1.071543421555916, -1.6347108723811592, 1.2342365754033977, 1.3927529861277888, -2.01096433006541]
#     move_group.go(joint_goal, wait=True)

#     joint_goal =  [-0.527962098728 ,-0.53808500839, -2.18009076867, 2.650631535, -1.18891828646, -1.46433124242]
#     move_group.go(joint_goal, wait=True)
    
#     move_group.stop()

    current_joints = move_group.get_current_joint_values()
    print (current_joints)

    quit()