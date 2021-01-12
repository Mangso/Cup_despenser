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
    joint_goal[0] = float(sys.argv[1])
    joint_goal[1] = float(sys.argv[2])
    joint_goal[2] = float(sys.argv[3])
    joint_goal[3] = float(sys.argv[4])
    joint_goal[4] = float(sys.argv[5])
    joint_goal[5] = float(sys.argv[6])

    #for item in joint_goal:
    print(joint_goal),
    print()
    # print math.cos(math.radians(joint_goal[1]))
    # print math.cos(math.radians(joint_goal[2]))
    # print math.cos(math.radians(joint_goal[3]))
    # print math.cos(math.radians(joint_goal[4]))
    # print math.cos(math.radians(joint_goal[5]))
    # joint_goal = [2.2411126525922413, -1.3436602035523912, -2.0697909274179422, 0.6622724740312428, 1.7868588497124587, 1.7113539612037907]

    # joint_goal2 = [1.5204237004310464, -1.2234713661491383, -2.060448448441854, -0.012849023353027792, 1.6853771921882825, 1.581778165977047]
	
    radians_ = []
    for item2 in joint_goal:
        radians_.append(math.radians(item2))

    print(radians_)

    #move_group.go(joint_goal, wait=True)
    #move_group.stop()

    #current_joints = move_group.get_current_joint_values()
    #print (current_joints)

    quit()
