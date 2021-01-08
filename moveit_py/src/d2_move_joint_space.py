#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from math import pi

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
    
    joint_goal[0] = -1.0115490297051097
    joint_goal[1] = -0.8238991149337542
    joint_goal[2] = 0.8239614788332459
    joint_goal[3] = -0.7933920325240735
    joint_goal[4] = -3.945688361841749e-05
    joint_goal[5] = 1.8050059279057975




    move_group.go(joint_goal, wait=True)
    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    print (current_joints)

    quit()