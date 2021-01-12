#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import math

if __name__ == "__main__":
    # Initialize moveit_commander and node
    # moveit_commander 
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('d1_get_basic_info', anonymous=False)

    # Get instance from moveit_commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Get group_commander from MoveGroupCommander
    group_name = "indy7"
    move_group = moveit_commander.MoveGroupCommander(group_name)


    # Get information
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()
    current_state = robot.get_current_state()

    print "============ Planning frame: %s" % planning_frame
    print "============ End effector link: %s" % eef_link
    print "============ Available Planning Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print current_state
    print "="*20
    

    joint_goal = [-30.25, -30.83, -124.91, 151.87, -68.12, -83.90]

    for item2 in joint_goal:
        # print(round(math.degrees(item2),2))
        print(math.radians(item2)),
    # joint_goal = [-0.3871867344150098, -1.1022436382952863, -1.6139942082902186, 1.2301102050538066, 1.405831691137374, -1.9886570599445468]



    # for item2 in joint_goal:
    #     print(round(math.degrees(item2),2))
    #     print(math.radians(round(math.degrees(item2),5)))

    

    quit()
