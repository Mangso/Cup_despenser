#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import roslib; roslib.load_manifest('ros_rdv')
from rdv_py.srv import *


def ctrl_client(num):
    rospy.wait_for_service('rdv_serial_py')
    try:
        tmp = rospy.ServiceProxy('rdv_serial_py',rdv_py);
        resp1 = tmp(num)
        return tmp.b

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "main":
    num = eval(input())
    if num >= 0 and num <=4000:
        ctrl_client(num)

