#! /usr/bin/env python

from __future__ import print_function
import roslib; roslib.load_manifest('ros_rdv')
from rdv_py.srv import rdv_py, rdvResponse

import rospy
import serial


def handle_rdv(req):

    print("Returning [%s]"%(req.a))

    ser = serial.Serial("/dev/ttyUSB0",57600,timeout=1)

    l1 = [0xFF, 0xFF, 0xFF, 0x01, 0x05, 0xF3, 0x86]
    
    dec = hex(int(req.a))
    dec_str = str(dec)

    if len(dec_str) == 6:
        p = dec_str[2:4]
        q = dec_str[4:]

    elif len(dec_str) == 5:
        p = dec_str[2:3]
        q = dec_str[3:]

    l1.append(int(p,16))
    l1.append(int(q,16))

    z = int(sum(l1[3:]))
    z = format(z,'b')

    final = z[-8:]

    inverse_s = '' 
    
    for k in final: 
        if k == '0': 
            inverse_s += '1'
            
        else: 
            inverse_s += '0'
    
    l1.append(int(inverse_s,2))

    ser.write(l1)

    return rdvResponse(req.a)

def add_two_ints_server():

    rospy.init_node('rdv_serial_py_server')
    s = rospy.Service('rdv_serial_py', rdv_py, handle_rdv)
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()