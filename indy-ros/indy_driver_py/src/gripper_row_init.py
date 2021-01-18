#!/usr/bin/python
from indy_utils import indydcp_client as client

import json
from time import sleep
import threading
import numpy as np

robot_ip = "192.168.212.17" # Robot ( Indy ) IP
robot_name = "NRMK-Indy7" # Robot name ( Indy7 )
 # robot_name = "NRMK - IndyRP2 " # Robot name ( IndyRP2 )

 # Create class object
indy = client.IndyDCPClient ( robot_ip , robot_name )

indy.connect()

indy.set_gripper_mode(32)
sleep(3)

indy.set_gripper_cmd(0)
sleep(3)

indy.set_gripper_mode(16)
sleep(3)

indy.set_gripper_cmd(5)
sleep(3)

indy.set_gripper_mode(32)
sleep(3)

indy.set_gripper_cmd(1)
sleep(3)


# Position Control Mode
indy.set_gripper_mode (48)
# disconnect to the robot

sleep(3)

indy.set_gripper_cmd(0)

sleep(2)

indy.set_gripper_cmd(255)

sleep(2)

indy.set_gripper_cmd(50)

indy.disconnect ()
