#!/usr/bin/python
#-*- coding: utf-8 -*-

from indy_utils import indydcp_client as client
from indy_utils import indy_program_maker
import utils_transf
import json


robot_ip = "192.168.212.17"    # 예시 STEP IP 주소
robot_name = "NRMK-Indy7"   # IndyRP2의 경우 "NRMK-IndyRP2"
indy = client.IndyDCPClient(robot_ip, robot_name) # indy 객체 생성

indy.connect() # 로봇 연결

# 로봇의 상태 출력.
status = indy.get_robot_status()
print("robot_status : " ,status)
print


# 관절 공간에서의 각도와 작업 공간에서의 위치방향 
joint_pos = indy.get_joint_pos() # [q1, q2, q3, q4, q5, q6]
task_pos = indy.get_task_pos()   # [x, y, z, u, v, w]

print("joint_pos : ", joint_pos)
print
print("task_pos : ", task_pos)
print
#  관절 및 작업 공간의 속도와 각 관절 토크 값 
print("joint_vel : " , indy.get_joint_vel())
print
print("task_vel : " , indy.get_task_vel())
print
print("control_torque : " , indy.get_control_torque())
print
# 작업 공간에서 위치(task_pos)에 해당하는 관절 공간의 위치 값.
print("get_inv_kin :" ,indy.get_inv_kin(task_pos = [0.5, -0.2, 0.3, 180, -10, 180],
                         init_q = [0, -20, -90, 0, -60, 0]))

print
# 로봇 모션 명령 (단일 경유점에 대한 모션 명령)

# ----홈위치.
indy.go_home()

# ----제로 위치
# indy.go_zero()
j_pos3= [-0.3877049283381902, -0.8175309407073752, -1.7243005798736448, 1.240622150131187, 1.3587633511823527, -2.110715797327151]
j_pos3 = utils_transf.rads2degs(j_pos3)
# indy.set_joint_vel_level(3)
indy.set_joint_blend_radius(3)
# -----조인트 무브
j_pos1 = [-21.09, -28.41, -70, 0.00, -81.49, -21.31] # degree
# indy.joint_move_to(j_pos1)

j_pos2 = [-21.1, -30.4, -41.81, 0.02, -108.05, -21.28]
indy.joint_move_to(utils_transf.rads2degs(j_pos1))
# # # -----태스크 무브
# t_pos1 = [0.5, -0.2, 0.3, 180, -10, 180]
# indy.task_move_to(t_pos1)


prog = indy_program_maker.JsonProgramComponent(policy=0, resume_time=2)

prog.add_joint_move_to(j_pos1, vel = 2, blend = 3)
prog.add_joint_move_to(j_pos2, vel = 2, blend = 3)
prog.add_joint_move_to(j_pos3, vel = 2, blend = 3)

json_string = json.dumps(prog.json_program)
indy.set_and_start_json_program(json_string)

# prog.add_joint_move_to(utils_transf.rads2degs(j_pos2), vel= 1, blend=5)
# print(utils_transf.degs2rads(j_pos2))











indy.disconnect() # 연결 해제
