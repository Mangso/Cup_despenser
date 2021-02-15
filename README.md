### 컴파일 환경
→ Ubuntu 18.04.5 LTS & ros melodic


## 컴파일 방법

'''
cd ~/catkin_ws/src
git clone https://github.com/Mangso/ros_rdv.git
catkin_make
'''

### Gripper & dispenser 초기화

#### dispenser

'''
sudo chmod 0666 /dev/ttyUSB0
'''

#### Gripper

'''
roscd  indy_driver_py/src && python gripper_row_init.py
'''

### launch 파일 실행 

'''
roslaunch indy_driver_py rdv_dcp.launch robot_ip:=192.168.212.17 robot_name:=NRMK-Indy7
'''

### rdv_cup 노드 실행

'''
rosrun ros_rdv rdv_cup
'''
