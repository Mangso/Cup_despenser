#### 컴파일 환경

  Ubuntu 18.04.5 LTS & ros melodic


### 컴파일 방법

```
cd ~/catkin_ws/src
git clone https://github.com/Mangso/ros_rdv.git
cd ~/catkin_ws && catkin_make
```

#### Gripper & dispenser 초기화

```
sudo chmod 0666 /dev/ttyUSB0
roscd  indy_driver_py/src && python gripper_row_init.py
```

#### launch 파일 실행 및 노드 실행 (터미널 다르게)

```
roslaunch indy_driver_py rdv_dcp.launch robot_ip:=192.168.212.17 robot_name:=NRMK-Indy7
rosrun ros_rdv rdv_cup
```


## 주의 사항

rdv_core.cpp , rdv_run.cpp 에 포함되어 있는 헤더파일이 아래와 같이 절대경로로 지정함. 
(상대경로시 못찾아서 방안으로 이렇게 함.)

```
#include </home/somag/catkin_ws/src/ros_rdv/src/include/rdv_core.h>
```