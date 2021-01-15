// ROS Includes
#include <ros/ros.h>

// User defined includes
#include </home/somag/catkin_ws/src/ros_rdv/src/include/rdv_core.h>
#include "ros_rdv/rdv.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rdv_cup");

    RdvCupNode rdv;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    
    ros::Rate loop_rate(10);
    int tmp =3;
    
  
    // 자세 시작.
    rdv.jmove_pickup_init_pos();
    // 컵 집으러 자세 낮춤.
    rdv.jmove_pickup_hold_pos();

    // 그리퍼로 컵 집음.
    while(ros::ok()){
        rdv.goToGripperState(200);
        ros::Duration(2).sleep();
        break;
    }

    // 디스펜서로 컵 고정.
    rdv.goOutDispenser();
    ros::Duration(2).sleep();

    // 컵 집고 자세 올라감.
    rdv.jmove_pickup_hold_up_pos();

    rdv.go_on_Dispenser();
    ros::Duration(2).sleep();

    rdv.jmove_pickup_drop_pos();

    while(ros::ok()){
        rdv.goToGripperState(0);
        ros::Duration(2).sleep();
        break;
    }


    ros::spinOnce();
    spinner.stop();

    return 0;
}