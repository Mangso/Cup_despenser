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
    
    
    // 세팅
    while(tmp--){
        rdv.step2();

        // 내려가고 
        rdv.step1();

        // 잡고
        while(ros::ok()){
            rdv.goToGripperState(200);
            ros::Duration(2).sleep();
            break;
        }

        rdv.goOutDispenser();
        ros::Duration(2).sleep();
        // 올라가고 
        rdv.step2();

        rdv.goInDispenser();
        ros::Duration(2).sleep();

        // 옆으로 옮기고
        rdv.step3();

        while(ros::ok()){
            rdv.goToGripperState(0);
            ros::Duration(2).sleep();
            break;
        }
    }

#if 0
    rdv.step3();
    rdv.step4();

    while(ros::ok()){
        rdv.goToGripperState(100);
        ros::Duration(2).sleep();
        rdv.goInDispenser();
        loop_rate.sleep();
        break;
    }

    rdv.step1();

#endif 

    ros::spinOnce();
    
    spinner.stop();

    return 0;
}