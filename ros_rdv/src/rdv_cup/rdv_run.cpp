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
    while(tmp--){
        while(ros::ok()){
            rdv.goToGripperState(0);
            ros::Duration(2).sleep();
            break;
        }

        rdv.step1();
        rdv.step2();

        while(ros::ok()){
            rdv.goToGripperState(215);
            ros::Duration(1.5).sleep();
            rdv.goOutDispenser();
            loop_rate.sleep();
            break;
        }


        rdv.step3();
        rdv.step4();

        while(ros::ok()){
            rdv.goToGripperState(100);
            ros::Duration(1).sleep();
            rdv.goInDispenser();
            loop_rate.sleep();
            break;
        }

        rdv.step1();
    


        ros::spinOnce();
    }

    spinner.stop();

    return 0;
}