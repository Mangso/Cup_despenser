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

    ros_rdv::rdv srv;
    
    rdv.step1();
    //rdv.goToDispenser(srv,1000);
    //rdv.goToGripperState(50);

    return 0;
}