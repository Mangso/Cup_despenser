// ROS Includes
#include <ros/ros.h>

// User defined includes
#include </home/somag/catkin_ws/src/ros_rdv/src/include/rdv_core.h>
#include "ros_rdv/rdv.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rdv_cup");

    RdvCupNode rdv;
    int tmp(3);
    while(tmp--){
        rdv.run();
    }

    //  rdv;
    // rdv.runz();

    return 0;
}