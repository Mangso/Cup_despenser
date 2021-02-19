#include <ros/ros.h>
#include "ros_rdv/rdv.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <cstdlib>

// 디스펜서 테스트 파일
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rdv_serial_client");


    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ros_rdv::rdv>("rdv_serial");
    client.waitForExistence();

    ros_rdv::rdv srv;

    int num;
    std::cin >> num;

    if (num >=0 && num <= 4000) {
        srv.request.a = num;
    }
    else {
        return 0;
    }


    if (client.call(srv))
    {
	    ROS_INFO("String: %s", srv.response.str.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service rdv");
        return 1;
    }

    return 0;
}
