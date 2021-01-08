#include <ros/ros.h>
#include "ros_rdv/intfloatstring.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "intfloatstring_client");

    if (argc != 4)
    {
        ROS_INFO("usage: add_two_ints_client X Y Z");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ros_rdv::intfloatstring>("intfloatstring");
    client.waitForExistence();

    ros_rdv::intfloatstring srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atof(argv[2]);
    srv.request.c = argv[3];

    if (client.call(srv))
    {
        ROS_INFO("Sum: %f", (float)srv.response.sum);
	    ROS_INFO("String: %s", srv.response.str.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service intfloatstring");
        return 1;
    }

    return 0;
}
