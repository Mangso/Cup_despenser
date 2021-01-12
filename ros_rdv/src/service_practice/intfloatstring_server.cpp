#include "ros/ros.h"
#include "ros_rdv/intfloatstring.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <cstdlib>

bool addnstr(ros_rdv::intfloatstring::Request &req, ros_rdv::intfloatstring::Response &res)
{
    res.sum = req.a + req.b;
    res.str = req.c + "aaa";
    ROS_INFO("request: x=%ld, y=%f, z=%s", (long int)req.a, (float)req.b, req.c.c_str());
    ROS_INFO("sending back response: [%f], [%s]", (float)res.sum, res.str.c_str());

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "intfloatstring_server");

    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("intfloatstring", addnstr);

    ROS_INFO("Ready to add (int+float), string");
    ros::spin();
    return 0;
}
