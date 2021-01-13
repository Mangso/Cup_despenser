#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ros/ros.h>
#include "ros_rdv/rdv.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32MultiArray.h>
#include <array>
#include <string>
#include <cstdlib>


class RdvCupNode{

    public:
        RdvCupNode();
        ~RdvCupNode();

        void initForROS();
        void goToJointState(const std::vector<double>& joint_goal);
        void goToGripperState(int msg);
        void goToDispenser(int num);

        void step1();
        void step2();
        void step3();
        void step4();
        void step5();



        ros::NodeHandle nh_;
        ros::Publisher gripper_pub;

        // dipenser ctrl
        ros::ServiceClient dispenser_;
        
        ros_rdv::rdv srv;

        const std::string PLANNING_GROUP = "indy7";

        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        const robot_state::JointModelGroup* joint_model_group;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    private:
};