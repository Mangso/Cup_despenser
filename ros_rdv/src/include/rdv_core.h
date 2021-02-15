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
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <algorithm>
#include <vector>
#include <string>
#include <cstdlib>

class RdvCupNode{

    public:

        RdvCupNode();

        void initForROS();
        void goToJointState(const std::vector<double>& joint_goal);
        
        //Gripper
        void goToGripperState(int msg);

        // Dispensor
        void go_on_Dispenser();
        void go_off_Dispenser();

        //Robot
        void restore_state_pub(uint32_t msg);
        void blend_state_pub(uint32_t msg);

        void robot_state_cb(const std_msgs::Int32::ConstPtr &msg);

        void goNearTrajectory();

        // joint_move pos
        void jmove_pickup_init_pos(uint32_t msg);
        void jmove_pickup_hold_pos();
        void jmove_pickup_hold_up_pos();
        void jmove_pickup_drop_pos();
        void jmove_pickup_rotate_pos();
        void go_home();

        void run();

        void test_step();

        ros::Subscriber robot_sub;

    private:

        ros::NodeHandle nh_;
        ros::Publisher gripper_pub;
        ros::Publisher restore_pub;
        ros::Publisher blend_pub;

        uint32_t robot_state;

        const std::string PLANNING_GROUP = "indy7";

        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        const robot_state::JointModelGroup* joint_model_group;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

};