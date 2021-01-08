#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rdv_picknplace");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  double joint0 = atof(argv[1]);
  double joint1 = atof(argv[2]);
  double joint2 = atof(argv[3]);
  double joint3 = atof(argv[4]);
  double joint4 = atof(argv[5]);
  double joint5 = atof(argv[6]);

  static const std::string PLANNING_GROUP = "indy7";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  robot_state::RobotState current_state = *move_group.getCurrentState();             
  std::vector<double> joint_positions;
  joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
  current_state.copyJointGroupPositions(joint_model_group, joint_positions);
  
  
  joint_positions[0] = joint0;
  joint_positions[1] = joint1;
  joint_positions[2] = joint2;
  joint_positions[3] = joint3;
  joint_positions[4] = joint4;
  joint_positions[5] = joint5;
  move_group.setJointValueTarget(joint_positions);
          
            //움직이라고 명령 
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(!success)
      throw std::runtime_error("No plan found");
  
  move_group.move();
  return 0;
}
