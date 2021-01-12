#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "ros_rdv/rdv.h"

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rdv_picknplace");
  ros::NodeHandle node_handle;

  ros::ServiceClient client = n.serviceClient<ros_rdv::rdv>("rdv_serial");
  client.waitForExistence();

  ros_rdv::rdv srv;


  srv.request.a()
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "indy7";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  double data[5][6] = {
    {-0.3611086222376268, -0.8412486994612669, -1.772032789549843, 1.2960815025309893, 1.3639748104335687, -2.063153708782497},
    {-0.4445353604829557, -1.125911900461542, -1.5700981950940986, 1.2007865253720986, 1.3550736312483975, -1.9805996351631652},
    {-0.45919612619970807, -0.8293804605477054, -1.6674875673553826, 1.2362167091875835, 1.2791518087866443, -2.1715386553313447}, 
    {-0.13770647798235258, -0.8157668923821496, -1.6913985781077048, 1.0732029570513133, 1.2056734472776829, -1.048942880448592},
    {0.37437312455278365, -0.7867944267990438, -1.73293741430517, 0.5295328950550796, 0.9892526200303859, 1.2952088379049917}
  }

  for(int i=0 ; i<10 ; i ++){
    robot_state::RobotState current_state = *move_group.getCurrentState();             
    std::vector<double> joint_positions;
    joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
    current_state.copyJointGroupPositions(joint_model_group, joint_positions);
    
    
    copy(data[i], data[i] + sizeof(double), joint_positions.begin());

    move_group.setJointValueTarget(joint_positions);
            
              //움직이라고 명령 
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success)
        throw std::runtime_error("No plan found");
    
    move_group.move();
  }
  return 0;
}
