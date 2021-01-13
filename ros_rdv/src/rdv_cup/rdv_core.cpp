#include </home/somag/catkin_ws/src/ros_rdv/src/include/rdv_core.h>

RdvCupNode::RdvCupNode():move_group(PLANNING_GROUP)
{
    initForROS();
}
void RdvCupNode::initForROS()
{
    gripper_pub = nh_.advertise<std_msgs::Int32MultiArray>("/robotis/pos",1);
    dispenser_ = nh_.serviceClient<ros_rdv::rdv>("rdv_serial");
}
void RdvCupNode::goToJointState(const std::vector<double>& joint_goal)
{
    
    robot_state::RobotState current_state = *move_group.getCurrentState();             
    std::vector<double> joint_positions;
    joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
    current_state.copyJointGroupPositions(joint_model_group, joint_positions);

    //joint를 어떻게 움직일지 move_group에 지시
    std::copy( joint_goal.begin(), joint_goal.end(), joint_positions.begin());
    move_group.setJointValueTarget(joint_positions);

    // move_group.setJointValueTarget(joint_goal);
    
    //움직이라고 명령 
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success)
        throw std::runtime_error("No plan found");

    //경로를 다움직일때 까지 코드 여기서 정지   
    move_group.move(); //blocking
}

void RdvCupNode::goToGripperState(int msg)
{
    std_msgs::Int32MultiArray pos;
    pos.data[0] = msg; 
    gripper_pub.publish(pos);
}

void RdvCupNode::goToDispenser(ros_rdv::rdv srv,int num)
{
    srv.request.a = num;
    dispenser_.call(srv);
}

void RdvCupNode::step1()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.3611086222376268, -0.8412486994612669, -1.772032789549843, 1.2960815025309893, 1.3639748104335687, -2.063153708782497};
    goToJointState(joint_goal);

}

void RdvCupNode::step2()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.4445353604829557, -1.125911900461542, -1.5700981950940986, 1.2007865253720986, 1.3550736312483975, -1.9805996351631652};
    goToJointState(joint_goal);

}

void RdvCupNode::step3()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.45919612619970807, -0.8293804605477054, -1.6674875673553826, 1.2362167091875835, 1.2791518087866443, -2.1715386553313447};
    goToJointState(joint_goal);

}

void RdvCupNode::step4()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.13770647798235258, -0.8157668923821496, -1.6913985781077048, 1.0732029570513133, 1.2056734472776829, -1.048942880448592};
    goToJointState(joint_goal);

}

void RdvCupNode::step5()
{
    std::vector<double> joint_goal(6);

    joint_goal = {0.37437312455278365, -0.7867944267990438, -1.73293741430517, 0.5295328950550796, 0.9892526200303859, 1.2952088379049917};
    goToJointState(joint_goal);

}






