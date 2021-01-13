#include </home/somag/catkin_ws/src/ros_rdv/src/include/rdv_core.h>

RdvCupNode::RdvCupNode():move_group(PLANNING_GROUP)
{
    initForROS();
}
void RdvCupNode::initForROS()
{
    gripper_pub = nh_.advertise<std_msgs::Int32MultiArray>("/robotis/pos",1);
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
    // std::cout << msg << '\n';
    std_msgs::Int32MultiArray pos;
    pos.data.clear();
    pos.data.push_back(msg);
    // std::cout << pos << '\n';
    gripper_pub.publish(pos);

    // ros::Duration(1.0).sleep();
}

void RdvCupNode::goInDispenser()
{
    ros::ServiceClient dispenser_;
    dispenser_ = nh_.serviceClient<ros_rdv::rdv>("rdv_serial");
    ros_rdv::rdv srv;
    srv.request.a = 3990;
    dispenser_.call(srv);
}

void RdvCupNode::goOutDispenser()
{
    ros::ServiceClient dispenser_;
    dispenser_ = nh_.serviceClient<ros_rdv::rdv>("rdv_serial");
    ros_rdv::rdv srv2;
    srv2.request.a = 300;
    dispenser_.call(srv2);
}

void RdvCupNode::step1()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.4988151002199794, -0.6321582550723461, -1.655619328441821, 1.2583823906879115, 1.1562806294462433, 0.7826056365942574};
    goToJointState(joint_goal);

}

void RdvCupNode::step2()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.40701078156507764, -1.0822786691616837, -1.618618348299541, 1.2103858362580675, 1.3465215179136254, 1.168672467135403};
    goToJointState(joint_goal);

}

void RdvCupNode::step3()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.4719370297392667, -0.5862560957448953, -1.687907919603716, 1.2864821916450202, 1.166752604958209, 0.7538077039363509};
    goToJointState(joint_goal);

}

void RdvCupNode::step4()
{
    std::vector<double> joint_goal(6);

    joint_goal = {0.08325220532012952, -0.5876523591464908, -1.690351380556508, 1.28962378429861, 1.16343647937942, 0.7716100623066932};
    goToJointState(joint_goal);

}

void RdvCupNode::step5()
{
    std::vector<double> joint_goal(6);

    joint_goal = {0.37437312455278365, -0.7867944267990438, -1.73293741430517, 0.5295328950550796, 0.9892526200303859, 1.2952088379049917};
    goToJointState(joint_goal);

}






