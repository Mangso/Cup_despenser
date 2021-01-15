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
    // std::copy( joint_goal.begin(), joint_goal.end(), joint_positions.begin());
    // move_group.setJointValueTarget(joint_positions);

    move_group.setJointValueTarget(joint_goal);
    
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

void RdvCupNode::go_on_Dispenser()
{
    ros::ServiceClient dispenser_;
    dispenser_ = nh_.serviceClient<ros_rdv::rdv>("rdv_serial");
    ros_rdv::rdv srv;
    srv.request.a = 3800;
    dispenser_.call(srv);
}

void RdvCupNode::go_off_Dispenser()
{
    ros::ServiceClient dispenser_;
    dispenser_ = nh_.serviceClient<ros_rdv::rdv>("rdv_serial");
    ros_rdv::rdv srv2;
    srv2.request.a = 2000;
    dispenser_.call(srv2);
}

// 첫 번째 Init 자세
void RdvCupNode::jmove_pickup_init_pos()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.3877049283381902, -0.8175309407073752, -1.7243005798736448, 1.240622150131187, 1.3587633511823527, -2.110715797327151 };
    goToJointState(joint_goal);

}

// Gripper로 컵 집기전에 자세
void RdvCupNode::jmove_pickup_hold_pos()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.3763862736093948, -1.2119113794473926, -1.5753713294134157, 1.2124143697519711, 1.4471048870886565, -1.8783651702393136 };
    goToJointState(joint_goal);

}

void RdvCupNode::jmove_pickup_hold_up_pos()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.3877049283381902, -0.8175309407073752, -1.7243005798736448, 1.240622150131187, 1.3587633511823527, -2.110715797327151 };
    goToJointState(joint_goal);

}

// 컵 드랍하기 전 자세.
void RdvCupNode::jmove_pickup_drop_pos()
{
    std::vector<double> joint_goal(6);

    joint_goal = {0.051542071411140256, -0.9379650718108202, -1.6963941731849383, 1.2308496673271747, 1.3881130739426493, -2.0260279074929834};
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






