#include </home/somag/catkin_ws/src/ros_rdv/src/include/rdv_core.h>

RdvCupNode::RdvCupNode() : move_group(PLANNING_GROUP)
{
    initForROS();
}

void RdvCupNode::initForROS()
{
    gripper_pub = nh_.advertise<std_msgs::Int32MultiArray>("/robotis/pos", 1);
    restore_pub = nh_.advertise<std_msgs::Int32>("/indy/restore", 10);
    robot_sub = nh_.subscribe("/indy/status", 1, &RdvCupNode::robot_state_cb, this);
}

void RdvCupNode::robot_state_cb(const std_msgs::Int32::ConstPtr &msg)
{

    // ROS_INFO("I heard [%d]", msg->data);

    if (msg->data == 3)
    {
        ROS_INFO("I heard [%d]", msg->data);
    }

    robot_state = msg->data;
}

void RdvCupNode::restore_state_pub(uint32_t msg)
{
    std_msgs::Int32 restore_msg;
    restore_msg.data = msg;

    ROS_INFO("restore_msg : %d", restore_msg.data);
    restore_pub.publish(restore_msg);
}

void RdvCupNode::goToJointState(const std::vector<double> &joint_goal)
{

    robot_state::RobotState current_state = *move_group.getCurrentState();
    std::vector<double> joint_positions;
    joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
    current_state.copyJointGroupPositions(joint_model_group, joint_positions);

    move_group.setJointValueTarget(joint_goal);

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
        throw std::runtime_error("No plan found");


    move_group.asyncMove();
    
}

void RdvCupNode::goNearTrajectory()
{
    std::vector<double> joints_temp = move_group.getCurrentJointValues();
    std::vector<double> traject_temp;
    std::vector<double> go_traject;
    std::vector<std::vector<double>> traj_2;

    moveit_msgs::RobotTrajectory msg;
    msg = my_plan.trajectory_;

    std::cout << msg.joint_trajectory.points.size();

    for (int i = 0; i < msg.joint_trajectory.points.size(); i++)
    {
        std::cout << i << '\n';
        double sum = 0;
        for (int j = 0; j < 6; j++)
        {
            sum += joints_temp[j] - msg.joint_trajectory.points[i].positions[j];
            std::cout << joints_temp[j] - msg.joint_trajectory.points[i].positions[j] << '\n';
        }
        std::cout << '\n';
        traject_temp.push_back(abs(sum));
        std::cout << sum<<'\n';
        std::cout << '\n';
    }

    std::cout << *min_element(traject_temp.begin(),traject_temp.end()) << '\n';

    // 최솟값의 인덱스
    int min_idx = min_element(traject_temp.begin(),traject_temp.end()) - traject_temp.begin();
    
    std::cout << min_idx << "\n";

    for(int i = 0 ; i <min_idx;i++){
        std::vector<double> tmp;
        for(int j=0; j< 6;j++)
        {
            tmp.push_back(msg.joint_trajectory.points[i].positions[j]);
        }
        traj_2.push_back(tmp);
    }

    std::cout << min_idx - 1<< "이하의 숫자를 입력하세요. : ";
    int a;
    std::cin >> a;

    for(int i = min_idx -1 ; i >= a; i--){
        move_group.setJointValueTarget(traj_2[i]);
        move_group.move();
        // ros::Duration(3).sleep();
    }

// 가장 가까운 Trajectory로 가기.
#if 0
    for(int i=0; i< 6;i++)
    {
        go_traject.push_back(msg.joint_trajectory.points[min_idx].positions[i]);
    }

    for (auto elem : go_traject)
        std::cout << elem << " ";

    ros::Duration(3).sleep();

    std::cout << "********Go to near Trajectory Target**********" << '\n';
    move_group.setJointValueTarget(go_traject);
    move_group.asyncMove();

#endif
    
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
    srv.request.a = 3750;
    dispenser_.call(srv);
}

void RdvCupNode::go_off_Dispenser()
{
    ros::ServiceClient dispenser_;
    dispenser_ = nh_.serviceClient<ros_rdv::rdv>("rdv_serial");
    ros_rdv::rdv srv2;
    srv2.request.a = 1880;
    dispenser_.call(srv2);
}

// 첫 번째 Init 자세

void RdvCupNode::jmove_pickup_init_pos()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.3877049283381902, -0.8175309407073752, -1.7243005798736448, 1.240622150131187, 1.3587633511823527, -2.110715797327151};
    goToJointState(joint_goal);
}

// Gripper로 컵 집기전에 자세
void RdvCupNode::jmove_pickup_hold_pos()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.3763862736093948, -1.2119113794473926, -1.5753713294134157, 1.2124143697519711, 1.4471048870886565, -1.8783651702393136};
    goToJointState(joint_goal);
}

// 컵을 집은 후 올라오는 자세.
void RdvCupNode::jmove_pickup_hold_up_pos()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.3877049283381902, -0.8175309407073752, -1.7243005798736448, 1.240622150131187, 1.3587633511823527, -2.110715797327151};
    goToJointState(joint_goal);
}

// 컵 드랍하기 전 자세.
void RdvCupNode::jmove_pickup_drop_pos()
{
    std::vector<double> joint_goal(6);

    joint_goal = {0.051542071411140256, -0.9379650718108202, -1.6963941731849383, 1.2308496673271747, 1.3881130739426493, -2.0260279074929834};
    goToJointState(joint_goal);
}

void RdvCupNode::jmove_pickup_rotate_pos()
{
    std::vector<double> joint_goal(6);

    joint_goal = {0.20717773374568005, -0.9341206116957262, -1.9314569202922214, 0.230729315710542, 1.236988438215329, 1.4860391420366277};
    goToJointState(joint_goal);
}

// Test용 자세.
void RdvCupNode::step5()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.03804444155529365, -0.12248827774201934, -0.1592239941769443, 0.12254124641848704, 0.14625972721967107, -0.18984983099339697};
    goToJointState(joint_goal);
}
/*
void RdvCupNode::go_home()
{
    std::vector<double> joint_goal(6);

    joint_goal = {0.37437312455278365, -0.7867944267990438, -1.73293741430517, 0.5295328950550796, 0.9892526200303859, 1.2952088379049917};
    goToJointState(joint_goal);
}
*/
void RdvCupNode::run()
{

    ros::AsyncSpinner spinner(2);
    spinner.start();

    jmove_pickup_init_pos();
    ros::Duration(6).sleep();
    move_group.stop();

    goNearTrajectory();

    ros::spinOnce();
    spinner.stop();

    /*
    충돌메시지 3 받으면.
    리셋하고, 로봇
    */

#if 0
    while (ros::ok())
    {
        if (robot_state == 3)
        {
            // move_group.stop();
            char input;
            std::cin >> input;

            if(input == 'r'){
                restore_state_pub(99);
            }
            else if (input == 'q')
            {
                break;
            }

        }
        else{
            char tmp;
            std::cin >> tmp;

            if(tmp == 's')
            {
                jmove_pickup_init_pos();
            }
            else if(tmp=='q')
            {
                break;
            }
        }
    }

     // if (robot_state == 3)
    // {
    //     restore_state_pub(99);
    // }
    // // 자세 시작.
    
    // goToJointState({0.0, -0.2619739207243489, -1.5707963267948966, 0.0, -1.3089969389957472, 0.0});
    
        //jmove_pickup_init_pos();

#endif

#if 0
    ros::Duration(2).sleep();
    move_group.stop();

    ros::Duration(3).sleep();
    goNearTrajectory();

    컵 집으러 자세 낮춤.

    ros::Duration(2).sleep();
    jmove_pickup_init_pos();

    jmove_pickup_init_pos();

    // 그리퍼로 컵 집음.
    while (ros::ok())
    {
        goToGripperState(200);
        // ros::spinOnce();
        // loop_rate.sleep();
        ros::Duration(1.6).sleep();
        break;
    }

    // 디스펜서로 컵 품.
    go_off_Dispenser();
    // ros::Duration(1).sleep();

    // 컵 집고 자세 올라감.
    jmove_pickup_hold_up_pos();

    // 디스펜서로 컵 고정.
    go_on_Dispenser();
    // ros::Duration(2).sleep();

    // 드랍자세로 옮김.
    jmove_pickup_rotate_pos();

    // 그리퍼 품.
    while (ros::ok())
    {
        goToGripperState(0);
        // ros::spinOnce();
        // loop_rate.sleep();
        ros::Duration(1.6).sleep();
        break;
    }
# endif

}
