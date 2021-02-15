#include </home/somag/catkin_ws/src/ros_rdv/src/include/rdv_core.h>

RdvCupNode::RdvCupNode() : move_group(PLANNING_GROUP)
{
    initForROS();
}

void RdvCupNode::initForROS()
{
    // publish
    gripper_pub = nh_.advertise<std_msgs::Int32MultiArray>("/robotis/pos", 1);
    restore_pub = nh_.advertise<std_msgs::Int32>("/indy/restore", 1);
    blend_pub = nh_.advertise<std_msgs::Int32>("/indy/blend",10);

    // subscribe
    robot_sub = nh_.subscribe("/indy/status", 10, &RdvCupNode::robot_state_cb, this);
}

// 로봇 상태 받아 오는 함수 (충돌메시지만 뜨게 함.)
void RdvCupNode::robot_state_cb(const std_msgs::Int32::ConstPtr &msg)
{
    if (msg->data == 3)
    {
        ROS_INFO("I heard [%d]", msg->data);
    }

    robot_state = msg->data;
}

// 로봇 복구 메시지 보내는 함수
void RdvCupNode::restore_state_pub(uint32_t msg)
{
    std_msgs::Int32 restore_msg;
    restore_msg.data = msg;

    ROS_INFO("restore_msg : %d", restore_msg.data);
    restore_pub.publish(restore_msg);
}

void RdvCupNode::blend_state_pub(uint32_t msg)
{
    std_msgs::Int32 blend_msg;
    blend_msg.data = msg;
    ROS_INFO("blend_msg : %d", blend_msg.data);
    blend_pub.publish(blend_msg);
}

// 지정한 좌표로 움직이게 하는 함수.
void RdvCupNode::goToJointState(const std::vector<double> &joint_goal)
{
    // robot_state::RobotState current_state = *move_group.getCurrentState();
    // std::vector<double> joint_positions;
    // joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
    // current_state.copyJointGroupPositions(joint_model_group, joint_positions);

    move_group.setJointValueTarget(joint_goal);

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
        throw std::runtime_error("No plan found");

    move_group.asyncMove();
}

// 충돌시 원하는 Trajectory로 역순 이동해주는 함수.
void RdvCupNode::goNearTrajectory()
{
    //현재 좌표 얻음.
    std::vector<double> joints_temp = move_group.getCurrentJointValues();
    std::vector<double> traject_temp;

    moveit_msgs::RobotTrajectory traj_msg;
    traj_msg = my_plan.trajectory_;
    
    for (int i = 0; i < traj_msg.joint_trajectory.points.size(); i++)
    {
        double sum = 0;
        for (int j = 0; j < 6; j++)
        {
            sum += abs(joints_temp[j] - traj_msg.joint_trajectory.points[i].positions[j]);
            std::cout << joints_temp[j] - traj_msg.joint_trajectory.points[i].positions[j] << '\n';
        }
        traject_temp.push_back(abs(sum));
    }

    // 최솟값의 인덱스
    int min_idx = min_element(traject_temp.begin(), traject_temp.end()) - traject_temp.begin();

    std::cout << min_idx << "\n";
    std::cout << min_idx - 1 << "이하의 숫자를 입력하세요. : ";


    int traj_idx;
    std::cin >> traj_idx;

    // 지정한 Trajectory로 가기 plan 만듬.
    moveit::planning_interface::MoveGroupInterface::Plan next_plan;
    std::cout << "******************************************************" << '\n';
    
    // 로봇 정보 얻기 위해 복사하고, Trajectory 초기화.
    next_plan = my_plan;
    next_plan.trajectory_.joint_trajectory.points.clear();

    // 내 현재좌표를 Trajectory 0번으로
    next_plan.trajectory_.joint_trajectory.points.push_back(traj_msg.joint_trajectory.points[min_idx-1]);
    for(int i=0; i< 6; i++)
        next_plan.trajectory_.joint_trajectory.points[0].positions[i] = joints_temp[i];

    // Trajectory plan에 쌓음.
    for (int i = min_idx-1; i >= traj_idx; i--)
        next_plan.trajectory_.joint_trajectory.points.push_back(traj_msg.joint_trajectory.points[i]);
    

    move_group.execute(next_plan);
}

// Gripper 강도 pub 하는 함수
void RdvCupNode::goToGripperState(int msg)
{
    std_msgs::Int32MultiArray pos;
    pos.data.clear();
    pos.data.push_back(msg);
    gripper_pub.publish(pos);

}

// Dispenser on. (Cup 고정)
void RdvCupNode::go_on_Dispenser()
{
    ros::ServiceClient dispenser_;
    dispenser_ = nh_.serviceClient<ros_rdv::rdv>("rdv_serial");
    ros_rdv::rdv srv;
    srv.request.a = 3750;
    dispenser_.call(srv);
}

// Dispense off. (Cup 해제)
void RdvCupNode::go_off_Dispenser()
{
    ros::ServiceClient dispenser_;
    dispenser_ = nh_.serviceClient<ros_rdv::rdv>("rdv_serial");
    ros_rdv::rdv srv2;
    srv2.request.a = 1880;
    dispenser_.call(srv2);
}

// 첫 번째 Init 자세
void RdvCupNode::jmove_pickup_init_pos(uint32_t msg)
{
    blend_state_pub(msg);

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
void RdvCupNode::test_step()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.03804444155529365, -0.12248827774201934, -0.1592239941769443, 0.12254124641848704, 0.14625972721967107, -0.18984983099339697};
    goToJointState(joint_goal);
}

void RdvCupNode::go_home()
{
    std::vector<double> joint_goal(6);

    joint_goal = {-0.00017431599862337982, -0.2619359353314217, -1.5709175556485884, -7.024416974229265e-05, -1.308908975576066, 1.8035665204102172e-05};
    goToJointState(joint_goal);
}

void RdvCupNode::run()
{



    // jmove_pickup_init_pos();
    // ros::Duration(10).sleep();
    // move_group.stop();

    // goNearTrajectory();

    // ros::spinOnce();
    // spinner.stop();

   
#if 1 // 충돌하기 위해 만든 테스트 함수.

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        if (robot_state == 3)
        {
            // move_group.stop();
            char input;
            std::cin >> input;

            if(input == 'r'){
                restore_state_pub(99);
                goNearTrajectory();
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
                jmove_pickup_init_pos(10);
            }
            else if(tmp=='q')
            {
                break;
            }
            else if (tmp == 'a')
            {
                goNearTrajectory();
            }
            else if(tmp == 'h')
            {
                go_home();
            }
        }
    }

    ros::spinOnce();
    spinner.stop();

#endif

// Cup pick n place 예제
#if 0

    ros::AsyncSpinner spinner(1);
    spinner.start();


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
#endif
}
