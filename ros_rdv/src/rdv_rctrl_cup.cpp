#include "rdv_rctrl_cup.hpp"
#include "rdv_rctrl_juicer.hpp"
//#define	TCP_MOVE

std::vector<double> rdv_rctrl_cup::m_joint_pickcup_init = { -0.391565,-0.898583,-1.68416,1.24128,1.37323,-2.09087};

rdv_rctrl_cup* rdv_rctrl_cup::instance = nullptr;

rdv_rctrl_cup* rdv_rctrl_cup::GetInstance()
{
	if( instance == nullptr )
	{
		instance = new rdv_rctrl_cup();
	}		
	return instance;
}
	
rdv_rctrl_cup::rdv_rctrl_cup()
{
	m_str_Link_QR_Code = "lc_cup_qr";
}


rdv_rctrl_cup::~rdv_rctrl_cup()
{
}

int rdv_rctrl_cup::run_pick_cup_ready()
{
	jmove_pickcup_initial_pos();
}


int rdv_rctrl_cup::run_pick_cup_init()
{
	RDV_Manipulator_ros ros_api;
	ros_api.ros_api_setVelocityMaxScaling(0);    
	jmove_pickcup_initial_pos();
	set_gripper_pos(255);	
	jmove(-0.38217911118182907, -1.2281512917555053, -1.5429144828151904, 1.2207867153888228, 1.4404820009766026, -1.9118241769295343);
	jmove_pickcup_initial_pos();
}

int rdv_rctrl_cup::run_pick_new_cup()
{
    RDV_Manipulator_ros ros_api;
    ros_api.ros_robot_set_joint_constraints("joint0", -0.4, 0.1, 2);

	jmove_pickcup_initial_pos();
    set_gripper_pos(0);
	jmove_pickcup_tophold_pos();
    set_gripper_pos(235);
    usleep(1000*1000*1.5);	
	jmove_pickcup_tophold_up_pos();
	set_gripper_pos(0);	
	jmove_pickcup_hold_pos();
    set_gripper_pos(180);
	usleep(1000*1000*1.5);	

#if 0
	jmove(-0.39156523536224974, -0.8909837408000261, -1.7027337291166913, 1.2414546384671867, 1.3415383416668982, -2.0982607466354124);
	jmove(-0.33178911005249967, -0.8275953125915305, -1.3078533203988043, 1.3920296610285505, 1.3002632472245417, -2.548015180584571);
	set_gripper_pos(0);
#else	
	jmove_pickcup_initial_pos();
#endif
	ros_api.ros_robot_clear_joint_constraints();

#if 1
	Node_Handler::GetInstance()->m_move_group->setMaxVelocityScalingFactor(0.7);
    ros_api.ros_api_execute_plan(m_plan_traject.cup_to_juicer);
#else
	ros_api.ros_robot_joint_move(rdv_rctrl_juicer::m_joint_juicer_calib);	
#endif
	
}

int rdv_rctrl_cup::jmove_pickcup_initial_pos()
{
#ifndef TCP_MOVE
	RDV_Manipulator_ros rdv_ros_api;

	std::vector<double> joint_pos_for_cup_pick = rdv_rctrl_cup::m_joint_pickcup_init;
    	rdv_ros_api.ros_robot_joint_move(joint_pos_for_cup_pick);
#else
	
	RDV_Manipulator_ros rdv_ros_api;	
	geometry_msgs::Quaternion orientation;
	geometry_msgs::Pose position;

	orientation.w=0.696608;
	orientation.x=0.717162;
	orientation.y=-0.018244;
	orientation.z=-0.009081;

	position.position.x = -0.213585;
	position.position.y = -0.473608;
	position.position.z = 0.899218;

	position.orientation = orientation;

	rdv_ros_api.ros_robot_position_move(position);

#endif
	return 0;
}

int rdv_rctrl_cup::jmove_pickcup_tophold_pos()
{
	jmove(-0.3871867344150098, -1.1022436382952863, -1.6139942082902186, 1.2301102050538066, 1.405831691137374, -1.9886570599445468);
}

int rdv_rctrl_cup::jmove_pickcup_tophold_up_pos()
{	
	jmove(-0.38681591674521093, -1.071543421555916, -1.6347108723811592, 1.2342365754033977, 1.3927529861277888, -2.01096433006541);
}

int rdv_rctrl_cup::jmove_pickcup_hold_pos()
{
	jmove(-0.3798179217459306, -1.277356736439644, -1.5299580500447836, 1.2160490309133662, 1.4174580504752605, -1.8313129674584225);
}


int rdv_rctrl_cup::run_calibration_pos()
{

}

int rdv_rctrl_cup::prepare_nonstop_movement()
{
	std::vector<double> start_joint = rdv_rctrl_cup::m_joint_pickcup_init;
	std::vector<double> goal_joint = rdv_rctrl_juicer::m_joint_juicer_calib;
	RDV_Manipulator_ros ros_api;

	ros_api.ros_api_planning_make_trajectory(start_joint, goal_joint, m_plan_traject.cup_to_juicer );
	usleep(1000*1000*1);
}

