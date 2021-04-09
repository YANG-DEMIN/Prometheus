/***************************************************************************************************************************
 * circular.cpp
 *
 * Author: Ydm
 *
 * Update Time: 2020.8.14
 *
 * descripation: demostration of proportion guidance on sitl model
 *      1.
 *      2.
 *      3.
***************************************************************************************************************************/

#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <math_utils.h>
#include <Eigen/Eigen>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <nlink_parser/LinktrackNode2.h>		//target information from Ground station

//#include <circular.h>

using namespace std;

float desire_x = 0.0;				//desired altitude
float desire_y = 0.0;				//desired altitude
float desire_z = 5.0;				//desired altitude
float desire_target_x = 200.0;				//desired altitude
float desire_target_y = 100.0;				//desired altitude
float desire_target_z = 50.0;				//desired altitude

float desire_Radius = 10.0;		//desired radius of circle
float MoveTimeCnt = 0.0;
float priod = 2000.0;			//to change velocity of flying using it

//float DESIRE_V = 10;

Eigen::Vector3d P_T0 = {200, 100, 50};		//initial position of Target
Eigen::Vector3d P_M0 = {0, 0, 5};			//initial position of Missile
Eigen::Vector3d V_T = {-3, 0, 0};			//velocity of Target
Eigen::Vector3d V_M = {0, 0, 0};
float VM = 5;						//velocity of Missile
float VT = 3;


double sita = 0.0;
double psi_v = 0.0;
double phi = 0;
double psi = 0;
double gamma_vehicle = 0;

mavros_msgs::State current_state;
mavros_msgs::State target_state;
Eigen::Vector3d position_get;
Eigen::Vector3d attitude_get;
Eigen::Vector3d velocity_get;
Eigen::Vector3d att_rate_get;

Eigen::Vector3d pos_target; 	//desired value in offboard mode
Eigen::Vector3d pos_drone;	
Eigen::Vector3d temp_pos_drone;	
Eigen::Vector3d temp_pos_target;
Eigen::Vector3d velocity_sp;

mavros_msgs::SetMode mode_cmd;
ros::Publisher setpoint_raw_local_pub;
ros::ServiceClient set_mode_client;

enum
{
	WATTING,
	CHECKING,
	PREPARE,
	REST,
	FLY,
	FLYOVER,
}FlyState = WATTING;		//the initial state

Eigen::Matrix3d L;
Eigen::Vector3d P_r;
Eigen::Vector3d P_T = P_T0;
Eigen::Vector3d P_M = P_M0;
double x_t = 200.0;
double y_t = 100.0;
double z_t = 50.0;
double x_m = 0.0;
double y_m = 0.0;
double z_m = 5.0;
double v_t_x = -3.0;
double v_t_y = 0.0;
double v_t_z = 0.0;


//distance of target
double R_T1 = sqrt(x_t*x_t + y_t*y_t);
double R_T2 = sqrt(x_t*x_t + y_t*y_t + z_t*z_t);
//angle of velocity
double epsilon_T1 = atan(y_t / x_t);
double epsilon_T2 = atan(z_t / sqrt(x_t*x_t + y_t*y_t));
double epsilon_T1_last = epsilon_T1;
double epsilon_T2_last = epsilon_T2;
double d_epsilon_T1 = 0.0;
double d_epsilon_T2 = 0.0;
//angle between velocity and LOS
double eta_T1 = M_PI - atan(y_t / x_t);
double eta_T2 = M_PI - atan(z_t / sqrt(x_t*x_t + y_t*y_t));

double theta_T1 = 0.0;
double theta_T2 = 0.0;
//distance of missile
double R1 = sqrt(x_m*x_m + y_m*y_m);
double R2 = sqrt(x_m*x_m + y_m*y_m + z_m*z_m);
double dR1 = 0.0;
double dR2 = 0.0;
//angle of velocity
double epsilon_1 = atan(y_m / x_m);
double epsilon_2 = atan(z_m / sqrt(x_m*x_m + y_m*y_m));
double d_epsilon_1 = 0.0;
double d_epsilon_2 = 0.0;

double eta_1 = 0.0;
double eta_2 = 0.0;

double theta_1 = 0.0;
double theta_2 = 0.0;

double x_r = 1;
double y_r = 1;
double z_r = 1;
double x_r_last = 0;
double y_r_last = 0;
double z_r_last = 0;


//receive the current position of drone from controller

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

void target_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	//read the drone postion from the Mavros Package [Frame: ENU]
	Eigen::Vector3d pos_target_fcu_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

	P_T = pos_target_fcu_enu;
}

void target_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	//read the drone postion from the Mavros Package [Frame: ENU]
	Eigen::Vector3d velocity_target_fcu_enu(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);

	V_T = velocity_target_fcu_enu;
	VT = sqrt(V_T[0]*V_T[0] + V_T[1]*V_T[1] + V_T[2]*V_T[2]);
}

void Guidance_Update(void)
{
	//update the position
	P_M = position_get;
	//cout << "P_M = position_get = " << P_M <<endl; 		//yes, i get the right position
	P_T= P_T;
	x_t = P_T[0];
	y_t = P_T[1];
	z_t = P_T[2];

	x_m = P_M[0];
	y_m = P_M[1];
	z_m = P_M[2];

	v_t_x = V_T[0];
	v_t_y = V_T[1];
	v_t_z = V_T[2];

	P_r = P_T - P_M;
	x_r = P_r[0];
	y_r = P_r[1];
	z_r = P_r[2];

	
	//update the target information*** 1 *****
	R_T1 = sqrt(x_t * x_t + y_t * y_t);
	epsilon_T1 = atan(y_t / x_t);

	//此处不要除以时间
	d_epsilon_T1 = (epsilon_T1 - epsilon_T1_last);
	epsilon_T1_last = epsilon_T1;

	theta_T1 = M_PI + atan(v_t_y / v_t_x);
	eta_T1 = epsilon_T1 - theta_T1;

	d_epsilon_1 = d_epsilon_T1;
	epsilon_1 = epsilon_T1;

	eta_1 = asin(-R1 * d_epsilon_1 / VM);	//此处VM为恒定值5

	theta_1 = epsilon_1 - eta_1;

	//update the target information**** 2 ******
	R_T2 = sqrt(x_t * x_t + y_t * y_t + z_t*z_t);
	epsilon_T2 = atan(z_t / sqrt(x_t*x_t + y_t*y_t));

	
	d_epsilon_T2 = (epsilon_T2 - epsilon_T2_last);
	epsilon_T2_last = epsilon_T2;

	//此处要确定下符号 速度大小也需要重新获取
	theta_T2 = M_PI + asin(v_t_z / VT);
	eta_T2 = epsilon_T2 - theta_T2;

	d_epsilon_2 = d_epsilon_T2;
	epsilon_2 = epsilon_T2;

	//cout << "R2 = " << R2 <<endl;
	//cout << "d_epsilon_2 = " << d_epsilon_2 <<endl;
	eta_2 = asin(-R2 * d_epsilon_2 / VM);	//此处VM为恒定值5

	//cout << "eta_2 = " << eta_2 << endl;
	//cout << "R2 = " << R2 << endl;
	//cout << "epsilon_T2 = " << epsilon_T2 << endl;

	theta_2 = epsilon_2 - eta_2;	

	//cout << "psi_v = " << psi_v << endl;
	theta_1 = theta_1 * 1.35;
	theta_2 = theta_2 * 1.20;
	//vz 的theta2符号需要确认
	V_M << VM * cos(theta_2) * cos(theta_1), VM * cos(theta_2) * sin(theta_1), VM * sin(theta_2);
	cout << "theta_1 = " << theta_1 << endl;
	cout << "theta_2 = " << theta_2 << endl;

}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	//read the drone postion from the Mavros Package [Frame: ENU]
	Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

	pos_drone = pos_drone_fcu_enu;
	position_get = pos_drone;
	P_M = position_get;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	
    velocity_get[0] = msg->twist.linear.x;
    velocity_get[1] = msg->twist.linear.y;
	velocity_get[2] = msg->twist.linear.z;
}

void att_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
	Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
      //Transform the Quaternion to euler Angles
      Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);

      attitude_get = euler_fcu;
		
	att_rate_get[0] = msg->angular_velocity.x;
      att_rate_get[1] = msg->angular_velocity.y;
      att_rate_get[2] = msg->angular_velocity.z;
      
 }

//send the desired position to controller (xyz, yaw)
void send_vel_setpoint(const Eigen::Vector3d& vel_sp)
{
	mavros_msgs::PositionTarget pos_setpoint;
	pos_setpoint.type_mask = 0b110111000111;	//0b 100 111 000 111  velocity

	//Bitmask to indicate which dimensions should be ignored (1 means ignore, 0 means not ignore; Bit 10 must set to 0)
	//Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7: ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11: yaw, bit 12:yaw_rate
	//Bit 10 should set to 0, means no force sp

	pos_setpoint.coordinate_frame = 1;

	pos_setpoint.velocity.x = vel_sp[0];
	pos_setpoint.velocity.y = vel_sp[1];
	pos_setpoint.velocity.z = vel_sp[2];


	setpoint_raw_local_pub.publish(pos_setpoint);
}

//send the desired position to controller (xyz, yaw)
void send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp)
{
	mavros_msgs::PositionTarget pos_setpoint;
	pos_setpoint.type_mask = 0b100111111000;	//0b 100 111 111 000 xyz + yaw 

	//Bitmask to indicate which dimensions should be ignored (1 means ignore, 0 means not ignore; Bit 10 must set to 0)
	//Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7: ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11: yaw, bit 12:yaw_rate
	//Bit 10 should set to 0, means no force sp

	pos_setpoint.coordinate_frame = 1;

	pos_setpoint.position.x = pos_sp[0];
	pos_setpoint.position.y = pos_sp[1];
	pos_setpoint.position.z = pos_sp[2];

	pos_setpoint.yaw = yaw_sp;

	setpoint_raw_local_pub.publish(pos_setpoint);
}

void FlyState_update(void)
{
	switch(FlyState)
	{
		case WATTING:
			if(current_state.mode != "OFFBOARD")	//waitting for offboard mode
			{
				pos_target[0] = pos_drone[0];
				pos_target[1] = pos_drone[1];
				pos_target[2] = pos_drone[2];
				temp_pos_drone[0] = pos_drone[0];
				temp_pos_drone[1] = pos_drone[1];
				temp_pos_drone[2] = pos_drone[2];
				send_pos_setpoint(pos_target, 0);
			}
			else
			{
				pos_target[0] = temp_pos_drone[0];
				pos_target[1] = temp_pos_drone[1];
				pos_target[2] = temp_pos_drone[2];
				send_pos_setpoint(pos_target, 0);
				FlyState = CHECKING;
			}
			//cout << "WATTING" << endl;
			break;

		case CHECKING:
			if(pos_drone[0] == 0 && pos_drone[1] == 0)	//no position information, land
			{
				cout << "Check error, make sure have local location" <<endl;
				mode_cmd.request.custom_mode = "AUTO.LAND";
				set_mode_client.call(mode_cmd);
				FlyState = WATTING;
			}
			else
			{
				FlyState = PREPARE;
				MoveTimeCnt = 0;
			}
			//cout << "CHECKING" << endl;
			break;
		case PREPARE:									//fly to the first point located in x negative axis

			FlyState = REST;
			//cout << "PREPARE" << endl;
			break;
		case REST:
			pos_target[0] = desire_x;				//fly to 10 m
			pos_target[1] = desire_y;
			pos_target[2] = desire_z;
			send_pos_setpoint(pos_target, 0);
			MoveTimeCnt += 1;
			
			if(MoveTimeCnt >= 100 && abs(P_T[0] - desire_target_x) < 3 && abs(P_T[1] - desire_target_y) < 3 
				&& abs(P_T[2] - desire_target_z) < 3)
			//if(MoveTimeCnt >= 100 )
			{
				MoveTimeCnt = 0;
				FlyState = FLY;
			}
			if(current_state.mode != "OFFBOARD")			//if it is switched to "onboard" mode, jump to the "WATTING"
			{
				FlyState = WATTING;
			}
			//cout << "REST" << endl;
			break;
		case FLY:
			{

				Guidance_Update();
				velocity_sp = V_M;
				send_vel_setpoint(velocity_sp);
				//cout << "I am here!!! " << endl;
				if(current_state.mode != "OFFBOARD")			//if it is switched to "onboard" mode, jump to the "WATTING"
				{
					FlyState = WATTING;
				}
				if(abs(P_r[0]*P_r[0] + P_r[1]*P_r[1] + P_r[2]*P_r[2]) < 2)
				{
					cout << "I hit it !!!" << endl;
					FlyState = FLYOVER;
				}
			}
			//cout << "FLY" << endl;
			break;
		case FLYOVER:
			{
				mode_cmd.request.custom_mode = "AUTO.LAND";
				set_mode_client.call(mode_cmd);
				FlyState = WATTING;
			}
			//cout << "FLYOVER" << endl;
			break;
		default:
			cout << "ERROR!!!" << endl;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "circular_offboard");
	ros::NodeHandle nh;
	
   	ros::Subscriber target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 100, target_cb);	
	ros::Subscriber target_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav1/mavros/local_position/velocity_local", 100, target_velocity_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/uav0/mavros/state", 10, state_cb);			//handle return function	
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose", 100, pos_cb);
	ros::Subscriber attitude_sub = nh.subscribe<sensor_msgs::Imu>("/uav0/mavros/imu/data", 10, att_cb);

	setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav0/mavros/setpoint_raw/local", 10);

	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");			//little question

	cout << "guidance node started!!!" << endl; 
	ros::Rate rate(20.0);
	while(ros::ok())
	{
		
		FlyState_update();
		ros::spinOnce();
		rate.sleep();

		cout << "P_T=" << "\t" << "\t" << "P_M="<< "\t"<< endl;
		cout<< "\t" << P_T[0] << "\t" << P_M[0] << endl;
		cout<< "\t" << P_T[1] << "\t" << P_M[1] << endl;
		cout<< "\t" << P_T[2] << "\t" << P_M[2] << endl;

		//cout << "theta_1 = " << theta_1 << endl;
		//cout << "theta_2 = " << theta_2 << endl;
		//cout << "V_M = " << endl << V_M << endl;

	}
	return 0;
}





