/***************************************************************************************************************************
 * circular.cpp
 *
 * Author: Ydm
 *
 * Update Time: 2020.8.14
 *
 * descripation: demostration of circular flying using mavros
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

//#include <circular.h>

using namespace std;

float desire_z = 10.0;				//desired altitude
float desire_Radius = 10.0;		//desired radius of circle
float MoveTimeCnt = 0.0;
float priod = 2000.0;			//to change velocity of flying using it

float DESIRE_V = 10;

Eigen::Vector3d P_T0 = {200, 100, 50};		//initial position of Target
Eigen::Vector3d P_M0 = {0, 0, 5};			//initial position of Missile
Eigen::Vector3d V_T = {-3, 0, 0};			//velocity of Target
Eigen::Vector3d V_M = {0, 0, 0};
float VM = 5;						//velocity of Missile

float K1 = 4;						//proportion of guidance
float K2 = 4;

double sita = 0.0;
double psi_v = 0.0;
double phi = 0;
double psi = 0;
double gamma_vehicle = 0;

mavros_msgs::State current_state;
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
double x_r = 1;
double y_r = 1;
double z_r = 1;
double x_r_last = 0;
double y_r_last = 0;
double z_r_last = 0;
double q_lambda = 0;
double q_gamma = 0;
double q_lambda_last = 0;
double q_gamma_last= 0;
double q_lambda_delta = 0;
double q_gamma_delta = 0;
double delta_sita = 0.0;
double delta_psi_v = 0.0;

//receive the current position of drone from controller

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;

	cout << "P_T = " << endl << P_T << endl;
	cout << "P_M = " << endl << P_M << endl;
	cout << "sita = " << sita << endl;
	cout << "psi_v = " << psi_v << endl;
	cout << "V_M = " << endl << V_M << endl;
}

void Guidance_Update(void)
{
	//update the position
	P_M = position_get;
	//cout << "P_M = position_get = " << P_M <<endl; 		//yes, i get the right position
	P_T = P_T + V_T * 0.05;
	
	//update the attitude
	phi = attitude_get[0];
	gamma_vehicle = attitude_get[1];
	psi = attitude_get[2];

	L << cos(phi) * cos(psi), -sin(phi) * cos(psi) * cos(gamma_vehicle) + sin(psi) * sin(gamma_vehicle), sin(phi) * cos(psi) * sin(gamma_vehicle) + sin(psi) * cos(gamma_vehicle),
    sin(phi), cos(phi) * cos(gamma_vehicle), -cos(phi) * sin(gamma_vehicle),
    -cos(phi) * sin(psi), sin(phi) * sin(psi) * cos(gamma_vehicle) + cos(psi) * sin(gamma_vehicle), -sin(phi) * sin(psi) * sin(gamma_vehicle) + cos(psi) * cos(gamma_vehicle);
	double L11 = L(0, 0);
	double L12 = L(0, 1);
	double L13 = L(0, 2);
	double L21 = L(1, 0);
	double L22 = L(1, 1);
	double L23 = L(1, 2);
	double L31 = L(2, 0);
	double L32 = L(2, 1);
	double L33 = L(2, 2);

	x_r_last = x_r;		//Because the feather of atan function 
	y_r_last = y_r;
	z_r_last = z_r;

	P_r = P_T - P_M;
	x_r = P_r[0];
	y_r = P_r[1];
	z_r = P_r[2];

	q_lambda = atan(y_r /x_r);
   	q_gamma = - atan(z_r/sqrt(x_r * x_r + y_r * y_r));
    	q_lambda_last = atan(y_r_last/x_r_last);
    	q_gamma_last = - atan(z_r_last/sqrt(x_r_last * x_r_last + y_r_last * y_r_last));
    	q_lambda_delta = q_lambda - q_lambda_last;
   	q_gamma_delta = q_gamma - q_gamma_last;
	
	delta_sita = K1 * q_gamma_delta;
      delta_psi_v = K2 * q_lambda_delta;
	if (delta_sita > 1 || delta_psi_v > 1)
	{
		delta_psi_v = 0;
		delta_sita = 0;
	}
	//cout << "I come in, and now psi_v = " << psi_v  << " sita = " << sita << endl;	
	sita = sita + delta_sita;
   	psi_v = psi_v + delta_psi_v;
	//cout << "I come out, and now psi_v = " << psi_v  << " sita = " << sita << endl;
	
	//cout << "psi_v = " << psi_v << endl;
	V_M << VM * cos(sita) * cos(psi_v), VM * cos(sita) * sin(psi_v), VM * sin(- sita);

}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	//read the drone postion from the Mavros Package [Frame: ENU]
	Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

	pos_drone = pos_drone_fcu_enu;
	position_get = pos_drone;
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
			temp_pos_target[0] = temp_pos_drone[0] - desire_Radius;
			temp_pos_target[1] = temp_pos_drone[1];
			temp_pos_target[2] = desire_z;
			MoveTimeCnt += 2;
			if(MoveTimeCnt >= 500)
			{
				FlyState = REST;
				MoveTimeCnt = 0;
			}
			pos_target[0] = temp_pos_drone[0] + (temp_pos_target[0] - temp_pos_drone[0]) * (MoveTimeCnt/500);
			pos_target[1] = temp_pos_drone[1] + (temp_pos_target[1] - temp_pos_drone[1]) * (MoveTimeCnt/500);
			pos_target[2] = temp_pos_drone[2] + (temp_pos_target[2] - temp_pos_drone[2]) * (MoveTimeCnt/500);
			send_pos_setpoint(pos_target, 0);
			if(current_state.mode != "OFFBOARD")			//if it is switched to "onboard" mode, jump to the "WATTING"
			{
				FlyState = WATTING;
			}
			//cout << "PREPARE" << endl;
			break;
		case REST:
			pos_target[0] = temp_pos_drone[0] - desire_Radius;
			pos_target[1] = temp_pos_drone[1];
			pos_target[2] = desire_z;
			send_pos_setpoint(pos_target, 0);
			MoveTimeCnt += 1;
			if(MoveTimeCnt >= 100)
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
				//float phase = 3.1415926;
				//float Omega = 2.0 * 3.14159 * MoveTimeCnt / priod;  //0~2pi
/* 				float theta = atan((pos_drone[1] - temp_pos_drone[1]) / (pos_drone[0] - temp_pos_drone[0]));
				//MoveTimeCnt += 3;
				if (pos_drone[0] >= 0)
				{
					velocity_sp[0] = DESIRE_V * sin(theta);
					velocity_sp[1] = - DESIRE_V * cos(theta);
					velocity_sp[2] = 0;
				}
				else
				{
					velocity_sp[0] = - DESIRE_V * sin(theta);
					velocity_sp[1] = DESIRE_V * cos(theta);
					velocity_sp[2] = 0;
				} */
				Guidance_Update();
				velocity_sp = V_M;
				//velocity_sp = {3,3,3};
				send_vel_setpoint(velocity_sp);
				//cout << "I am here!!! " << endl;
				if(current_state.mode != "OFFBOARD")			//if it is switched to "onboard" mode, jump to the "WATTING"
				{
					FlyState = WATTING;
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
	
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);			//handle return function	
    	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
	ros::Subscriber attitude_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, att_cb);

	setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");			//little question
		nh.param<float>("desire_z", desire_z, 10.0);
		nh.param<float>("desire_Radius", desire_Radius, 10.0);

	cout << "circular node started!!!" << endl; 
	ros::Rate rate(20.0);
	while(ros::ok())
	{
		
		FlyState_update();
		ros::spinOnce();
		rate.sleep();

	}
	return 0;
}





