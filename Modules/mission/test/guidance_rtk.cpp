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
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>

#include <nlink_parser/LinktrackNode2.h>		//target information from Ground station

// #define LAT_0		47.3900
// #define LON_0		8.5400
#define RADIUS	 		6378137
#define f		1/298.257223563
//#include <circular.h>

using namespace std;
double e = 0.0;
double Rm = 0.0;
double Rn = 0.0;

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

float K1 = 4;						//proportion of guidance
float K2 = 4;
float UpdateTime = 0.05;

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

Eigen::Vector3d home;
//Eigen::Vector3d uav_home;

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

double x_r = 0;
double y_r = 0;
double z_r = 0;
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




// void target_pos_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
// {
// 	//read the drone postion from the Mavros Package [Frame: ENU]
// 	Eigen::Vector3d pos_drone_fcu_enu(msg->latitude, msg->longitude, msg->altitude);

// 	pos_drone[0] = (pos_drone_fcu_enu[0] - LAT_0) * LAT_TO_M;
// 	pos_drone[1] = (pos_drone_fcu_enu[1] - LON_0) * LAT_TO_M;
// 	pos_drone[2] = pos_drone_fcu_enu[2] - 50;
// 	//pos_drone = pos_drone_fcu_enu;
// 	position_get = pos_drone;
// 	P_M = position_get;
// }

void target_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	//read the drone postion from the Mavros Package [Frame: ENU]
	Eigen::Vector3d pos_target_fcu_enu(msg->latitude, msg->longitude, msg->altitude);

	//e = sqrt(FLATTENING*(2-FLATTENING));
	Rm = RADIUS * (1 - 2*f + 3*f*sin(home[0]) * sin(home[0]));
	Rn = RADIUS * (1 + f * sin(home[0]) * sin(home[0]));

	P_T[0] = (pos_target_fcu_enu[1] - home[1]) * M_PI / 180.0 * (Rn + pos_target_fcu_enu[2]) * cos(home[0] * M_PI / 180);		//东 经度
	P_T[1] = (pos_target_fcu_enu[0] - home[0]) * M_PI / 180.0 * (Rm + pos_target_fcu_enu[2]);			//北 维度
	P_T[2] = pos_target_fcu_enu[2] - home[2];

	// P_T[0] = (pos_target_fcu_enu[0] - home[0]) * LAT_TO_M;
	// P_T[1] = (pos_target_fcu_enu[1] - home[1]) * 110946.252133;
	// P_T[2] = pos_target_fcu_enu[2] - home[2];

}

//mavros_msgs::HomePosition;
void home_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{ 	
	//read the drone postion from the Mavros Package [Frame: ENU]
	home[0] = msg->geo.latitude;
	home[1] = msg->geo.longitude;
	home[2] = msg->geo.altitude;

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

	x_r_last = x_r;		//Because the feather of atan function 
	y_r_last = y_r;
	z_r_last = z_r;

	P_r = P_T - P_M;

	// cout << "P_r = " << endl << P_r << endl;

	x_r = P_r[0];
	y_r = P_r[1];
	z_r = P_r[2];

	q_lambda = atan2(y_r , x_r);
   	q_gamma = atan2(z_r , sqrt(x_r * x_r + y_r * y_r));
    q_lambda_last = atan2(y_r_last , x_r_last);
    q_gamma_last = atan2(z_r_last , sqrt(x_r_last * x_r_last + y_r_last * y_r_last));

	// q_lambda = atan(y_r /x_r);
   	// q_gamma = - atan(z_r/sqrt(x_r * x_r + y_r * y_r));
    // q_lambda_last = atan(y_r_last/x_r_last);
    // q_gamma_last = - atan(z_r_last/sqrt(x_r_last * x_r_last + y_r_last * y_r_last));
	if (q_lambda_last == atan2(0, 0) && q_gamma_last == atan2(0, 0))
	{
		q_lambda_last = q_lambda;
		q_gamma_last = q_gamma;
		cout << "I CHANGE THE VALUE!!!" << endl;
	}
    q_lambda_delta = q_lambda - q_lambda_last;
   	q_gamma_delta = q_gamma - q_gamma_last;
	
	delta_sita = K1 * q_gamma_delta;
    delta_psi_v = K2 * q_lambda_delta;
		
	sita = sita + delta_sita;
   	psi_v = psi_v + delta_psi_v;
	if (psi_v > 2 * M_PI)
	{
		psi_v = psi_v - 2 * M_PI;
	}
	
	V_M << VM * cos(sita) * cos(psi_v), VM * cos(sita) * sin(psi_v), VM * sin(sita);
	// cout << "sita = " << sita << endl;
	// cout << "psi_v = " << psi_v << endl;

}


//receive the current position of drone from controller

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
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

			FlyState = REST;
			//cout << "PREPARE" << endl;
			break;
		case REST:
			pos_target[0] = desire_x;				//fly to 10 m
			pos_target[1] = desire_y;
			pos_target[2] = desire_z;
			send_pos_setpoint(pos_target, 0);
			MoveTimeCnt += 1;
			
			if(MoveTimeCnt >= 200)
			// if(MoveTimeCnt >= 100 && abs(P_T[0] - desire_target_x) < 3 && abs(P_T[1] - desire_target_y) < 3 
			// 	&& abs(P_T[2] - desire_target_z) < 3)
			//if(MoveTimeCnt >= 100 )
			{
				MoveTimeCnt = 0;
				FlyState = FLY;
				cout << "---- Start attack!!! ----" <<endl;
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
	ros::init(argc, argv, "guidance_sitl");
	ros::NodeHandle nh;
	
   	//ros::Subscriber target_local_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 100, target_cb);
	ros::Subscriber target_global_sub = nh.subscribe<sensor_msgs::NavSatFix>("/uav1/mavros/global_position/global", 100, target_cb);	

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/uav0/mavros/state", 10, state_cb);			//handle return function	
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose", 100, pos_cb);
	//ros::Subscriber global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>("/uav0/mavros/global_position/global", 100, pos_cb);
	ros::Subscriber home_sub = nh.subscribe<mavros_msgs::HomePosition>("/uav0/mavros/home_position/home", 100, home_cb);
	ros::Subscriber attitude_sub = nh.subscribe<sensor_msgs::Imu>("uav0//mavros/imu/data", 10, att_cb);

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





