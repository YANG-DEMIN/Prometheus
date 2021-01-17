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
#include <cmath>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <iostream>
#include <math_utils.h>
#include <Eigen/Eigen>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
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

float desire_z = 10.0;				//desired altitude
float desire_Radius = 10.0;		//desired radius of circle
float MoveTimeCnt = 0.0;
float priod = 2000.0;			//to change velocity of flying using it

float DESIRE_V = 10;

Eigen::Vector3d P_T0 = {200, 100, 50};		//initial position of Target
Eigen::Vector3d P_M0 = {0, 0, 5};			//initial position of Missile
Eigen::Vector3d V_T = {-3, 0, 0};			//velocity of Target
Eigen::Vector3d V_M = {0, 0, 0};
float VM = 1;						//velocity of Missile

float K1 = 4;						//proportion of guidance
float K2 = 4;

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
Eigen::Vector2d Actuator_sp;

mavros_msgs::SetMode mode_cmd;
ros::Publisher setpoint_raw_local_pub;
ros::Publisher actuator_setpoint_pub;
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
double yaw_target = 0.0;

//receive the current position of drone from controller

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

float str2float(string str)
{
  float result;
  stringstream stream(str);
  stream >> result;
  return result;
}

void target_cb(const std_msgs::String::ConstPtr& msg)
{
  int num = msg->data.size();
  P_T[0] = str2float(msg->data.substr(8,15));
  P_T[1] = str2float(msg->data.substr(18,25));
  P_T[2] = str2float(msg->data.substr(28,35));

  P_M[0] = str2float(msg->data.substr(43,50));
  P_M[1] = str2float(msg->data.substr(53,60));
  P_M[2] = str2float(msg->data.substr(63,70));
  //东北天坐标系
  //printf("the size of string is %d\n",num);
  //printf("%f\n", P_M[0]);
  //printf("%f\n", P_M[1]);
  //printf("%f\n", P_M[2]);
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	//read the drone postion from the Mavros Package [Frame: ENU]
	Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

	pos_drone = pos_drone_fcu_enu;

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

//send the desired position to controller (vx, vy, vz)
void send_vel_setpoint(const Eigen::Vector3d& vel_sp)
{
	mavros_msgs::PositionTarget pos_setpoint;
	pos_setpoint.type_mask = 0b110111000111;	//0b 110 111 000 111  velocity

	//Bitmask to indicate which dimensions should be ignored (1 means ignore, 0 means not ignore; Bit 10 must set to 0)
	//Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7: ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11: yaw, bit 12:yaw_rate
	//Bit 10 should set to 0, means no force sp

	pos_setpoint.coordinate_frame = 1;

	pos_setpoint.velocity.x = vel_sp[0];
	pos_setpoint.velocity.y = vel_sp[1];
	pos_setpoint.velocity.z = vel_sp[2];

	pos_setpoint.yaw = yaw_target;

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


// 【发布】底层控制量（Mx My Mz 及 F） [0][1][2][3]分别对应 roll pitch yaw控制量 及 油门推力 注意 这里是NED系的！！
void send_actuator_setpoint(const Eigen::Vector2d& actuator_sp)
{
    mavros_msgs::ActuatorControl actuator_setpoint;

    actuator_setpoint.group_mix = 0;
    actuator_setpoint.controls[0] = 0.0;
    actuator_setpoint.controls[1] = 0.0;
    actuator_setpoint.controls[2] = actuator_sp(1);		//偏航
    actuator_setpoint.controls[3] = actuator_sp(0);		//油门
    actuator_setpoint.controls[4] = 0.0;
    actuator_setpoint.controls[5] = 0.0;
    actuator_setpoint.controls[6] = 0.0;
    actuator_setpoint.controls[7] = 0.0;

    actuator_setpoint_pub.publish(actuator_setpoint);

    // // 检查飞控是否收到控制量
    // cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>command_to_mavros<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    // //ned to enu
    // cout << "actuator_target [0 1 2 3] : " << actuator_target.controls[0] << " [ ] "<< -actuator_target.controls[1] <<" [ ] "<<-actuator_target.controls[2]<<" [ ] "<<actuator_target.controls[3] <<" [ ] "<<endl;

    // cout << "actuator_target [4 5 6 7] : " << actuator_target.controls[4] << " [ ] "<< actuator_target.controls[5] <<" [ ] "<<actuator_target.controls[6]<<" [ ] "<<actuator_target.controls[7] <<" [ ] "<<endl;

}
void Guidance_Update(void)
{

	P_r = P_T - P_M;
	x_r = P_r[0];
	y_r = P_r[1];
	z_r = P_r[2];

	psi = atan2(y_r , x_r);			//psi is defined to the angle of T-M line and x axis(East orientation)
	double yaw_fcu = attitude_get[2];		//实际偏航角 东北天坐标系 【-pi - pi】
	//yaw_target = psi + M_PI * 3 / 2;
	//if (yaw_target > 2 * M_PI)
	//{
	//	yaw_target = yaw_target - 2 * M_PI;
	//}
	yaw_target = psi;
	//double yaw_control = -10 * (yaw_target - yaw_fcu) / 2 / M_PI;
	//Actuator_sp[0] = 0.8;				//throttle
	//Actuator_sp[1] = yaw_control;		//yaw
	cout << "psi = " << psi << endl;
	cout << "yaw_fcu = " << yaw_fcu << endl;
	cout << "yaw_target = " << yaw_target << endl;
	//cout << "yaw_control = " << yaw_control << endl;
	V_M << VM * cos(psi), VM * sin(psi), 0;

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
			pos_target[0] = pos_drone[0];				//fly to 10 m
			pos_target[1] = pos_drone[1];
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

				//Guidance_Update();
				//velocity_sp = V_M;
				velocity_sp = {0.4,0.4,0};
				//send_actuator_setpoint(Actuator_sp);
				send_vel_setpoint(velocity_sp);
				//cout << "I am here!!! " << endl;
				if(current_state.mode != "OFFBOARD")			//if it is switched to "onboard" mode, jump to the "WATTING"
				{
					FlyState = WATTING;
				}
				if(abs(P_r[0]*P_r[0] + P_r[1]*P_r[1] + P_r[2]*P_r[2]) < 0.5)
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
	
   	ros::Subscriber target_sub = nh.subscribe<std_msgs::String>("/nlink_linktrack_data_transmission", 100, target_cb);	
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);			//handle return function	
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
	ros::Subscriber attitude_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, att_cb);

	setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
	actuator_setpoint_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);

	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");			//little question
		nh.param<float>("desire_z", desire_z, 10.0);
		nh.param<float>("desire_Radius", desire_Radius, 10.0);

	cout << "guidance node started!!!" << endl; 
	ros::Rate rate(20.0);
	while(ros::ok())
	{
		
		FlyState_update();
		Guidance_Update();
		ros::spinOnce();
		rate.sleep();

		cout << "P_T=" << "\t" << "\t" << "P_M="<< "\t"<< endl;
		cout << "\t" << P_T[0] << "\t" << P_M[0] << endl;
		cout << "\t" << P_T[1] << "\t" << P_M[1] << endl;
		cout << "\t" << P_T[2] << "\t" << P_M[2] << endl;

		cout << "vx = " << V_M[0] << endl;
		cout << "vy = " << V_M[1] << endl;
		cout << "vz = " << V_M[2] << endl;

	}
	return 0;
}





