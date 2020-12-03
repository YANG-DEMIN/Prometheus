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
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>


using namespace std;

float desire_z = 10.0;				//desired altitude
float desire_Radius = 10.0;		//desired radius of circle
float MoveTimeCnt = 0.0;
float priod = 2000.0;			//to change velocity of flying using it

Eigen::Vector3d pos_target; 	//desired value in offboard mode
Eigen::Vector3d temp_pos_drone;	
Eigen::Vector3d temp_pos_target;
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

//receive the current position of drone from controller
Eigen::Vector3d pos_drone;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	//read the drone postion from the Mavros Package [Frame: ENU]
	Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

	pos_drone = pos_drone_fcu_enu;
}

//receive the current position of drone from controller
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
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
				float phase = 3.1415926;
				float Omega = 2.0 * 3.14159 * MoveTimeCnt / priod;  //0~2pi
				MoveTimeCnt += 3;
				if(MoveTimeCnt >= priod)
				{
					FlyState = FLYOVER;
				}
				pos_target[0] = temp_pos_drone[0] + desire_Radius * cos(Omega + phase);
				pos_target[1] = temp_pos_drone[1] + desire_Radius * sin(Omega + phase);
				pos_target[2] = desire_z;
				send_pos_setpoint(pos_target, 0);
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
			cout << "FLYOVER" << endl;
			break;
		default:
			cout << "ERROR!!!" << endl;
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "circular_offboard");
	ros::NodeHandle nh("~");
	ros::Rate rate(20.0);

	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);			//handle return function

	setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");			//little question
		nh.param<float>("desire_z", desire_z, 10.0);
		nh.param<float>("desire_Radius", desire_Radius, 10.0);

	cout << "circular node started!!!" << endl; 

	while(1)
	{
		FlyState_update();
		ros::spinOnce();
		rate.sleep();

	}
	return 0;
}





