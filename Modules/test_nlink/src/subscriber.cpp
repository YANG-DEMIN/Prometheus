/***************************************************************************************************************************
* subscriber.cpp
*
* Author: YDM
*
* Update Time: 2020.12.08
*
* Introduction:   
*         1. 订阅来自两个节点的NodeFrame2协议消息并打印在终端
***************************************************************************************************************************/
#include <ros/ros.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackNode2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <std_msgs/String.h>

using namespace std;
float target_pos[3];
//UWB callback function
void tagframe2Callback(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg) {
  int id;
  float dis;
  for(auto &item : msg->nodes)
  {
    id = item.id;
    dis = item.dis;
  }
  printf("UWB:\n");
  printf("\tSystemTime: %d\n", msg->system_time);
  printf("\tposition:\n");
  printf("\t\tx: %f\n", msg->pos_3d[0]);
  printf("\t\ty: %f\n", msg->pos_3d[1]);
  printf("\t\tz: %f\n", msg->pos_3d[2]);
  
  //printf("Target:\n");
  //printf("\tSystemTime: %d\n", msg->system_time);
  //printf("\tDistance: %f\n", dis);

}

//PX4 callback function
void px4_cb(const geometry_msgs::PoseStamped &msg)
{
  printf("UAV:\n");
  printf("\tstamp: %d\n", msg.header.stamp.sec);
  printf("\tPosition:\n");
  printf("\t\tx: %f\n", msg.pose.position.x);
  printf("\t\ty: %f\n", msg.pose.position.y);
  printf("\t\tz: %f\n", msg.pose.position.z);
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
  target_pos[0] = str2float(msg->data.substr(8,15));
  target_pos[1] = str2float(msg->data.substr(18,25));
  target_pos[2] = str2float(msg->data.substr(28,35));
  printf("the size of string is %d\n",num);
  printf("%f\n", target_pos[0]);
  printf("%f\n", target_pos[1]);
  printf("%f\n", target_pos[2]);
  //printf("lenght=: %d\n", msg->data.size);
//  printf("\tstamp: %d\n", msg.header.stamp.sec);
//  printf("\tPosition:\n");
//  printf("\t\tx: %f\n", msg.pose.position.x);
//  printf("\t\ty: %f\n", msg.pose.position.y);
//  printf("\t\tz: %f\n", msg.pose.position.z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "linktrack_subscriber");
  ros::NodeHandle nh;
  ros::Rate rate(50.0);

  ros::Subscriber uwb_sub = 
		nh.subscribe("nlink_linktrack_nodeframe2", 1000, tagframe2Callback);
  ros::Subscriber px4_sub = 
		nh.subscribe("mavros/local_position/pose", 1000, px4_cb);
  ros::Subscriber target_sub = 
	  nh.subscribe("/nlink_linktrack_data_transmission", 1000, target_cb);
  while(ros::ok())
	{
		
		ros::spinOnce();
		rate.sleep();

	}

  return 0;
}
