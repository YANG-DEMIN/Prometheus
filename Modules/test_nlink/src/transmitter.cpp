/***************************************************************************************************************************
* transmitter.cpp
*
* Author: YDM
*
* Update Time: 2020.12.08
*
* Introduction:   
*   1. 订阅AnchorFrame0协议消息并将两个tag位置信息打印在终端
    2. 发送target位置至UAV，通过LinktrackNode2消息
    3. 目标ID必须设置为2， 无人机ID设置为1
***************************************************************************************************************************/
#include <ros/ros.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackAnchorframe0.h>
//不确定是否需要添加Noden.h
#include <nlink_parser/LinktrackNode2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <std_msgs/String.h>

// #define LAT_START 22.5180977 //A0维度
// #define LONG_START 113.9007239 //A0经度
// #define LSB_M_TO_LAT_LONG 8.993216059e-6 //meter2lat_lon

using namespace std;

int target_id, uav_id;
float target_pos[3], uav_pos[3];
float local_pos[3], global_pos[3];

//UWB Anchor callback function
void Anchorframe0Callback(const nlink_parser::LinktrackAnchorframe0::ConstPtr &msg) {

  for(auto &item : msg->nodes)
  {
    if(item.id == 2)
    {
      target_id = item.id;
      target_pos[0] = item.pos_3d[0];
      target_pos[1] = item.pos_3d[1];
      target_pos[2] = item.pos_3d[2];
    }

    if(item.id == 1)
    {
      uav_id = item.id;
      uav_pos[0] = item.pos_3d[0];
      uav_pos[1] = item.pos_3d[1];
      uav_pos[2] = item.pos_3d[2];
    }

  }



  printf("SystemTime: %d\n", msg->system_time);
  printf("UAV:\n");
  printf("\tID: %d\n", uav_id);
  printf("\tposition:\n");
  printf("\t\tx: %f\n", uav_pos[0]);
  printf("\t\ty: %f\n", uav_pos[1]);
  printf("\t\tz: %f\n", uav_pos[2]);

  printf("Target:\n");
  printf("\tID: %d\n", target_id);
  printf("\tposition:\n");
  printf("\t\tx: %f\n", target_pos[0]);
  printf("\t\ty: %f\n", target_pos[1]);
  printf("\t\tz: %f\n", target_pos[2]);
}

// //local_pos-->uwb坐标系东北天
// //global_pos-->全球坐标系
// float *local2global(float Local_Pos[3])
// {
    
//     //维度->北
//   global_pos[0] = (Local_Pos[1] * LSB_M_TO_LAT_LONG + LAT_START) * 10e7;
//     //经度
//   global_pos[1] = (Local_Pos[0] * LSB_M_TO_LAT_LONG + LONG_START) * 10e7;
//     //高度
//   global_pos[2] = (Local_Pos[2]) * 10e3;
  
//   return global_pos;
// }

std_msgs::String str;
int main(int argc, char **argv) {
  ros::init(argc, argv, "linktrack_transmitter");
  ros::NodeHandle nh;
  ros::Rate rate(10.0);

  std::cout << "Transmitter Started !!!" << endl;
  ros::Subscriber anchor_sub = 
	nh.subscribe("nlink_linktrack_anchorframe0", 1000, Anchorframe0Callback);

  ros::Publisher send_location_pub = nh.advertise<std_msgs::String>("/nlink_linktrack_data_transmission", 10);
  std::cout << " nlink_linktrack_data_transmission has been advertised,use 'rostopic echo /nlink_linktrack_data_transmission' to view the data" << endl;
  
  while(ros::ok())
	{
    str.data = "target: " + std::to_string(target_pos[0]) + "  "+ std::to_string(target_pos[1]) + "  "+ std::to_string(target_pos[2]) + " " +
                "uav: " + std::to_string(uav_pos[0]) + "  "+ std::to_string(uav_pos[1]) + "  "+ std::to_string(uav_pos[2]);
		send_location_pub.publish(str);
		ros::spinOnce();
		rate.sleep();

	}

  return 0;
}
