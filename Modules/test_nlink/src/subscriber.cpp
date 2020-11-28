#include <ros/ros.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackNode2.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

//UWB callback function
void tagframe2Callback(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg) {
  int id;
  float dis;
  for(auto &item : msg->nodes)
  {
    id = item.id;
    dis = item.dis;
  }
  printf("Target:\n");
  printf("\tSystemTime: %d\n", msg->system_time);
  printf("\tDistance: %f\n", dis);

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "linktrack_example");
  ros::NodeHandle nh;

  ros::Subscriber uwb_sub = 
		nh.subscribe("nlink_linktrack_nodeframe2", 1000, tagframe2Callback);
  ros::Subscriber px4_sub = 
		nh.subscribe("mavros/local_position/pose", 1000, px4_cb);
  ros::spin();

  return 0;
}
