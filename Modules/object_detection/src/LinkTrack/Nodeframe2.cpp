#include <nlink_parser/LinktrackNodeframe2.h>
#include <ros/ros.h>

void tagframe0Callback(const nlink_parser::LinktrackNodeframe2 &msg) {
  ROS_INFO("msg LinktrackNodeframe2 received,systemTime: %d", msg.system_time);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "linktrack_example");
  ros::NodeHandle nh;

  ros::Subscriber sub = 
		nh.subscribe("nlink_linktrack_nodeframe2", 1000, tagframe0Callback);
  ros::spin();

  return 0;
}
