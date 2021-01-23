#! /usr/bin/env python
# -*- coding: utf-8 -*-
#author : Colin Lee
#email  : lcyfly1@163.com
#description  :  control qrcode movement 
import rospy
import math
import rospy
import tf
import yaml
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandVtolTransition, SetMode
from geometry_msgs.msg import PoseStamped, Pose, Twist
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
#from pyquaternion import Quaternion
from multiprocessing import Process
import sys

move_type = 1
#motion_type = 0     #位置控制
motion_type = 0     #速度控制


def pose_publisher_line():
    pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    pose_msg = PositionTarget()
    #pose_msg.model_name = 'car_landing_pad'
    rate = rospy.Rate(100)
    linear_vel = 1.0
    time = 0.0
    while not rospy.is_shutdown():

        #pose_msg.header.stamp = rospy.Time.now()
        #pose_msg.header.frame_id = "home"
        pose_msg.coordinate_frame = 1

        pose_msg.position.x = 2.0
        pose_msg.position.y = 2.0
        pose_msg.position.z = 0.0

        pose_msg.velocity.x = 0.2
        pose_msg.velocity.y = 0.2
        pose_msg.velocity.z = 0.0

        pose_msg.yaw = 0.0
        pose_msg.yaw_rate = 0.0

        if(motion_type == 0):
            pose_msg.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
            pose_msg.type_mask = 504
            print('Pos_x :',pose_msg.position.x)
            print('Pos_y :',pose_msg.position.y)
            print('Pos_z :',pose_msg.position.z)
            print('Yaw   :',pose_msg.yaw)
        if(motion_type == 1):
            pose_msg.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                 + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ  + PositionTarget.IGNORE_YAW_RATE
            print('Vel_x :',pose_msg.velocity.x)
            print('Vel_y :',pose_msg.velocity.y)
            print('Vel_z :',pose_msg.velocity.z)
            print('Yaw   :',pose_msg.yaw)
        pub.publish(pose_msg)

        rate.sleep()


def pose_publisher_circle():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'car_landing_pad'
    rate = rospy.Rate(100)
    linear_vel = 0.5
    circle_radius = 3.0
    omega = math.fabs(linear_vel / circle_radius)
    time = 0.0
    while not rospy.is_shutdown():
        angle = time * omega
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        time = time + 0.01
        pose_msg.pose.position.x = circle_radius*cos_angle
        pose_msg.pose.position.y = circle_radius*sin_angle
        pose_msg.pose.position.z = 0.01
        pub.publish(pose_msg)
        print('Pos_x :',pose_msg.pose.position.x)
        print('Pos_y :',pose_msg.pose.position.y)
        rate.sleep()

if __name__ == '__main__':
      rospy.init_node('car_pose_publisher')
      try:
          if move_type == 0:
            pose_publisher_circle()
          elif move_type == 1:
            pose_publisher_line()
          
      except rospy.ROSInterruptException:
          pass
