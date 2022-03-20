#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math

start_list=[]
start_dis = 0
ele_in_flag = False
door_close = False
from geometry_msgs.msg import Twist

def avg(list):
    sum = 0
    count = 0
    for n in list:
        if not math.isinf(n):
            sum = sum + n
            count = count + 1
    # rospy.loginfo("sum:%f, count:%d", sum, count)
    avg = sum/count
    return avg

def laserCallback(msg):
    global start_dis
    global start_list
    global ele_in_flag
    global door_close
    dis_avg = avg(list(msg.ranges)[814:914])
    start_list.append(dis_avg)
    # rospy.loginfo("start_list_len:%d", len(start_list))
    cmd_vel = rospy.Publisher('/panda/cmd_vel', Twist, queue_size=10)
    if not ele_in_flag:
        if len(start_list) == 10:
            start_dis = avg(start_list)
            rospy.loginfo("start_dis:%f", start_dis)
        if start_dis!=0 and dis_avg - start_dis > 1.0:
            rospy.loginfo("dis:%f", dis_avg - start_dis)
            move_cmd = Twist()
            move_cmd.linear.x = 0.3
            rospy.sleep(1)
            cmd_vel.publish(move_cmd)
            rospy.sleep(3.5+start_dis/0.3)
            rospy.loginfo("run time:%f", 3.5+start_dis/0.3)
            move_cmd.linear.x = 0
            cmd_vel.publish(move_cmd)
            ele_in_flag = True
            rospy.sleep(5)
    else: 
        dis_temp = avg(list(msg.ranges)[238:338])
        if door_close:
            dis_temp = avg(list(msg.ranges)[238:338])
            if dis_temp > 2:
                move_cmd = Twist()
                move_cmd.linear.x = -0.3
                rospy.sleep(2)
                cmd_vel.publish(move_cmd)
                rospy.sleep(4+start_dis/0.3)
                rospy.loginfo("go out!!!!!!!")
                move_cmd.linear.x = 0
                move_cmd.angular.z = 0.5
                cmd_vel.publish(move_cmd)
                rospy.sleep(16)
                rospy.signal_shutdown("closed!")
        if dis_temp < 1:
            rospy.loginfo("door closed!")
            door_close = True




    # rospy.loginfo("length:%d", len(msg.ranges))
    # rospy.loginfo(type(list(msg.ranges)[300]))

def pose_subscriber():
	# ROS节点初始化)
    rospy.init_node('laser_subscriber', anonymous=True)
 
	# 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    rospy.Subscriber("/panda/scan", LaserScan, laserCallback)
 
	# 循环等待回调函数
    rospy.spin()
 
if __name__ == '__main__':
    pose_subscriber()
 
 