#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from geometry_msgs.msg import Twist
from action_panda.srv import *


class ElevatorServer:

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.start_list=[]
        self.start_dis = 0
        self.ele_in_flag = False
        self.door_close = False
        self.cmd_vel = rospy.Publisher(self.robot_name + '/cmd_vel', Twist, queue_size=10)
        self.move_cmd = Twist()
        self.inside_flag = False
        self.scan_ = LaserScan()
        rospy.logwarn("Elevator_server ready!")
        rospy.Service('/elevator', elevator, self.elevatorCallback)

    def avg(self, list_):
        sum_ = 0
        count = 0
        for n in list_:
            if not math.isinf(n):
                sum_ = sum_ + n
                count = count + 1
        # rospy.loginfo("sum:%f, count:%d", sum, count)
        avg_ = sum_/count
        return avg_

    def laserCallback(self):
        dis_avg = self.avg(list(self.scan_.ranges)[814:914])
        self.start_list.append(dis_avg)
        # rospy.loginfo("start_list_len:%d", len(start_list))
        if not self.ele_in_flag:
            if len(self.start_list) == 10:
                self.start_dis = self.avg(self.start_list)
                rospy.loginfo("start_dis:%f", self.start_dis)
            if self.start_dis!=0 and dis_avg - self.start_dis > 1.0:
                rospy.loginfo("dis:%f", dis_avg - self.start_dis)
                
                self.move_cmd.linear.x = 0.3
                rospy.sleep(1)
                self.cmd_vel.publish(self.move_cmd)
                rospy.sleep(3.5+self.start_dis/0.3)
                rospy.loginfo("run time:%f", 3.5+self.start_dis/0.3)
                self.move_cmd.linear.x = 0
                self.cmd_vel.publish(self.move_cmd)
                self.ele_in_flag = True
                rospy.sleep(5)
        else: 
            dis_temp = self.avg(list(self.scan_.ranges)[238:338])
            if self.door_close:
                dis_temp = self.avg(list(self.scan_.ranges)[238:338])
                if dis_temp > 2:
                    self.move_cmd.linear.x = -0.3
                    rospy.sleep(2)
                    self.cmd_vel.publish(self.move_cmd)
                    rospy.sleep(4+self.start_dis/0.3)
                    rospy.loginfo("go out!!!!!!!")
                    self.move_cmd.linear.x = 0
                    self.move_cmd.angular.z = 0.5
                    self.cmd_vel.publish(self.move_cmd)
                    rospy.sleep(14)
                    self.move_cmd.angular.z = 0
                    self.cmd_vel.publish(self.move_cmd)
                    self.inside_flag = True
                    # rospy.signal_shutdown("closed!")
            if dis_temp < 1:
                rospy.loginfo("door closed!")
                self.door_close = True

    def elevatorCallback(self, req):
        # rospy.Subscriber(self.robot_name + "/scan", LaserScan, self.laserCallback)
        while not rospy.is_shutdown():
            if self.inside_flag:
                break
            self.scan_ = rospy.wait_for_message(self.robot_name + '/scan', LaserScan)       
            self.laserCallback()
            rospy.sleep(1)
        return elevatorResponse(True) 

if __name__=='__main__':
    rospy.init_node('elevator_server', anonymous=True)
    robot_name = rospy.get_param('~robot_name', 'panda')
    elevator_server = ElevatorServer(robot_name)
    rospy.spin()
 
 