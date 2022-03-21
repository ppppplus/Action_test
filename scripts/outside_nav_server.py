#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslaunch
import rospy
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from action_panda.srv import *
import os
import subprocess
from subprocess import Popen
import math

class OutsideNavServer:

    def __init__(self, robot_name, goal_tole):
        self.robot_name = robot_name
        self.goal_dis2 = goal_tole**2
        self.goal_x = 5.14
        self.goal_y = -6.8
        self.goal_angz = -0.7
        self.goal_w = 0.7
        self.ac_move_base = actionlib.SimpleActionClient("/" + self.robot_name + "/move_base", MoveBaseAction)  
        self.cmd_vel = rospy.Publisher(self.robot_name + '/cmd_vel', Twist, queue_size=10)
        self.move_cmd = Twist()
        rospy.logwarn("Outside_nav_server ready!")
        rospy.Service('/outside_nav', outside_nav, self.outside_navCallback)
    
    def create_geo_pose(self):
        pose = Pose()

        pose.position.x = self.goal_x
        pose.position.y = self.goal_y
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = self.goal_angz
        pose.orientation.w = self.goal_w
        return pose

    def create_move_base_goal(self):
        target = PoseStamped()
        target.header.frame_id = self.robot_name + "/map"
        target.header.stamp = rospy.Time.now()
        target.pose = self.create_geo_pose()
        goal = MoveBaseGoal(target)
        return goal

    def IfReach(self, fb_):
        fb_x = fb_.feedback.base_position.pose.position.x
        fb_y = fb_.feedback.base_position.pose.position.y
        dis_2 = (fb_x-self.goal_x)**2 + (fb_y-self.goal_y)**2
        rospy.loginfo("goal_dis:%f", dis_2)
        if dis_2 < self.goal_dis2:
            return True
        else:
            return False

    def CarMove(self, t_):
        self.cmd_vel.publish(self.move_cmd)
        rospy.sleep(t_)
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd) # stop

    def outside_navCallback(self, req):
        rospy.logwarn("car_id:%d", req.car_id)
        rospy.loginfo("Outside navigation reset...")
        self.move_cmd.linear.x = -0.3
        self.CarMove(2)
        self.move_cmd.angular.z = 0.5
        self.CarMove(6)
        outside_nav_process = subprocess.Popen(['bash', '-c', 'roslaunch segwayrmp move_base_map_outside.launch'])
        rospy.loginfo("Outside navgation started.")
        rospy.loginfo("Waiting for move_base action server...")  
    
        # Wait 60 seconds for the action server to become available  
        self.ac_move_base.wait_for_server(rospy.Duration(60)) 
        rospy.loginfo("move_base start!!!")
        # rospy.sleep(8)
        
        rospy.loginfo("Goal1 send!!!x:[%f] y[%f]", self.goal_x, self.goal_y)
        self.ac_move_base.send_goal(self.create_move_base_goal())
        
        while not rospy.is_shutdown():
            fb = rospy.wait_for_message(self.robot_name + '/move_base/feedback', MoveBaseActionFeedback)          
            if self.IfReach(fb):
                break
            rospy.sleep(1)
        
        rospy.loginfo("Goal1 succeeded!")  
        rospy.sleep(1) 
        self.ac_move_base.cancel_goal()
        rospy.loginfo("Goal1 canceled!")
        rospy.sleep(2)
        self.move_cmd.angular.z = -0.5
        self.CarMove(5)
        # rospy.loginfo("turn right...")        
        self.goal_x = 0.18
        self.goal_y = -7.75
        self.goal_angz = 1
        self.goal_w = 0 # goal2
        rospy.loginfo("Goal2 send!!!x:[%f] y[%f]", self.goal_x, self.goal_y)
        self.ac_move_base.send_goal(self.create_move_base_goal())

        while not rospy.is_shutdown():
            fb = rospy.wait_for_message(self.robot_name + '/move_base/feedback', MoveBaseActionFeedback)       
            if self.IfReach(fb):
                break
            rospy.sleep(1)

        rospy.loginfo("Goal2 succeeded!")
        rospy.sleep(1)  
        self.ac_move_base.cancel_goal()  
        outside_nav_process.terminate()
        rospy.loginfo("Nav node closed!")
    	# # 反馈数据
        return outside_navResponse(True)
        

if __name__ == "__main__":
    rospy.init_node('outside_nav_server')
    outside_nav_server = OutsideNavServer("panda", 0.3)
    rospy.spin()