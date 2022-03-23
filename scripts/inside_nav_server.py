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

point_dict = {('1170','180'):(2.83,0,0,1), ('680','1220'):(3,1,0,1)}

class InsideNavServer:

    def __init__(self, robot_name, goal_tole):
        self.robot_name = robot_name
        self.goal_dis2 = goal_tole**2
        self.goal_x = 2.83
        self.goal_y = 0
        self.goal_angz = 0
        self.goal_w = 1
        self.ac_move_base = actionlib.SimpleActionClient("/" + self.robot_name + "/move_base", MoveBaseAction)  
        self.cmd_vel = rospy.Publisher(self.robot_name + '/cmd_vel', Twist, queue_size=10)
        self.move_cmd = Twist()
        rospy.logwarn("Inside_nav_server ready!")
        rospy.Service('/inside_nav', inside_nav, self.inside_navCallback)

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

    def inside_navCallback(self, req):
        rospy.logwarn("car_id:%d", req.car_id)

        goal_point = point_dict[(req.picX, req.picY)]
        (self.goal_x, self.goal_y, self.goal_angz, self.goal_w) = goal_point
        rospy.loginfo("Get goal from client:x[%f], y[%f]", self.goal_x, self.goal_y)

        inside_nav_process = subprocess.Popen(['bash', '-c', 'roslaunch segwayrmp move_base_map_inside.launch'])
        rospy.loginfo("Inside navgation started.")
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
        
        # while not rospy.is_shutdown():
        #     state = move_base.get_state()
        #     rospy.loginfo("state:::::::::::::::::::::::::::::::::::::;%d", state)  
        #     if state == GoalStatus.SUCCEEDED:  
        #         rospy.loginfo("Goal succeeded!")  
        #         inside_nav_process.terminate()
        #     rospy.sleep(1)
        # rospy.loginfo("state:::::::::::::::::::::::::::::::::::::;%d", GoalStatus.SUCCEEDED)  

        rospy.loginfo("Goal1 succeeded!") 
        self.ac_move_base.cancel_goal() 
        # rospy.loginfo("Goal 1 reached!!!!")  
        inside_nav_process.terminate()
        rospy.loginfo("Nav node closed!")
        return inside_navResponse(True)
        

if __name__ == "__main__":
    rospy.init_node('inside_nav_server')
    robot_name = rospy.get_param('~robot_name', 'panda')
    xy_tole = rospy.get_param("~xy_tole", 0.3)
    inside_nav_server = InsideNavServer(robot_name, xy_tole)
    rospy.spin()