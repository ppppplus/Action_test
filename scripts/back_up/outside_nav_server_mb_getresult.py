#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslaunch
import rospy
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from action_panda.srv import *
import os
import subprocess
from subprocess import Popen
import math

class OutsideNavServer:

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.goal_x = 5.14
        self.goal_y = -6.5
        self.goal_angz = -0.7
        self.goal_w = 0.7
        self.goal_pub = rospy.Publisher(self.robot_name + '/move_base_simple/goal', PoseStamped, queue_size = 1)
        self.cmd_vel = rospy.Publisher(self.robot_name + '/cmd_vel', Twist, queue_size=10)
        self.move_cmd = Twist()

        rospy.Service('/outside_nav', outside_nav, self.outside_navCallback)

    def pub_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = self.robot_name + '/map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.goal_x
        pose.pose.position.y = self.goal_y
        pose.pose.orientation.z = self.goal_angz # z=sin(yaw/2)
        pose.pose.orientation.w = self.goal_w # w=cos(yaw/2)
        self.goal_pub.publish(pose)

    def outside_navCallback(self, req):
    	# 显示请求数据
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nics/panda_ws/src/segwayrmp/launch/trans/move_base_map.launch"])
        # launch.start()
        # subprocess.call (["roslaunch segwayrmp move_base_map.launch"],shell=True)
        outside_nav_process = subprocess.Popen(['bash', '-c', 'roslaunch segwayrmp move_base_map_outside.launch'])
    
        rospy.loginfo("outside navgation started.")
        # rospy.loginfo("Waiting for move_base action server...")  
    
        # Wait 60 seconds for the action server to become available  
        # self.ac_move_base.wait_for_server(rospy.Duration(60)) 
        # rospy.loginfo("move_base start!!!")
        rospy.sleep(8)
        
        rospy.loginfo("Goal1 send!!!x:[%f] y[%f]", self.goal_x, self.goal_y)
        self.pub_pose()
        while not rospy.is_shutdown():
            res = rospy.wait_for_message(self.robot_name + '/move_base/status', GoalStatusArray)       
            rospy.loginfo(res.status_list[0].status)
            if res.status_list and res.status_list[0].status==3:
                break
            rospy.sleep(1)
        
        # while not rospy.is_shutdown():
        #     state = move_base.get_state()
        #     rospy.loginfo("state:::::::::::::::::::::::::::::::::::::;%d", state)  
        #     if state == GoalStatus.SUCCEEDED:  
        #         rospy.loginfo("Goal succeeded!")  
        #         outside_nav_process.terminate()
        #     rospy.sleep(1)
        # rospy.loginfo("state:::::::::::::::::::::::::::::::::::::;%d", GoalStatus.SUCCEEDED)  
       
        # self._ac_move_base.wait_for_result()
        # result = self._ac_move_base.get_result()
        # self.ac_move_base.wait_for_result()
        # rospy.loginfo("else::::::::::::::::::!")
        # state = self.ac_move_base.get_state()  
        # if state == GoalStatus.SUCCEEDED:  
        rospy.loginfo("Goal1 succeeded!")  
        # rospy.loginfo("Goal 1 reached!!!!")  
        self.move_cmd.angular.z = -0.5
        self.cmd_vel.publish(self.move_cmd)
        rospy.sleep(6)
        self.move_cmd.angular.z = 0
        self.cmd_vel.publish(self.move_cmd)
        
        self.goal_x = 0.18
        self.goal_y = -7.7
        self.goal_angz = 1
        self.goal_w = 0
        rospy.loginfo("Goal2 send!!!x:[%f] y[%f]", self.goal_x, self.goal_y)
        self.pub_pose()
        while not rospy.is_shutdown():
            res = rospy.wait_for_message(self.robot_name + '/move_base/status',  GoalStatusArray)       
            if res.status_list and res.status_list[0].status==3:
                break
            rospy.sleep(1)
        rospy.loginfo("Goal2 succeeded!")  
        outside_nav_process.terminate()
        rospy.loginfo("Nav node closed!")
    	# # 反馈数据
        return outside_navResponse(True)
        

if __name__ == "__main__":
    rospy.init_node('outside_nav_server_node')
    outside_nav_server = OutsideNavServer("panda")
    rospy.spin()