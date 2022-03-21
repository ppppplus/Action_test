#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslaunch
import rospy
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseStatus
from action_panda.srv import *
import os
import subprocess
from subprocess import Popen
import math

class InsideNavServer:

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.goal_x = 2.83
        self.goal_y = 0
        self.goal_angz = 0
        self.goal_w = 1
        self.goal_pub = rospy.Publisher(self.robot_name + '/move_base_simple/goal', PoseStamped, queue_size = 1)
        self.cmd_vel = rospy.Publisher(self.robot_name + '/cmd_vel', Twist, queue_size=10)
        self.move_cmd = Twist()

        rospy.Service('/inside_nav', inside_nav, self.inside_navCallback)

    def pub_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = self.robot_name + '/map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.goal_x
        pose.pose.position.y = self.goal_y
        pose.pose.orientation.z = self.goal_angz # z=sin(yaw/2)
        pose.pose.orientation.w = self.goal_w # w=cos(yaw/2)
        self.goal_pub.publish(pose)

    def inside_navCallback(self, req):
    	# 显示请求数据
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nics/panda_ws/src/segwayrmp/launch/trans/move_base_map.launch"])
        # launch.start()
        # subprocess.call (["roslaunch segwayrmp move_base_map.launch"],shell=True)
        inside_nav_process = subprocess.Popen(['bash', '-c', 'roslaunch segwayrmp move_base_map_inside.launch'])
    
        rospy.loginfo("inside navgation started.")
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
            if res.status_list and res.status_list[0].status == 3:
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

        rospy.loginfo("Goal succeeded!")  
        # rospy.loginfo("Goal 1 reached!!!!")  
        inside_nav_process.terminate()
        rospy.loginfo("Nav node closed!")
        return inside_navResponse(True)
        

if __name__ == "__main__":
    rospy.init_node('inside_nav_server_node')
    inside_nav_server = InsideNavServer("panda")
    rospy.spin()