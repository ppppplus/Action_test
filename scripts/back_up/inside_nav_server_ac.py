#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslaunch
import rospy
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from action_panda.srv import *
import os
import subprocess
from subprocess import Popen
import math

class InsideNavServer:

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.ac_move_base = actionlib.SimpleActionClient("/" + self.robot_name + "/move_base", MoveBaseAction)  

        rospy.Service('/inside_nav', inside_nav, self.inside_navCallback)

    def create_geo_pose(self):
        pose = Pose()
        pose.position.x = self.goal_x
        pose.position.y = self.goal_y
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        return pose

    def create_move_base_goal(self):
        target = PoseStamped()
        target.header.frame_id = self.robot_name + "/map"
        target.header.stamp = rospy.Time.now()
        target.pose = self.create_geo_pose()
        goal = MoveBaseGoal(target)
        return goal

    def inside_navCallback(self, req):
    	# 显示请求数据
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nics/panda_ws/src/segwayrmp/launch/trans/move_base_map.launch"])
        # launch.start()
        # subprocess.call (["roslaunch segwayrmp move_base_map.launch"],shell=True)
        inside_nav_process = subprocess.Popen(['bash', '-c', 'roslaunch segwayrmp move_base_map_inside.launch'])
    
        rospy.loginfo("Inside navgation started.")
        rospy.loginfo("Waiting for move_base action server...")  
    
        # Wait 60 seconds for the action server to become available  
        self.ac_move_base.wait_for_server(rospy.Duration(60)) 
        rospy.loginfo("move_base start!!!")
        rospy.sleep(5)
        
        rospy.loginfo("goal send!!!x:[%f] y[%f] yaw[%f]", self.goal_x, self.goal_y, self.goal_yaw)
        self.ac_move_base.send_goal(self.create_move_base_goal())
       
        # while not rospy.is_shutdown():
        #     state = move_base.get_state()
        #     rospy.loginfo("state:::::::::::::::::::::::::::::::::::::;%d", state)  
        #     if state == GoalStatus.SUCCEEDED:  
        #         rospy.loginfo("Goal succeeded!")  
        #         inside_nav_process.terminate()
        #     rospy.sleep(1)
        # rospy.loginfo("state:::::::::::::::::::::::::::::::::::::;%d", GoalStatus.SUCCEEDED)  
        finished_time = self.ac_move_base.wait_for_result()
        if not finished_time:
            self.ac_move_base.cancel_goal()  
            rospy.loginfo("Timed out achieving goal")  
        else:  
            state = self.ac_move_base.get_state()  
            if state == GoalStatus.SUCCEEDED:  
                rospy.loginfo("Goal succeeded!")  
                inside_nav_process.terminate()
            else:  
                rospy.loginfo("Goal failed with error code: " + str(state)) 
    	# # 反馈数据
        return inside_navResponse(True)
        

if __name__ == "__main__":
    rospy.init_node('inside_nav_server_node')
    inside_nav_server = InsideNavServer("panda")
    rospy.spin()