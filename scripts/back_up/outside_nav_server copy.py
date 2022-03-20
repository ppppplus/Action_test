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
        self.ac_move_base = actionlib.SimpleActionClient("/" + self.robot_name + "/move_base", MoveBaseAction)  
        self.cmd_vel = rospy.Publisher(self.robot_name + '/cmd_vel', Twist, queue_size=10)
        self.move_cmd = Twist()

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

    def outside_navCallback(self, req):
    	# 显示请求数据
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nics/panda_ws/src/segwayrmp/launch/trans/move_base_map.launch"])
        # launch.start()
        # subprocess.call (["roslaunch segwayrmp move_base_map.launch"],shell=True)
        outside_nav_process = subprocess.Popen(['bash', '-c', 'roslaunch segwayrmp move_base_map_outside.launch'])
    
        rospy.loginfo("outside navgation started.")
        rospy.loginfo("Waiting for move_base action server...")  
    
        # Wait 60 seconds for the action server to become available  
        self.ac_move_base.wait_for_server(rospy.Duration(60)) 
        rospy.loginfo("move_base start!!!")
        rospy.sleep(5)
        
        rospy.loginfo("Goal1 send!!!x:[%f] y[%f]", self.goal_x, self.goal_y)
        self.ac_move_base.send_goal(self.create_move_base_goal())
       
        # while not rospy.is_shutdown():
        #     state = move_base.get_state()
        #     rospy.loginfo("state:::::::::::::::::::::::::::::::::::::;%d", state)  
        #     if state == GoalStatus.SUCCEEDED:  
        #         rospy.loginfo("Goal succeeded!")  
        #         outside_nav_process.terminate()
        #     rospy.sleep(1)
        # rospy.loginfo("state:::::::::::::::::::::::::::::::::::::;%d", GoalStatus.SUCCEEDED)  
        rospy.loginfo("debug1::::::::::::::::::!")
       
        # self._ac_move_base.wait_for_result()
        # result = self._ac_move_base.get_result()
        # self.ac_move_base.wait_for_result()
        while not rospy.is_shutdown():
            result = self.ac_move_base.get_result()
            rospy.loginfo(result)
            rospy.sleep(1)
        rospy.loginfo("debug2::::::::::::::::::!")
        if not finished_time:
            rospy.loginfo("debug3::::::::::::::::::!")
            self.ac_move_base.cancel_goal()  
            rospy.loginfo("Timed out achieving goal") 
        else:  
            # rospy.loginfo("else::::::::::::::::::!")
            # state = self.ac_move_base.get_state()  
            # if state == GoalStatus.SUCCEEDED:  
            rospy.loginfo("Goal succeeded!")  
            rospy.loginfo("Goal 1 reached!!!!")  
            self.move_cmd.angular.z = -0.5
            self.cmd_vel.publish(self.move_cmd)
            rospy.sleep(8)
            self.goal_x = 0.18
            self.goal_y = -7.7
            self.goal_angz = 1
            self.goal_w = 0
            rospy.loginfo("Goal2 send!!!x:[%f] y[%f]", self.goal_x, self.goal_y)
            self.ac_move_base.send_goal(self.create_move_base_goal())
            finished_time = self.ac_move_base.wait_for_result(rospy.Duration(300))
            if not finished_time:
                self.ac_move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.ac_move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")  
                    outside_nav_process.terminate()
                else:  
                    rospy.loginfo("Goal failed with error code: " + str(state)) 
    	# # 反馈数据
        rospy.loginfo("out::::::::::::::::::!")
        return outside_navResponse(True)
        

if __name__ == "__main__":
    rospy.init_node('outside_nav_server_node')
    outside_nav_server = OutsideNavServer("panda")
    rospy.spin()