#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import time
from action_panda.srv import *

class TaskReadyServer:

    def __init__(self):
        rospy.logwarn("Task_ready_server ready!")
        rospy.Service('/task_ready', task_ready, self.task_readyCallback)

    def CurrentTimeMillis(self):
        return int(round(time.time() * 1000))

    def task_readyCallback(self, req):
        # rospy.Subscriber(self.robot_name + "/scan", LaserScan, self.laserCallback)
        if req.type == "delivery" and self.CurrentTimeMillis() - req.actionTime < 60000:
            rospy.loginfo("Delivery command received!")
            
        res = task_readyResponse()
        res.deviceId = req.deviceId
        res.indoorX = req.indoorX
        res.indoorY = req.indoorY
        res.floorNo = req.floorNo
        return res

if __name__=='__main__':
    rospy.init_node('task_ready_server', anonymous=True)
    task_ready_server = TaskReadyServer()
    rospy.spin()
 
 