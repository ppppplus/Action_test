#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "action_panda/box_focousAction.h"

typedef actionlib::SimpleActionServer<action_panda::box_focousAction> Server;

// 收到action的goal后调用的回调函数
void execute(const action_panda::box_focousGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    action_panda::box_focousFeedback feedback;

    ROS_INFO("Car %d is focousing... first_rot:%d", goal->car_id, goal->first_rot);

    // 假设洗盘子的进度，并且按照1hz的频率发布进度feedback
    feedback.find_flag = 1;
    as->publishFeedback(feedback);
    r.sleep();
	// 当action完成后，向客户端返回结果
    ROS_INFO("Car %d finish focousing.", goal->car_id);
    as->setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "box_focous_node");
    ros::NodeHandle n;

	// 定义一个服务器
    Server server(n, "box_focous", boost::bind(&execute, _1, &server), false);
	
	// 服务器开始运行
    server.start();

    ros::spin();

    return 0;
}
