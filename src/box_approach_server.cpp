#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "action_panda/box_approachAction.h"

typedef actionlib::SimpleActionServer<action_panda::box_approachAction> Server;

// 收到action的goal后调用的回调函数
void execute(const action_panda::box_approachGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    action_panda::box_approachFeedback feedback;

    ROS_INFO("Car %d is approaching...", goal->car_id);

    // 假设洗盘子的进度，并且按照1hz的频率发布进度feedback
    feedback.detect_flag = 1;
    as->publishFeedback(feedback);
    r.sleep();
    feedback.mb_flag = 1;
    as->publishFeedback(feedback);
    r.sleep();
    action_panda::box_approachResult result;
    result.first_rot = 1;
    // as->publishResult(result);
	// 当action完成后，向客户端返回结果
    ROS_INFO("Car %d finish approaching.", goal->car_id);
    as->setSucceeded(result);
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "box_approach_node");
    ros::NodeHandle n;

	// 定义一个服务器
    Server server(n, "box_approach", boost::bind(&execute, _1, &server), false);
	
	// 服务器开始运行
    server.start();

    ros::spin();

    return 0;
}
