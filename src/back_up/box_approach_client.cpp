#include <actionlib/client/simple_action_client.h>
#include "action_panda/box_approachAction.h"

typedef actionlib::SimpleActionClient<action_panda::box_approachAction> Client;

// 当action完成后会调用次回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
        const action_panda::box_approachResultConstPtr& result)
{
    ROS_INFO("Yay! The dishes are now clean");
    ros::shutdown();
}

// 当action激活后会调用次回调函数一次
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// 收到feedback后调用的回调函数
void feedbackCb(const action_panda::box_approachFeedbackConstPtr& feedback)
{
    ROS_INFO(" detect:%d, mb: %d ", feedback->detect_flag, feedback->mb_flag);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "box_approach_client");

	// 定义一个客户端
    Client client("box_approach", true);

	// 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

	// 创建一个action的goal
    action_panda::box_approachGoal goal;
    goal.car_id = 1;

    // 发送action的goal给服务器端，并且设置回调函数
    client.sendGoal(goal,  &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}
