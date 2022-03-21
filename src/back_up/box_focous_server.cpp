#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "action_panda/box_focousAction.h"

class BoxFocoustest
{
    protected:
        std::string action_name;
        ros::NodeHandle n;
        actionlib::SimpleActionServer<action_panda::box_focousAction> as_;
        action_panda::box_focousFeedback feedback;

    public:
        BoxFocoustest(std::string name):
        as_(n, name, boost::bind(&BoxFocoustest::execute, this, _1), false),
        action_name(name)
        {
            as_.start();
        }

        ~BoxFocoustest(void)
        {}

        // 收到action的goal后调用的回调函数
        void execute(const action_panda::box_focousGoalConstPtr& goal)
        {
            ros::Rate r(1);
            ROS_INFO("Car %d is focousing... first_rot:%d", goal->car_id, goal->first_rot);

            // 假设洗盘子的进度，并且按照1hz的频率发布进度feedback
            feedback.find_flag = 1;
            as_.publishFeedback(feedback);
            r.sleep();
        	// 当action完成后，向客户端返回结果
            ROS_INFO("Car %d finish focousing.", goal->car_id);
            as_.setSucceeded();
            ros::shutdown();
        }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "box_focous_node");
    BoxFocoustest boxfocoustest("box_focous");

    ros::spin();

    return 0;
}
