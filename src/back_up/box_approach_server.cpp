#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "action_panda/box_approachAction.h"


class BoxApproachtest
{
    protected:
        std::string action_name;
        ros::NodeHandle n;
        actionlib::SimpleActionServer<action_panda::box_approachAction> as_;
        action_panda::box_approachFeedback feedback;

    public:
        BoxApproachtest(std::string name):
          as_(n, name, boost::bind(&BoxApproachtest::execute, this, _1), false),
          action_name(name)

        {
          as_.start();
        }

        ~BoxApproachtest(void)
        {}
        // 收到action的goal后调用的回调函数
        void execute(const action_panda::box_approachGoalConstPtr& goal)
        {
            ros::Rate r(1);
            ROS_INFO("Car %d is approaching...", goal->car_id);

            // 假设洗盘子的进度，并且按照1hz的频率发布进度feedback
            feedback.detect_flag = 1;
            as_.publishFeedback(feedback);
            r.sleep();
            feedback.mb_flag = 1;
            as_.publishFeedback(feedback);
            r.sleep();
            action_panda::box_approachResult result;
            result.first_rot = 1;
            // as->publishResult(result);
        	// 当action完成后，向客户端返回结果
            ROS_INFO("Car %d finish approaching.", goal->car_id);
            as_.setSucceeded(result);
            ros::shutdown();
        }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "box_approach_node");
    BoxApproachtest boxapptest("box_approach");

    ros::spin();

    return 0;
}
