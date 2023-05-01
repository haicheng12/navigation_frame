#include <ros/ros.h>       //ros头文件
#include "logger/logger.h" //日志头文件
#include "context/context.h"
#include "utility_tool/utility_tool.h"
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

using namespace Sakura::Logger;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation");

    if (Nav::initLogger()) // 初始化日志系统
    {
        info("日志系统初始化成功");
    }
    else
    {
        fatal("日志系统初始化失败");
        return 0;
    }

    Nav::Context *context = new Nav::Context(); // //开辟空间
    context->initTaskPtr();                     // 初始化线程指针

    // 录制线路
    bool is_init_path_task = false;
    TaskState task_state; // 导航任务状态
    if (context->getPathTaskPtr()->initial())
    {
        info("线路任务初始化成功");
        is_init_path_task = true;
    }
    else
    {
        fatal("线路任务初始化失败");
        return 0;
    }

    ros::Time current_time = ros::Time::now();

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        // 先写个简单的，打印位置信息
        geometry_msgs::PoseStamped current_pose = context->getMessagePtr()->getCurrentPose();
        // info("current_pose is %f %f %f", current_pose.pose.position.x, current_pose.pose.position.y, tf::getYaw(current_pose.pose.orientation));

        // // 串口通信
        // if (context->getChassisPtr()->initial())
        // {
        //     context->getChassisPtr()->start();
        // }

        if (is_init_path_task)
        {
            context->getPathTaskPtr()->setCurrentPose(current_pose);
            task_state = context->getPathTaskPtr()->update(); // 导航任务状态
        }

        double dt = (ros::Time::now() - current_time).toSec();
        // info("dt is %f", dt);
        if (dt >= 3.0) // 每隔一段时间发布一次
        {
            // info("导航任务状态 task_state is %d", task_state);
            switch (task_state)
            {
            case 1:
                info("空闲状态");
                break;
            case 2:
                info("路线状态");
            case 3:
                info("跟线状态");
            case 4:
                info("旋转状态");
            default:
                break;
            }
            // info("线路任务状态 path_task_state_ is %d", context->getPathTaskPtr()->path_task_state_);

            current_time = ros::Time::now();
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
    delete context; ////释放空间
    info("导航程序关闭");

    return 0;
}
