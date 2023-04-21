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
    ros::NodeHandle n;

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

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        geometry_msgs::PoseStamped current_pose = context->getMessagePtr()->getCurrentPose();
        info("current_pose is %f %f %f", current_pose.pose.position.x, current_pose.pose.position.y, tf::getYaw(current_pose.pose.orientation));

        ros::spinOnce();
        loop_rate.sleep();
    }
    delete context; ////释放空间
    info("导航程序关闭");

    return 0;
}
