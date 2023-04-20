#include <ros/ros.h>       //ros头文件
#include "logger/logger.h" //日志头文件
#include "context/context.h"
#include "utility_tool/utility_tool.h"

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
    }

    Nav::Context *context = new Nav::Context(); // //开辟空间
    context->initTaskPtr();                     // 初始化任务指针
    context->getPathTaskPtr()->initial();       // 初始化任务

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        context->getPathTaskPtr()->update(); // 循环执行

        ros::spinOnce();
        loop_rate.sleep();
    }
    delete context; ////释放空间
    info("程序关闭");

    return 0;
}
