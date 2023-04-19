#include <ros/ros.h>       //ros头文件
#include "logger/logger.h" //日志头文件
#include "context/context.h"

using namespace Sakura::Logger;

int main(int argc, char **argv)
{
    Logger::getInstance()->open("/home/ubuntu/log/navigation_frame.log");
    Logger::getInstance()->setMax(5120000); // log最大存储空间 5M
    // Logger::getInstance()->setLevel(Logger::INFO);
    // Logger::getInstance()->log(Logger::DEBUG,__FILE__,__LINE__,"hello Sakura");
    // Logger::getInstance()->log(Logger::DEBUG,__FILE__,__LINE__,"name is %s,age is %d","旋涡鸣人",18);

    // debug("hello Sakura");
    // info("info message");
    // warn("warn message");
    // erro("erro message");
    // fatal("fatal message");

    ros::init(argc, argv, "navigation_frame");
    ros::NodeHandle n;

    Nav::Context *context = new Nav::Context(); // //开辟空间
    context->initMissionPtr();                  // 初始化任务指针
    context->getPathMissionPtr()->initial();    // 初始化任务

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        context->getPathMissionPtr()->update(); // 循环执行

        ros::spinOnce();
        loop_rate.sleep();
    }
    delete context; ////释放空间
    info("程序关闭");

    return 0;
}
