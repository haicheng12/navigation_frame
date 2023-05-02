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
        return 1;
    }

    Nav::Context *context = new Nav::Context(); // //开辟空间
    context->initTaskPtr();                     // 初始化线程指针

    TaskState task_state; // 导航任务状态
    // 线路任务
    bool is_init_path_task = false;
    if (context->getPathTaskPtr()->initial())
    {
        info("线路任务初始化成功");
        is_init_path_task = true;
    }
    else
    {
        fatal("线路任务初始化失败");
        return 1;
    }

    Result planner_state; // 规划器状态
    // 跟线规划器
    bool is_init_follow_track = false;
    if (context->getFollowTrackPtr()->initial())
    {
        info("跟线规划器初始化成功");
        is_init_follow_track = true;
    }
    else
    {
        fatal("跟线规划器初始化失败");
        return 1;
    }

    ros::Time current_time = ros::Time::now();

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        // 先写个简单的，打印位置信息
        geometry_msgs::PoseStamped current_pose = context->getMessagePtr()->getCurrentPose();
        // info("current_pose is %f %f %f", current_pose.pose.position.x, current_pose.pose.position.y, tf::getYaw(current_pose.pose.orientation));

        bool is_pub_3D_speed = context->getMessagePtr()->getIsPub3DSpeed();
        //  info("is_pub_3D_speed %d", is_pub_3D_speed);
        bool is_pub_2D_speed = context->getMessagePtr()->getIsPub2DSpeed();
        //  info("is_pub_2D_speed %d", is_pub_2D_speed);
        bool is_pub_wave_speed = context->getMessagePtr()->getIsPubWaveSpeed();
        //  info("is_pub_wave_speed %d", is_pub_wave_speed);

        double current_vel_x = context->getMissionPtr()->getSpeed(); // 当前速度
                                                                     // info("current_vel_x %f", current_vel_x);
        bool is_stop_car = context->getMissionPtr()->getCarStop();   // 是否停车
                                                                     // info("is_stop_car %d", is_stop_car);

        // // 串口通信
        // if (context->getChassisPtr()->initial())
        // {
        //     context->getChassisPtr()->start();
        // }

        if (is_init_path_task && is_init_follow_track)
        {
            context->getPathTaskPtr()->setCurrentPose(current_pose);

            context->getFollowTrackPtr()->setCurrentPose(current_pose);
            context->getFollowTrackPtr()->setIsPub3DSpeed(is_pub_3D_speed);
            context->getFollowTrackPtr()->setIsPub2DSpeed(is_pub_2D_speed);
            context->getFollowTrackPtr()->setIsPubWaveSpeed(is_pub_wave_speed);
            context->getFollowTrackPtr()->setCurrentSpeed(current_vel_x);
            context->getFollowTrackPtr()->setCarStop(is_stop_car);

            task_state = context->getPathTaskPtr()->update();       // 导航任务状态
            planner_state = context->getFollowTrackPtr()->update(); // 规划器状态
        }

        double dt = (ros::Time::now() - current_time).toSec();
        // info("dt is %f", dt);
        if (dt >= 1.0) // 每隔一段时间发布一次
        {
            // info("导航任务状态 task_state is %d", task_state);
            // switch (task_state)
            // {
            // case 1:
            //     info("空闲状态");
            //     break;
            // case 2:
            //     info("路线状态");
            //     break;
            // case 3:
            //     info("跟线状态");
            //     break;
            // case 4:
            //     info("旋转状态");
            //     break;
            // default:
            //     break;
            // }

            // info("规划器状态 planner_state is %d", planner_state);
            switch (planner_state)
            {
            case 1:
                info("跟线成功");
                break;
            case 2:
                info("跟线正在进行");
                break;
            case 3:
                info("跟线暂停");
                break;
            case 4:
                info("跟线失败");
                break;
            default:
                break;
            }
            current_time = ros::Time::now();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    delete context; ////释放空间
    info("导航程序关闭");

    return 0;
}
