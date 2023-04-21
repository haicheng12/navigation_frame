#include "message/message.h"

using namespace Sakura::Logger;

namespace Nav
{
    Message::Message() // 构造函数
    {
        info("构造函数");

        nh_.getParam("is_use_sim", is_use_sim_);
        nh_.getParam("interval", interval_);

        is_pose_sub_ = false;

        if (is_use_sim_) // 使用仿真
        {
            odom_sub_ = nh_.subscribe("/odom", 10, &Message::odomCallback, this); // 里程计
        }
        else // 雷达定位
        {
            current_pose_sub_ = nh_.subscribe("/ndt_pose", 10, &Message::currentPoseCallback, this); // 回调当前位置
        }
    }

    Message::~Message() // 析构函数
    {
        info("析构函数");
    }

    void Message::odomCallback(const nav_msgs::OdometryConstPtr &msg) // 回调当前位置
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        current_pose_.pose = msg->pose.pose;

        is_pose_sub_ = true;
    }

    void Message::currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) // 回调当前位置
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        current_pose_.pose = msg->pose;

        is_pose_sub_ = true;
    }
}