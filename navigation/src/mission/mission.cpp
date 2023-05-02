#include "mission/mission.h"

using namespace Sakura::Logger;

namespace Nav
{
    Mission::Mission() // 构造函数
    {
        // info("构造函数");
        is_stop_car_ = false;
        current_vel_x_ = 0.4; // 初始速度

        set_speed_sub_ = nh_.subscribe("/set_speed", 1, &Mission::setSpeed, this); // 设定机器人速度
        car_stop_sub_ = nh_.subscribe("/car_stop", 1, &Mission::carStop, this);    // 是否暂停小车
    }

    Mission::~Mission() // 析构函数
    {
        // info("析构函数");
    }

    void Mission::setSpeed(const std_msgs::Float64 &msg) // 设定速度
    {
        current_vel_x_ = msg.data;
    }

    void Mission::carStop(const std_msgs::Bool &msg) // 是否停车
    {
        is_stop_car_ = msg.data;
    }

    double Mission::getSpeed() // 获取当前速度
    {
        return current_vel_x_;
    }

    bool Mission::getCarStop() // 获取暂停状态
    {
        return is_stop_car_;
    }
}