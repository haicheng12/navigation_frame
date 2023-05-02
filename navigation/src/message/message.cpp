#include "message/message.h"

using namespace Sakura::Logger;

namespace Nav
{
    Message::Message() // 构造函数
    {
        info("构造函数");

        nh_.getParam("is_use_sim", is_use_sim_);
        nh_.getParam("scan_distance_3D", scan_distance_3D_);
        nh_.getParam("scan_distance_2D", scan_distance_2D_);
        nh_.getParam("wave_distance", wave_distance_);

        initial(); // 初始化

        // if (is_use_sim_) // 使用仿真
        // {
        odom_sub_ = nh_.subscribe("/odom", 10, &Message::odomCallback, this); // 里程计
        // }
        // else // 雷达定位
        // {
        //     current_pose_sub_ = nh_.subscribe("/ndt_pose", 10, &Message::currentPoseCallback, this); // 回调当前位置
        //     scan_3D_sub_ = nh_.subscribe("/scan", 20, &Message::scan3DCallback, this);               // 雷达回调
        //     scan_2D_sub_ = nh_.subscribe("/scan_2D", 20, &Message::scan2DCallback, this);            // 雷达回调
        //     wave_sub_ = nh_.subscribe("/sonar", 10, &Message::waveCallback, this);                   // 超声波回调
        // }
    }

    Message::~Message() // 析构函数
    {
        info("析构函数");
    }

    void Message::initial() // 初始化
    {
        is_odom_sub_ = false;

        smallest_3D_distance_ = 10.0; // 定义了一个接收激光雷达扫描的距离
        smallest_2D_distance_ = 10.0;

        is_pub_3D_speed_ = true;
        is_pub_2D_speed_ = true;
        is_pub_wave_speed_ = true;
    }

    void Message::odomCallback(const nav_msgs::OdometryConstPtr &msg) // 回调当前位置
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        current_pose_.pose = msg->pose.pose;
        current_vel_.twist = msg->twist.twist;

        is_odom_sub_ = true;
    }

    void Message::currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) // 回调当前位置
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        current_pose_.pose = msg->pose;

        is_odom_sub_ = true;
    }

    void Message::scan3DCallback(const sensor_msgs::LaserScanConstPtr &msg)
    {
        int arr_size = floor((msg->angle_max - msg->angle_min) / msg->angle_increment);
        for (int i = 900; i < 1240; i++) // 滤波 90
        {
            if (msg->ranges[i] < 0.1) // 滤波
            {
                continue;
            }
            if (msg->ranges[i] < smallest_3D_distance_)
            {
                smallest_3D_distance_ = msg->ranges[i];
            }
        }

        if (smallest_3D_distance_ <= scan_distance_3D_) // 雷达避障的距离
        {
            info("多线雷达停障");
            is_pub_3D_speed_ = false;
        }
        else
        {
            is_pub_3D_speed_ = true;
        }
        smallest_3D_distance_ = 10.0;
    }

    void Message::scan2DCallback(const sensor_msgs::LaserScan::ConstPtr &scan) // 雷达回调
    {
        int arr_size = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);
        for (int i = 1200; i < 1400; i++) // right
        {
            if (scan->ranges[i] < smallest_2D_distance_)
            {
                smallest_2D_distance_ = scan->ranges[i];
            }
        }
        for (int i = 0; i < 200; i++) // left
        {
            if (scan->ranges[i] < smallest_2D_distance_)
            {
                smallest_2D_distance_ = scan->ranges[i];
            }
        }

        if (smallest_2D_distance_ <= scan_distance_2D_) // 雷达避障的距离#1.0
        {
            info("单线雷达停障");
            is_pub_2D_speed_ = false;
        }
        else
        {
            is_pub_2D_speed_ = true;
        }
        smallest_2D_distance_ = 10.0;
    }

    void Message::waveCallback(const std_msgs::UInt16MultiArrayConstPtr &msg) // 超声波回调
    {
        int data0 = msg->data[0]; // 前1
        int data1 = msg->data[1]; // 前2
        int data2 = msg->data[2]; // 前3
        int data3 = msg->data[3]; // 前4
        int data4 = msg->data[4]; // 后1
        int data5 = msg->data[5]; // 后2
        int data6 = msg->data[6]; // 后3
        int data7 = msg->data[7]; // 后4
        if ((data0 > 0 && data0 < wave_distance_) || (data1 > 0 && data1 < wave_distance_) || (data2 > 0 && data2 < wave_distance_) || (data3 > 0 && data3 < wave_distance_) || (data4 > 0 && data4 < wave_distance_) || (data5 > 0 && data5 < wave_distance_) || (data6 > 0 && data6 < wave_distance_) || (data7 > 0 && data7 < wave_distance_))
        {
            info("超声波停障");
            is_pub_wave_speed_ = false;
        }
        else
        {
            is_pub_wave_speed_ = true;
        }
    }

    bool Message::getIsPub3DSpeed() // 获取发布速度状态
    {
        return is_pub_3D_speed_;
    }

    bool Message::getIsPub2DSpeed() // 获取发布速度状态
    {
        return is_pub_2D_speed_;
    }

    bool Message::getIsPubWaveSpeed() // 获取发布速度状态
    {
        return is_pub_wave_speed_;
    }
}