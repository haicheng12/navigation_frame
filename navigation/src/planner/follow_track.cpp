#include "planner/follow_track.h"

using namespace Sakura::Logger;

namespace Nav
{
    FollowTrack::FollowTrack() // 构造函数
    {
        // info("构造函数");

        nh_.getParam("car_velocity", car_velocity_);
        nh_.getParam("point_distance", point_distance_);
        nh_.getParam("scan_distance_3D", scan_distance_3D_);
        nh_.getParam("scan_distance_2D", scan_distance_2D_);
        nh_.getParam("wave_distance", wave_distance_);

        global_path_sub_ = nh_.subscribe("/path_line", 1, &FollowTrack::globalPathCallback, this); // 线路回调
        global_num_sub_ = nh_.subscribe("/path_num", 1, &FollowTrack::globalNumCallback, this);    // 线路标号回调
        set_speed_sub_ = nh_.subscribe("/set_speed", 1, &FollowTrack::setSpeed, this);             // 设定机器人速度
        car_stop_sub_ = nh_.subscribe("/car_stop", 1, &FollowTrack::carStop, this);                // 是否暂停小车

        scan_3D_sub_ = nh_.subscribe("/scan", 20, &FollowTrack::scan3DCallback, this);    // 雷达回调
        scan_2D_sub_ = nh_.subscribe("/scan_2D", 20, &FollowTrack::scan2DCallback, this); // 雷达回调
        wave_sub_ = nh_.subscribe("/sonar", 10, &FollowTrack::waveCallback, this);        // 超声波回调

        // robot_car_info_pub_ = nh_.advertise<agv_robot::robot_car_info>("/robot_car_info_msg", 1);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    FollowTrack::~FollowTrack() // 析构函数
    {
        exit();
        info("析构函数");
    }

    bool FollowTrack::initial()
    {
        info("初始化");

        r_x_.clear();
        r_y_.clear();

        pointNum = 0; // 保存路径点的个数
        index_ = 1;
        global_num_ = 0;

        is_sub_path_ = false;

        Position_KP = 0.5; // 位置式PID控制器参数设定
        Position_KI = 0.0;
        Position_KD = 0.5;

        current_vel_x_ = car_velocity_;

        smallest_3D_distance_ = 10.0; // 定义了一个接收激光雷达扫描的距离
        smallest_2D_distance_ = 10.0;
        is_pub_3D_speed_ = true;
        is_pub_2D_speed_ = true;
        is_pub_wave_speed_ = true;

        is_stop_car_ = false;

        follow_state_ == FollowTrackState::BEING; // 初始化为正常行使状态

        return true;
    }

    Result FollowTrack::update()
    {
        // info("循环执行");

        if (is_sub_path_)
        {
            follow_state_ = calculateTrack();

            switch (follow_state_)
            {
            case BEING: // 再行走的过程中添加旋转控制
            {
                info("BEING");
                double remaining_dis = sqrt(pow(r_y_.back() - current_pose_msg_.pose.position.y, 2) +
                                            pow(r_x_.back() - current_pose_msg_.pose.position.x, 2));
                double position_motor = Position_PID(-remaining_dis, 0.0, Position_KP, Position_KI, Position_KD);
                double theta = current_vel_x_ * curvature_k_;

                if (is_pub_3D_speed_ && is_pub_2D_speed_ && is_pub_wave_speed_)
                {
                    geometry_msgs::Twist vel_msg;
                    vel_msg.linear.x = Xianfu(position_motor, current_vel_x_);
                    vel_msg.angular.z = Xianfu(theta, 1.0);
                    cmd_vel_pub_.publish(vel_msg);
                    info("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);

                    return Result::RUN;
                }
                else
                {
                    info("SCAN STOP or WAVE STOP");
                    geometry_msgs::Twist vel_msg;
                    vel_msg.linear.x = 0.0;
                    vel_msg.angular.z = Xianfu(theta, 1.0);
                    cmd_vel_pub_.publish(vel_msg);
                    // ROS_INFO("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);

                    return Result::PAUSE;
                }
                return Result::RUN;
                break;
            }
            case ARRIVE_GOAL:
            {
                info("ARRIVE_GOAL");
                geometry_msgs::Twist vel_msg;
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
                cmd_vel_pub_.publish(vel_msg);
                // ROS_INFO("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);

                is_sub_path_ = false;

                return Result::SUCCEED;

                break;
            }
            case STOP:
            {
                info("STOP");
                geometry_msgs::Twist vel_msg;
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
                cmd_vel_pub_.publish(vel_msg);
                // ROS_INFO("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);

                return Result::PAUSE;
            }
            default:
                return Result::FAIL;
                break;
            }
        }

        return Result::SUCCEED;
    }

    bool FollowTrack::exit()
    {
        info("退出");
        return true;
    }

    void FollowTrack::setCurrentPose(const geometry_msgs::PoseStamped &msg)
    {
        current_pose_msg_ = msg;
    }

    void FollowTrack::scan3DCallback(const sensor_msgs::LaserScanConstPtr &msg)
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

    void FollowTrack::scan2DCallback(const sensor_msgs::LaserScan::ConstPtr &scan) // 雷达回调
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

    void FollowTrack::waveCallback(const std_msgs::UInt16MultiArrayConstPtr &msg) // 超声波回调
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

    void FollowTrack::setSpeed(const std_msgs::Float64 &msg) // 设定速度
    {
        current_vel_x_ = msg.data;
    }

    void FollowTrack::carStop(const std_msgs::Bool &msg)
    {
        is_stop_car_ = msg.data;
    }

    void FollowTrack::globalPathCallback(const nav_msgs::Path &msg)
    {
        initial();

        pointNum = msg.poses.size();
        info("pointNum is %d", pointNum);

        r_x_.clear();
        r_y_.clear();
        for (int i = 0; i < pointNum; i++)
        {
            r_x_.push_back(msg.poses[i].pose.position.x);
            r_y_.push_back(msg.poses[i].pose.position.y);
        }

        for (int i = 0; i < pointNum; i++)
        {
            double dis = sqrt(pow(r_x_[i] - current_pose_msg_.pose.position.x, 2) +
                              pow(r_y_[i] - current_pose_msg_.pose.position.y, 2));
            info("i, r_x_, r_y_, dis is %d %f %f %f", i, r_x_[i], r_y_[i], dis);
        }

        is_sub_path_ = true;
    }

    void FollowTrack::globalNumCallback(const std_msgs::Int32 &msg)
    {
        global_num_ = msg.data;
    }

    FollowTrackState FollowTrack::calculateTrack()
    {
        double calRPY = tf::getYaw(current_pose_msg_.pose.orientation);
        // info("calRPY  is %f", calRPY);

        info("index_  is %d", index_);

        double alpha = angleLimit(atan2(r_y_[index_] - current_pose_msg_.pose.position.y, r_x_[index_] - current_pose_msg_.pose.position.x) - calRPY);
        info("alpha  is %f", alpha);

        // 当前点和目标点的距离dl
        double dl = sqrt(pow(r_y_[index_] - current_pose_msg_.pose.position.y, 2) +
                         pow(r_x_[index_] - current_pose_msg_.pose.position.x, 2));
        info("dl  is %f", dl);

        double dis = sqrt(pow(r_y_.back() - current_pose_msg_.pose.position.y, 2) +
                          pow(r_x_.back() - current_pose_msg_.pose.position.x, 2));
        info("dis  is %f", dis);

        curvature_k_ = 2 * sin(alpha) / dl; // 跟踪曲率 k = 2 * sin(a) / Ld
        // std::cout << "curvature_k_ " << curvature_k_ << std::endl;

        if (is_stop_car_)
        {
            return FollowTrackState::STOP;
        }
        else
        {
            if (dl > point_distance_ && index_ < pointNum) // 行走过程中
            {
                return FollowTrackState::BEING;
            }
            else if (dl <= point_distance_ && index_ < pointNum) // 更新下一个坐标点
            {
                ++index_;
                return FollowTrackState::BEING;
            }
            else if (dis <= point_distance_ && index_ >= pointNum - 1) // 到达终点
            {
                return FollowTrackState::ARRIVE_GOAL;
            }
            else
            {
                warn("NOTHING!");
                return FollowTrackState::STOP;
            }
        }
        return FollowTrackState::BEING;
    }
}
