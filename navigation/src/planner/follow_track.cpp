#include "planner/follow_track.h"

using namespace Sakura::Logger;

namespace Nav
{
    FollowTrack::FollowTrack() // 构造函数
    {
        // info("构造函数");

        // nh_.getParam("car_velocity", car_velocity_);
        // nh_.getParam("point_distance", point_distance_);

        global_path_sub_ = nh_.subscribe("/path_line", 1, &FollowTrack::globalPathCallback, this); // 线路回调
        global_num_sub_ = nh_.subscribe("/path_num", 1, &FollowTrack::globalNumCallback, this);    // 线路标号回调

        // robot_car_info_pub_ = nh_.advertise<agv_robot::robot_car_info>("/robot_car_info_msg", 1);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    FollowTrack::~FollowTrack() // 析构函数
    {
        exit();
        // info("析构函数");
    }

    bool FollowTrack::initial()
    {
        // info("初始化");

        car_velocity_ = 0.4;
        point_distance_ = 0.3;

        r_x_.clear();
        r_y_.clear();

        point_num_ = 0; // 保存路径点的个数
        index_ = 1;
        global_num_ = 0;

        is_sub_path_ = false;

        Position_KP = 0.5; // 位置式PID控制器参数设定
        Position_KI = 0.0;
        Position_KD = 0.5;

        current_vel_x_ = car_velocity_;

        // follow_state_ == FollowTrackState::RUNNING; // 初始化

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
            case RUNNING:
            {
                // info("RUNNING");
                double remaining_dis = sqrt(pow(r_y_.back() - current_pose_.pose.position.y, 2) +
                                            pow(r_x_.back() - current_pose_.pose.position.x, 2));
                double position_motor = Position_PID(-remaining_dis, 0.0, Position_KP, Position_KI, Position_KD);
                double theta = current_vel_x_ * curvature_k_;

                if (is_pub_3D_speed_ && is_pub_2D_speed_ && is_pub_wave_speed_)
                {
                    geometry_msgs::Twist vel_msg;
                    vel_msg.linear.x = velLimit(position_motor, current_vel_x_);
                    vel_msg.angular.z = velLimit(theta, 1.0);
                    cmd_vel_pub_.publish(vel_msg);
                    // info("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);

                    return Result::RUN;
                }
                else
                {
                    // info("SCAN STOP or WAVE STOP");
                    geometry_msgs::Twist vel_msg;
                    vel_msg.linear.x = 0.0;
                    vel_msg.angular.z = velLimit(theta, 1.0);
                    cmd_vel_pub_.publish(vel_msg);
                    // ROS_INFO("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);

                    return Result::PAUSE;
                }
                break;
            }
            case ARRIVE_GOAL:
            {
                // info("ARRIVE_GOAL");
                geometry_msgs::Twist vel_msg;
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
                cmd_vel_pub_.publish(vel_msg);
                // ROS_INFO("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);

                return Result::SUCCEED;

                is_sub_path_ = false;
            }
            break;
            case STOP:
            {
                // info("STOP");
                geometry_msgs::Twist vel_msg;
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
                cmd_vel_pub_.publish(vel_msg);
                // ROS_INFO("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);

                return Result::PAUSE;
            }
            break;
            default:
                return Result::FAIL;
                break;
            }
        }
    }

    bool FollowTrack::exit()
    {
        // info("退出");
        return true;
    }

    void FollowTrack::setCurrentPose(const geometry_msgs::PoseStamped &msg)
    {
        current_pose_ = msg;
        // info("current_pose_ is %f %f %f", current_pose_.pose.position.x, current_pose_.pose.position.y, tf::getYaw(current_pose_.pose.orientation));
    }

    void FollowTrack::setIsPub3DSpeed(bool flag)
    {
        is_pub_3D_speed_ = flag;
    }

    void FollowTrack::setIsPub2DSpeed(bool flag)
    {
        is_pub_2D_speed_ = flag;
    }

    void FollowTrack::setIsPubWaveSpeed(bool flag)
    {
        is_pub_wave_speed_ = flag;
    }

    void FollowTrack::setCurrentSpeed(double vel)
    {
        current_vel_x_ = vel;
    }

    void FollowTrack::setCarStop(bool flag)
    {
        is_stop_car_ = flag;
    }

    void FollowTrack::globalPathCallback(const nav_msgs::Path &msg)
    {
        initial();

        point_num_ = msg.poses.size();
        // info("point_num_ is %d", point_num_);

        r_x_.clear();
        r_y_.clear();
        for (int i = 0; i < point_num_; i++)
        {
            r_x_.push_back(msg.poses[i].pose.position.x);
            r_y_.push_back(msg.poses[i].pose.position.y);
        }

        for (int i = 0; i < point_num_; i++)
        {
            double dis = sqrt(pow(r_x_[i] - current_pose_.pose.position.x, 2) +
                              pow(r_y_[i] - current_pose_.pose.position.y, 2));
            // info("i, r_x_, r_y_, dis is %d %f %f %f", i, r_x_[i], r_y_[i], dis);
        }

        is_sub_path_ = true;
    }

    void FollowTrack::globalNumCallback(const std_msgs::Int32 &msg)
    {
        global_num_ = msg.data;
    }

    FollowTrackState FollowTrack::calculateTrack()
    {
        double cal_yaw = tf::getYaw(current_pose_.pose.orientation);
        // info("cal_yaw  is %f", cal_yaw);

        // info("index_  is %d", index_);

        double alpha = angleLimit(atan2(r_y_[index_] - current_pose_.pose.position.y, r_x_[index_] - current_pose_.pose.position.x) - cal_yaw);
        // info("alpha  is %f", alpha);

        // 当前点和目标点的距离dl
        double dl = sqrt(pow(r_y_[index_] - current_pose_.pose.position.y, 2) +
                         pow(r_x_[index_] - current_pose_.pose.position.x, 2));
        // info("dl  is %f", dl);

        double dis = sqrt(pow(r_y_.back() - current_pose_.pose.position.y, 2) +
                          pow(r_x_.back() - current_pose_.pose.position.x, 2));
        // info("dis  is %f", dis);

        curvature_k_ = 2 * sin(alpha) / dl; // 跟踪曲率 k = 2 * sin(a) / Ld
        // std::cout << "curvature_k_ " << curvature_k_ << std::endl;

        if (is_stop_car_)
        {
            return FollowTrackState::STOP;
        }
        else
        {
            if (dl > point_distance_ && index_ < point_num_) // 行走过程中
            {
                return FollowTrackState::RUNNING;
            }
            else if (dl <= point_distance_ && index_ < point_num_) // 更新下一个坐标点
            {
                ++index_;
                return FollowTrackState::RUNNING;
            }
            else if (dis <= point_distance_ && index_ >= point_num_ - 1) // 到达终点
            {
                return FollowTrackState::ARRIVE_GOAL;
            }
            else
            {
                warn("NOTHING!");
                return FollowTrackState::STOP;
            }
        }
    }
}
