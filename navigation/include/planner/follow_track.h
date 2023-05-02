#ifndef _FOLLOW_TRACK_H_
#define _FOLLOW_TRACK_H_

#include "planner.h"
#include "utility_tool/utility_tool.h"

#include <iostream>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

enum FollowTrackState : u_int8_t // 跟线状态
{
	RUNNING = 1, // 正常行驶
	ARRIVE_GOAL, // 到达终点
	STOP		 // 暂停行使
};

namespace Nav
{
	// 派生类
	class FollowTrack : public Planner
	{
	public:
		FollowTrack();
		virtual ~FollowTrack();

		bool initial() override;
		Result update() override;
		bool exit() override;

		void setCurrentPose(const geometry_msgs::PoseStamped &msg);
		void setIsPub3DSpeed(bool flag);
		void setIsPub2DSpeed(bool flag);
		void setIsPubWaveSpeed(bool flag);
		void setCurrentSpeed(double vel);
		void setCarStop(bool);

		geometry_msgs::PoseStamped current_pose_; // 当前位置
		geometry_msgs::TwistStamped current_vel_; // 当前速度

	private:
		// ROS messages (topics)
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Subscriber global_path_sub_;
		ros::Subscriber global_num_sub_;

		ros::Publisher robot_car_info_pub_;
		ros::Publisher cmd_vel_pub_;

		void pubRobotCarInfo();								// 发布小车状态信息
		void globalPathCallback(const nav_msgs::Path &msg); // 线路回调
		void globalNumCallback(const std_msgs::Int32 &msg); // 线路标号回调

		FollowTrackState calculateTrack();
		FollowTrackState follow_state_;

		std::vector<double> r_x_;
		std::vector<double> r_y_;

		int point_num_; // 保存路径点的个数
		int index_;

		double curvature_k_;

		bool is_sub_path_;

		double Position_KP; // 位置式PID控制器参数设定
		double Position_KI;
		double Position_KD;

		int global_num_;

		bool is_pub_3D_speed_;
		bool is_pub_2D_speed_;
		bool is_pub_wave_speed_;

		double current_vel_x_; // 当前速度
		bool is_stop_car_;	   // 是否停车

		// 参数加载
		double car_velocity_;
		double point_distance_;
	};
}

#endif