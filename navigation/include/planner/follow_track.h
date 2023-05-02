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
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16MultiArray.h"

enum FollowTrackState : u_int8_t // 跟线状态
{
	BEING = 1,	 // 正常行驶
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

		geometry_msgs::PoseStamped current_pose_msg_; // 当前位置

	private:
		// ROS messages (topics)
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Subscriber global_path_sub_;
		ros::Subscriber global_num_sub_;
		ros::Subscriber set_speed_sub_;
		ros::Subscriber car_stop_sub_;

		ros::Subscriber scan_3D_sub_;
		ros::Subscriber scan_2D_sub_;
		ros::Subscriber wave_sub_;

		ros::Publisher robot_car_info_pub_;
		ros::Publisher cmd_vel_pub_;

		void pubRobotCarInfo();											  // 发布小车状态信息
		void globalPathCallback(const nav_msgs::Path &msg);				  // 线路回调
		void globalNumCallback(const std_msgs::Int32 &msg);				  // 线路标号回调
		void setSpeed(const std_msgs::Float64 &msg);					  // 设定速度
		void carStop(const std_msgs::Bool &msg);						  // 小车暂停命令
		void scan3DCallback(const sensor_msgs::LaserScanConstPtr &msg);	  // 雷达回调
		void scan2DCallback(const sensor_msgs::LaserScanConstPtr &msg);	  // 雷达回调
		void waveCallback(const std_msgs::UInt16MultiArrayConstPtr &msg); // 超声波回调

		FollowTrackState calculateTrack();
		FollowTrackState follow_state_;

		geometry_msgs::TwistStamped current_vel_msg_; // 当前速度

		std::vector<double> r_x_;
		std::vector<double> r_y_;

		int pointNum; // 保存路径点的个数
		int index_;

		double curvature_k_;

		bool is_sub_path_;

		double Position_KP; // 位置式PID控制器参数设定
		double Position_KI;
		double Position_KD;

		int global_num_;
		double current_vel_x_;

		double smallest_3D_distance_; // 定义了一个接收激光雷达扫描的距离
		double smallest_2D_distance_;
		bool is_pub_3D_speed_;
		bool is_pub_2D_speed_;
		bool is_pub_wave_speed_;

		bool is_stop_car_;

		// 参数加载
		double car_velocity_;
		double point_distance_;
		double scan_distance_3D_;
		double scan_distance_2D_;
		int wave_distance_;
	};
}

#endif