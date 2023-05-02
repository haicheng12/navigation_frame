#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include <iostream>
#include <mutex>
#include "logger/logger.h"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/UInt16MultiArray.h"
#include "sensor_msgs/LaserScan.h"

using namespace Sakura::Logger;

namespace Nav
{
	class Message
	{
	public:
		Message();
		virtual ~Message();

		geometry_msgs::PoseStamped getCurrentPose() // 获取当前位置
		{
			if (!is_odom_sub_)
			{
				warn("current pose data has error !!!");
			}
			return current_pose_;
		};

		geometry_msgs::TwistStamped getCurrentVel() // 获取当前速度
		{
			if (!is_odom_sub_)
			{
				warn("current vel data has error !!!");
			}
			return current_vel_;
		};

		bool getIsPub3DSpeed();	  // 获取发布速度状态
		bool getIsPub2DSpeed();	  // 获取发布速度状态
		bool getIsPubWaveSpeed(); // 获取发布速度状态

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Subscriber odom_sub_;		   // 接收里程计
		ros::Subscriber current_pose_sub_; // 接收当前位置

		ros::Subscriber scan_3D_sub_; // 接收多线雷达数据
		ros::Subscriber scan_2D_sub_; // 接收单线雷达数据
		ros::Subscriber wave_sub_;	  // 接收超声波数据

		void initial();															 // 初始化
		void odomCallback(const nav_msgs::OdometryConstPtr &msg);				 // 里程计
		void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg); // 回调当前位置
		void scan3DCallback(const sensor_msgs::LaserScanConstPtr &msg);			 // 雷达回调
		void scan2DCallback(const sensor_msgs::LaserScanConstPtr &msg);			 // 雷达回调
		void waveCallback(const std_msgs::UInt16MultiArrayConstPtr &msg);		 // 超声波回调

		geometry_msgs::PoseStamped current_pose_; // 当前位置
		geometry_msgs::TwistStamped current_vel_; // 当前速度

		std::mutex pose_mutex_;

		bool is_odom_sub_;

		double smallest_3D_distance_; // 定义了一个接收激光雷达扫描的距离
		double smallest_2D_distance_;

		bool is_pub_3D_speed_;
		bool is_pub_2D_speed_;
		bool is_pub_wave_speed_;

		// param参数加载
		double is_use_sim_;
		double scan_distance_3D_;
		double scan_distance_2D_;
		int wave_distance_;
	};
}

#endif
