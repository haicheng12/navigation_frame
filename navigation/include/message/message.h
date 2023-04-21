#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include <iostream>
#include <mutex>
#include "logger/logger.h"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

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
			if (!is_pose_sub_)
			{
				warn("current pose data has error !!!");
			}
			return current_pose_;
		};

	private:
		ros::NodeHandle nh_;

		ros::Subscriber odom_sub_;				 // 接收里程计
		ros::Subscriber current_pose_sub_; // 接收当前位置

		void odomCallback(const nav_msgs::OdometryConstPtr &msg);								 // 里程计
		void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg); // 回调当前位置

		geometry_msgs::PoseStamped current_pose_;
		bool is_pose_sub_;
		std::mutex pose_mutex_;

		// param参数加载
		double interval_;
		bool is_use_sim_;
	};
}

#endif
