#ifndef _MISSION_H_
#define _MISSION_H_

#include <iostream>
#include "logger/logger.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

namespace Nav
{
	class Mission
	{
	public:
		Mission();
		virtual ~Mission();

		double getSpeed(); // 获取当前速度
		bool getCarStop(); // 获取暂停状态

	private:
		// ROS messages (topics)
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Subscriber set_speed_sub_;
		ros::Subscriber car_stop_sub_;

		void setSpeed(const std_msgs::Float64 &msg); // 设定速度
		void carStop(const std_msgs::Bool &msg);	 // 小车暂停命令

		double current_vel_x_; // 当前速度
		bool is_stop_car_;	   // 是否停车
	};
}

#endif
