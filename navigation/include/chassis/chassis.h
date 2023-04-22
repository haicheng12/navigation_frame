#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include <iostream>
#include <mutex>
#include "logger/logger.h"
#include <ros/ros.h>
#include <serial/serial.h>

using namespace Sakura::Logger;

namespace Nav
{
	class Chassis
	{
	public:
		Chassis();
		virtual ~Chassis();

		bool initial();
		void start();

	private:
		ros::NodeHandle nh_;
		// 创建一个serial类
		serial::Serial sp_;
	};
}

#endif
