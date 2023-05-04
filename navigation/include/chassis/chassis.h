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
		serial::Serial ser_;

		// data 为发送数据
		// result 为接收数据
		std::string data_, state_, result_;
		int func_;

		int serialWrite(serial::Serial &ser, std::string &serial_msg);
		int serialRead(serial::Serial &ser, std::string &result);
	};
}

#endif
