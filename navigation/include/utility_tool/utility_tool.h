#ifndef _UTILITY_TOOL_H_
#define _UTILITY_TOOL_H_

#include "logger/logger.h" //日志头文件
#include "ros/ros.h"	   //

using namespace Sakura::Logger;

namespace Nav
{
	static bool initLogger() // 初始化日志系统
	{
		Logger::getInstance()->open("/home/ubuntu/log/navigation.log");
		Logger::getInstance()->setMax(5120000); // log最大存储空间 5M
												// Logger::getInstance()->setLevel(Logger::INFO);
												// Logger::getInstance()->log(Logger::DEBUG,__FILE__,__LINE__,"hello Sakura");
												// Logger::getInstance()->log(Logger::DEBUG,__FILE__,__LINE__,"name is %s,age is %d","旋涡鸣人",18);

		debug("hello logger");
		info("info message");
		warn("warn message");
		erro("erro message");
		fatal("fatal message");

		return true;
	};

	inline double angleLimit(double yaw) // 角度限制
	{
		double theta = yaw;
		if (theta > M_PI)
		{
			theta = theta - 2 * M_PI;
		}
		else if (theta < -M_PI)
		{
			theta = theta + 2 * M_PI;
		}
		return theta;
	}

	static double Xianfu(double value, double Amplitude) // 限制最大速度幅度
	{
		double temp;
		if (value > Amplitude)
			temp = Amplitude;
		else if (value < -Amplitude)
			temp = -Amplitude;
		else
			temp = value;

		return temp;
	}

	// 函数功能：位置式PID控制器
	// 入口参数：当前位置，目标位置
	// 返回  值：控制的速度
	// 根据位置式离散PID公式
	// Speed=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
	// e(k)代表本次偏差
	// e(k-1)代表上一次的偏差
	// ∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
	// Speed代表输出
	static double Position_PID(double Position, double target, double Position_KP, double Position_KI, double Position_KD)
	{
		static double Bias, Speed, Integral_bias, Last_Bias;
		Bias = target - Position; // 计算偏差
		Integral_bias += Bias;	  // 求出偏差的积分
		Integral_bias = Xianfu(Integral_bias, 0.5);
		Speed = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias); // 位置式PID控制器
		Last_Bias = Bias;																			 // 保存上一次偏差

		if (Speed > 0.5)
			Speed = 0.5;
		if (Speed < -0.5)
			Speed = -0.5;

		return Speed; // 增量输出
	}
}

#endif
