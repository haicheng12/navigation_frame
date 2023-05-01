#ifndef _UTILITY_TOOL_H_
#define _UTILITY_TOOL_H_

#include "logger/logger.h" //日志头文件

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
}

#endif
