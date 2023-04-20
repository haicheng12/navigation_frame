#ifndef _TASK_H_
#define _TASK_H_

#include "logger/logger.h"

enum Result : u_int8_t // 任务状态
{
	SUCCEED = 1, // 成功
	FAILED,		 // 失败
};

namespace Nav
{
	// 基类
	class Task
	{
	public:
		Task(){};
		virtual ~Task(){};

		virtual bool initial() { return true; };
		virtual Result update() { return Result::SUCCEED; };
		virtual bool exit() { return true; };

	private:
	};
}

#endif