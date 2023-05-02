#ifndef _PLANNER_H_
#define _PLANNER_H_

#include "logger/logger.h"

enum Result : u_int8_t // 任务状态
{
	SUCCEED = 1, // 成功
	RUN,		 // 正在进行
	PAUSE,		 // 暂停
	FAIL		 // 失败
};

namespace Nav
{
	// 基类
	class Planner
	{
	public:
		Planner(){};
		virtual ~Planner(){};

		virtual bool initial() { return true; };
		virtual Result update() { return Result::SUCCEED; };
		virtual bool exit() { return true; };

	private:
	};
}

#endif