#ifndef _TASK_H_
#define _TASK_H_

#include "logger/logger.h"

enum TaskState : u_int8_t // 任务状态
{
	IDEL = 1, // 空闲
	PATH,	  // 线路
	FOLLOW,	  // 跟线
	ROTATION  // 旋转
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
		virtual TaskState update() { return TaskState::IDEL; };
		virtual bool exit() { return true; };

	private:
	};
}

#endif