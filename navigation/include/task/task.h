#ifndef _TASK_H_
#define _TASK_H_

#include "logger/logger.h"

namespace Nav
{
	// 基类
	class Task
	{
	public:
		Task(){};
		virtual ~Task(){};

		virtual bool initial() { return true; };
		virtual bool update() { return true; };
		virtual bool exit() { return true; };

	private:
	};
}

#endif