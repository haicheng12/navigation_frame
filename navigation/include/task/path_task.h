#ifndef _PATH_TASK_H_
#define _PATH_TASK_H_

#include "task.h"

namespace Nav
{
	// 派生类
	class PathTask : public Task
	{
	public:
		PathTask();
		virtual ~PathTask();

		bool initial() override;
		Result update() override;
		bool exit() override;

	private:
	};
}

#endif