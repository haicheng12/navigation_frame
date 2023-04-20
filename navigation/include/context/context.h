#ifndef _CONTEXT_H_
#define _CONTEXT_H_

#include <thread>
#include "logger/logger.h"
#include "task/task.h"
#include "task/path_task.h"

using namespace Sakura::Logger;

namespace Nav
{
	class Context
	{
	public:
		Context(){};
		virtual ~Context(){};

		bool initTaskPtr()
		{
			path_task_ptr_ = std::make_shared<PathTask>();
			return true;
		};

		std::shared_ptr<PathTask> getPathTaskPtr()
		{
			if (!path_task_ptr_)
			{
				warn("path_task_ptr_ is empty!");
			}
			return path_task_ptr_;
		}

	private:
		std::shared_ptr<PathTask> path_task_ptr_;
	};
}

#endif