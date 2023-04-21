#ifndef _CONTEXT_H_
#define _CONTEXT_H_

#include <thread>
#include <mutex>
#include "logger/logger.h"
#include "task/task.h"
#include "task/path_task.h"
#include "message/message.h"

using namespace Sakura::Logger;

namespace Nav
{
	class Context
	{
	public:
		Context(){};
		virtual ~Context(){};

		bool initTaskPtr() // 初始化任务线程
		{
			std::lock_guard<std::mutex> lock(ptr_mutex_);

			path_task_ptr_ = std::make_shared<PathTask>();
			message_ptr_ = std::make_shared<Message>();

			return true;
		};

		std::shared_ptr<PathTask> getPathTaskPtr() // 获取录制线路任务的线程
		{
			if (!path_task_ptr_)
			{
				warn("path_task_ptr_ is empty!");
			}
			return path_task_ptr_;
		}

		std::shared_ptr<Message> getMessagePtr() // 获取传感器信息的线程
		{
			if (!message_ptr_)
			{
				warn("message_ptr_ is empty!");
			}
			return message_ptr_;
		}

	private:
		std::shared_ptr<PathTask> path_task_ptr_; // 录制线路任务的线程
		std::shared_ptr<Message> message_ptr_;		// 获取传感器信息的线程

		std::mutex ptr_mutex_; // 线程锁
	};
}

#endif