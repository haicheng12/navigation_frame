#ifndef _CONTEXT_H_
#define _CONTEXT_H_

#include <thread>
#include <mutex>
#include "logger/logger.h"
#include "task/task.h"
#include "task/path_task.h"
#include "planner/planner.h"
#include "planner/follow_track.h"
#include "planner/rotation.h"
#include "message/message.h"
#include "chassis/chassis.h"
#include "mission/mission.h"

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

			message_ptr_ = std::make_shared<Message>();
			// chassis_ptr_ = std::make_shared<Chassis>();
			path_task_ptr_ = std::make_shared<PathTask>();
			follow_track_ptr_ = std::make_shared<FollowTrack>();
			mission_ptr_ = std::make_shared<Mission>();

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

		std::shared_ptr<Chassis> getChassisPtr() // 获取串口通信的线程
		{
			if (!chassis_ptr_)
			{
				warn("chassis_ptr_ is empty!");
			}
			return chassis_ptr_;
		}

		std::shared_ptr<FollowTrack> getFollowTrackPtr() // 获取跟线的线程
		{
			if (!follow_track_ptr_)
			{
				warn("follow_track_ptr_ is empty!");
			}
			return follow_track_ptr_;
		}

		std::shared_ptr<Mission> getMissionPtr() // 获取任务的线程
		{
			if (!mission_ptr_)
			{
				warn("mission_ptr_ is empty!");
			}
			return mission_ptr_;
		}

	private:
		std::shared_ptr<PathTask> path_task_ptr_;		// 录制线路任务的线程
		std::shared_ptr<Message> message_ptr_;			// 获取传感器信息的线程
		std::shared_ptr<Chassis> chassis_ptr_;			// 串口通信的线程
		std::shared_ptr<FollowTrack> follow_track_ptr_; // 跟线的线程
		std::shared_ptr<Mission> mission_ptr_;			// 任务的线程

		std::mutex ptr_mutex_; // 线程锁
	};
}

#endif