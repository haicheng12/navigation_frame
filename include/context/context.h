#ifndef _CONTEXT_H_
#define _CONTEXT_H_

#include <thread>
#include "logger/logger.h"
#include "mission/mission.h"
#include "mission/path_mission.h"

using namespace Sakura::Logger;

namespace Nav
{
	class Context
	{
	public:
		Context(){};
		virtual ~Context(){};

		bool initMissionPtr()
		{
			path_mission_ptr_ = std::make_shared<PathMission>();
			return true;
		};

		std::shared_ptr<PathMission> getPathMissionPtr()
		{
			if (!path_mission_ptr_)
			{
				warn("path_mission_ptr_ is empty!");
			}
			return path_mission_ptr_;
		}

	private:
		std::shared_ptr<PathMission> path_mission_ptr_;
	};
}

#endif