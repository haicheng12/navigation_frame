#ifndef _MISSION_H_
#define _MISSION_H_

#include <iostream>
#include "logger/logger.h"

enum missionState : u_int8_t // 任务状态
{
	IDEL = 1,	 // 空闲
	RECORD_PATH, // 录制线路
	PUB_PATH	 // 发布线路
};

namespace Nav
{
	class Mission
	{
	public:
		Mission();
		virtual ~Mission();

	private:
	};
}

#endif
