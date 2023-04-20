#ifndef _FOLLOW_TRACK_H_
#define _FOLLOW_TRACK_H_

#include "planner.h"
#include <iostream>

enum FollowTrackState : u_int8_t // 跟线状态机
{
	RUN = 1,	 // 正常行驶
	ARRIVE_GOAL, // 到达终点
	STOP		 // 暂停行使
};

namespace Nav
{
	// 派生类
	class FollowTrack : public Planner
	{
	public:
		FollowTrack();
		virtual ~FollowTrack();

		bool initial() override;
		Result update() override;
		bool exit() override;

	private:
	};
}

#endif