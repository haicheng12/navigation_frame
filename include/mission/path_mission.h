#ifndef _PATH_MISSION_H_
#define _PATH_MISSION_H_

#include "mission.h"

namespace Nav
{
	// 派生类
	class PathMission : public Mission
	{
	public:
		PathMission();
		virtual ~PathMission();

		bool initial() override;
		bool update() override;
		bool exit() override;

	private:
	};
}

#endif