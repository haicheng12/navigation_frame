#ifndef _MISSION_H_
#define _MISSION_H_

#include "logger/logger.h"

namespace Nav
{
	// 基类
	class Mission
	{
	public:
		Mission(){};
		virtual ~Mission(){};

		virtual bool initial() { return true; };
		virtual bool update() { return true; };
		virtual bool exit() { return true; };

	private:
	};
}

#endif