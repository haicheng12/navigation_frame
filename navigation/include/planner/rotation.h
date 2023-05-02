#ifndef _ROTATION_H_
#define _ROTATION_H_

#include "planner.h"
#include <iostream>

enum RotationState : u_int8_t // 旋转状态
{
	RUNNING = 1, // 正常行驶
	FINISH		 // 到达终点
};

namespace Nav
{
	// 派生类
	class Rotation : public Planner
	{
	public:
		Rotation();
		virtual ~Rotation();

		bool initial() override;
		Result update() override;
		bool exit() override;

	private:
	};
}

#endif