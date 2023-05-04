#ifndef _STATE_H_
#define _STATE_H_

#include <iostream>
#include "logger/logger.h"

// 状态机状态枚举类型
enum class State : uint8_t
{
	IDEL,	// 空闲
	RECORD, // 录制线路
	FOLLOW, // 跟线
	STOP	// 暂停
};

// 状态机事件枚举类型
enum class Event : uint8_t
{
	START_EVENT, // 事件开始
	STOP_EVENT	 // 事件停止
};

using namespace Sakura::Logger;

namespace Nav
{
	class StateMachine // 状态机类
	{
	public:
		StateMachine();
		virtual ~StateMachine();

		// 处理事件
		void handleEvent(Event event);

	private:
		State currentState;
	};
}

#endif
