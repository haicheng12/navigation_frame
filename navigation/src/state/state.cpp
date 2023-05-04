#include "state/state.h"

namespace Nav
{
    StateMachine::StateMachine() : currentState(State::IDEL) // 初始化为一个状态
    {
    }

    StateMachine ::~StateMachine()
    {
    }

    // 处理事件
    void StateMachine::handleEvent(Event event) // 通过界面切换事件状态
    {
        switch (currentState)
        {
        case State::IDEL:
            if (event == Event::START_EVENT)
            {
                info("空闲状态");
                // currentState = State::RUNNING;
            }
            break;
        case State::RECORD:
            if (event == Event::START_EVENT)
            {
                info("录制线路状态");
                // currentState = State::STOP;
            }
            break;
        case State::FOLLOW:
            if (event == Event::START_EVENT)
            {
                info("跟线行使状态");
            }
            break;
        case State::STOP:
            if (event == Event::START_EVENT)
            {
                info("停车状态");
            }
            break;
        }
    }
}