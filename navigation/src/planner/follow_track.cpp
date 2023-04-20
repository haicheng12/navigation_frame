#include "planner/follow_track.h"

using namespace Sakura::Logger;

namespace Nav
{
    FollowTrack::FollowTrack() // 构造函数
    {
        info("构造函数");
    }

    FollowTrack::~FollowTrack() // 析构函数
    {
        exit();
        info("析构函数");
    }

    bool FollowTrack::initial()
    {
        info("初始化");
        return true;
    }
    Result FollowTrack::update()
    {
        info("循环执行");
        return Result::SUCCEED;
    }
    bool FollowTrack::exit()
    {
        info("退出");
        return true;
    }
}
