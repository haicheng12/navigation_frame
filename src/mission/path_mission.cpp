#include "mission/path_mission.h"

using namespace Sakura::Logger;

namespace Nav
{
    PathMission::PathMission() // 构造函数
    {
        info("构造函数");
    }

    PathMission::~PathMission() // 析构函数
    {
        exit();
        info("析构函数");
    }

    bool PathMission::initial()
    {
        info("初始化");
        return true;
    }
    bool PathMission::update()
    {
        info("循环执行");
        return true;
    }
    bool PathMission::exit()
    {
        info("退出");
        return true;
    }
}