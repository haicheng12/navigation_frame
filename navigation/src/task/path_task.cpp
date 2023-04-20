#include "task/path_task.h"

using namespace Sakura::Logger;

namespace Nav
{
    PathTask::PathTask() // 构造函数
    {
        info("构造函数");
    }

    PathTask::~PathTask() // 析构函数
    {
        exit();
        info("析构函数");
    }

    bool PathTask::initial()
    {
        info("初始化");
        return true;
    }
    bool PathTask::update()
    {
        info("循环执行");
        return true;
    }
    bool PathTask::exit()
    {
        info("退出");
        return true;
    }
}
