#include "planner/rotation.h"

using namespace Sakura::Logger;

namespace Nav
{
    Rotation::Rotation() // 构造函数
    {
        info("构造函数");
    }

    Rotation::~Rotation() // 析构函数
    {
        exit();
        info("析构函数");
    }

    bool Rotation::initial()
    {
        info("初始化");
        return true;
    }
    Result Rotation::update()
    {
        info("循环执行");
        return Result::SUCCEED;
    }
    bool Rotation::exit()
    {
        info("退出");
        return true;
    }
}
