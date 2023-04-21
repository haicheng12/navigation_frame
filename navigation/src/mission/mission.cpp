#include "mission/mission.h"

using namespace Sakura::Logger;

namespace Nav
{
    Mission::Mission() // 构造函数
    {
        info("构造函数");
    }

    Mission::~Mission() // 析构函数
    {
        info("析构函数");
    }

}