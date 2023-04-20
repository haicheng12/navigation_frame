#include "message/message.h"

using namespace Sakura::Logger;

namespace Nav
{
    Message::Message() // 构造函数
    {
        info("构造函数");
    }

    Message::~Message() // 析构函数
    {
        info("析构函数");
    }
}