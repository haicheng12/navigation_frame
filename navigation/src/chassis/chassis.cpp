#include "chassis/chassis.h"

using namespace Sakura::Logger;

namespace Nav
{
    Chassis::Chassis() // 构造函数
    {
        info("构造函数");
        initial(); // 初始化
    }

    Chassis::~Chassis() // 析构函数
    {
        info("析构函数");

        // 关闭串口
        sp_.close();
    }

    bool Chassis::initial() // 初始化
    {
        // 创建timeout
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        // 设置要打开的串口名称
        sp_.setPort("/dev/ttyUSB0");
        // 设置串口通信的波特率
        sp_.setBaudrate(115200);
        // 串口设置timeout
        sp_.setTimeout(to);

        try
        {
            // 打开串口
            sp_.open();
        }
        catch (serial::IOException &e)
        {
            warn("Unable to open port.");
            return false;
        }

        // 判断串口是否打开成功
        if (sp_.isOpen())
        {
            info("/dev/ttyUSB0 is opened.");
            return true;
        }
        else
        {
            return false;
        }
        return true;
    }

    void Chassis::start()
    {
        // 获取缓冲区内的字节数
        size_t n = sp_.available();
        if (n != 0)
        {
            uint8_t buffer[1024];
            // 读出数据
            n = sp_.read(buffer, n);

            for (int i = 0; i < n; i++)
            {
                // 16进制的方式打印到屏幕
                info("std::hex %02x", (buffer[i] & 0xff));
            }
            info("\n");
            // 把数据发送回去
            sp_.write(buffer, n);
        }
    }

}
