#include "chassis/chassis.h"

using namespace Sakura::Logger;

namespace Nav
{
    Chassis::Chassis() : func_(0) // 构造函数
    {
        // info("构造函数");
    }

    Chassis::~Chassis() // 析构函数
    {
        // info("析构函数");
        ser_.close();
    }

    bool Chassis::initial() // 初始化
    {
        try
        {
            ser_.setPort("/dev/ttyUSB0");
            ser_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(to);
            ser_.open();
        }
        catch (serial::IOException &e)
        {
            fatal("Unable to open port ");
            return false;
        }

        if (ser_.isOpen())
        {
            info("Serial Port initialized.\n");
            return true;
        }
        else
        {
            return false;
        }
    }

    void Chassis::start()
    {
        // info("键盘输入数字1点亮STM32的LED灯，请输入：");
        // std::cin >> func_;
        // switch (func_)
        // {
        // case 1:
        //     data_ = "test \r\n";
        //     break;
        // default:
        //     fatal("输入错误，请重新输入！！！");
        //     break;
        // }

        data_ = "test \r\n";

        //  串口写数据
        serialWrite(ser_, data_);
        info("串口写入数据为：%s ", data_.c_str());

        // 串口读数据
        serialRead(ser_, result_);
        info("串口读取数据为：%s ", result_.c_str());
    }

    int Chassis::serialWrite(serial::Serial &ser, std::string &serial_msg)
    {
        ser_.write(serial_msg);
        return 0;
    }

    int Chassis::serialRead(serial::Serial &ser, std::string &result)
    {
        result = ser_.read(ser_.available());
        return 0;
    }
}
