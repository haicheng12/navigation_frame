#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
using namespace std;

int serial_write(serial::Serial &ser, std::string &serial_msg)
{
    ser.write(serial_msg);
    return 0;
}

int serial_read(serial::Serial &ser, std::string &result)
{
    result = ser.read(ser.available());
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "baseRun");

    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    serial::Serial ser;

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized.\n");
    }
    else
    {
        return -1;
    }
    // data 为发送数据
    // result 为接收数据
    std::string data, state, result;
    std::string data2;
    int func(0);
    /*****************************************************************************
     * 以下逻辑可以按照你自己的写，主要工作是为 data 赋值
     *****************************************************************************/
    // cout << "Please input function number:" << endl;
    // cout << "0:制动  1:前进  2:后退  3:向左平移  4:向右平移  5:左转  6:右转" << endl
    //      << endl;
    while (ros::ok())
    {
        cout << "Your function number is: ";
        cin >> func;
        switch (func)
        {
        case 0: // 制动
            data = "$,4,1,500,44,#\r\n";
            data2 = "test \r\n";
            // state = "制动";
            break;
        // case 1: // 前进
        //     data = "$,4,1,500,44,#\r\n";
        //     state = "前进";
        //     break;
        // case 2: // 后退
        //     data = "$,4,2,500,44,#\r\n";
        //     state = "后退";
        //     break;
        // case 3: // 向左平移
        //     data = "$,4,3,500,44,#\r\n";
        //     state = "向左平移";
        //     break;
        // case 4: // 向右平移
        //     data = "$,4,4,500,44,#\r\n";
        //     state = "向右平移";
        //     break;
        // case 5: // 左转
        //     data = "$,4,5,500,44,#\r\n";
        //     state = "左转";
        //     break;
        // case 6: // 右转
        //     data = "$,4,6,500,44,#\r\n";
        //     state = "右转";
        //     break;
        default:
            ROS_ERROR_STREAM("No this function number!!!");
            break;
        }
        /*****************************************************************************/
        // data = "test";
        //  串口写数据
        serial_write(ser, data2);
        // cout << " the data write to serial is :  " << data.c_str();
        cout << data2.c_str();

        // 串口读数据
        serial_read(ser, result);
        // cout << " the data read from serial is : " << result.c_str();
        cout << result.c_str();
        // cout << " the state of robot is : " << state.c_str() << endl
        //      << endl;
    }

    ser.close();
    return 0;
}
