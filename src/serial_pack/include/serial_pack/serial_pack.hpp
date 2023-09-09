#include "crc/crc8.h"
#include "serial_pack/serial_pack.h"
#include <serial/serial.h>
#include <ros/ros.h>

using namespace std;
class serial_c
{
private:
    // serial_c(const serial_c &);
    // serial_c &operator=(const serial_c &);

public:
    serial::Serial ser;
    serial_c(const std::string &port = "",uint32_t baudrate = 9600);
    ~serial_c();
};

serial_c::serial_c(const std::string &port,uint32_t baudrate)
{
    ser.setPort(port);                                         // 设置打开的串口名称
    ser.setBaudrate(baudrate);                                 // 设置串口的波特率
    serial::Timeout to = serial::Timeout::simpleTimeout(5000); // 创建timeout
    ser.setTimeout(to);
    try
    {
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open RS485 port:" + port);
        exit(0);
    }
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("RS485 Port:" + port + " initialized.");
    }
    else
    {
        ROS_ERROR_STREAM("Unable to initial RS485 port:" + port);
        exit(0);
    }
}

serial_c::~serial_c()
{
}
