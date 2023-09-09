#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
#include <thread>

#include "crc/crc8.h"
#include "serial_pack/serial_pack.hpp"
#include "serial_pack/serial_up.h"

using namespace std;
/*
    下方通信
    1. 四块挡板的控制
    2. 托盘的朝向
    3. 推杆的运动
*/
class serial_pack_up
{
private:
    serial_c serial;
    ros::NodeHandle nh;
    _up_send_data send_data;
    ros::Subscriber sub_up;

public:
    void doMsg(const serial_pack::serial_up::ConstPtr &msg);

    serial_pack_up(const std::string &port, uint32_t baudrate, ros::NodeHandle _nh) : serial(port, baudrate)
    {
        nh = _nh;
        sub_up = nh.subscribe<serial_pack::serial_up>("/up", 10, &serial_pack_up::doMsg, this);

        send_data.up_board.head = 0xFF;
        send_data.up_board.tail = 0xFE;
        send_data.up_board.tack_ID = 0xAE;
        send_data.up_board.up_board_text.x_move = 0;
        send_data.up_board.up_board_text.y_move = 0;
        send_data.up_board.up_board_text.grasp = 0;
        send_data.up_board.up_board_text.lift = 0;
        send_data.up_board.up_board_text.yaw = 0;
        send_data.up_board.crc8 = Get_CRC8_Check_Sum_UI((unsigned char *)&(send_data.up_board), 10, 0xFF);
    }
    ~serial_pack_up();
};

serial_pack_up::~serial_pack_up()
{
}

void serial_pack_up::doMsg(const serial_pack::serial_up::ConstPtr &msg)
{
    send_data.up_board.up_board_text.x_move = msg->x_move;
    send_data.up_board.up_board_text.y_move = msg->y_move;
    send_data.up_board.up_board_text.grasp = msg->grasp;
    send_data.up_board.up_board_text.lift = msg->lift;
    send_data.up_board.up_board_text.yaw = msg->yaw;
    send_data.up_board.crc8 = Get_CRC8_Check_Sum_UI((unsigned char *)&(send_data.up_board), 10, 0xFF);
    serial.ser.write((uint8_t *)&send_data, 12);
    for (size_t i = 0; i < 12; i++)
    {
        printf("%02X ",send_data.c[i]);
    }
    cout<<endl;
    ROS_INFO_STREAM("low receive");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "up_node");
    ros::NodeHandle _nh("~");
    std::string _serial_port;
    int _serial_baud;
    _nh.param("port", _serial_port, std::string("/dev/ttyCH9344USB1"));
    _nh.param("baud", _serial_baud, 460800);
    serial_pack_up a(_serial_port, (uint32_t)_serial_baud, _nh);
    ros::AsyncSpinner spinner(5);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
