#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
#include <thread>

#include "crc/crc8.h"
#include "serial_pack/serial_pack.hpp"
#include "serial_pack/serial_low.h"

using namespace std;
/*
    下方通信
    1. 四块挡板的控制
    2. 托盘的朝向
    3. 推杆的运动
*/
class serial_pack_low
{
private:
    serial_c serial;
    ros::NodeHandle nh;
    _low_send_data send_data;
    ros::Subscriber sub_low;

public:
    void doMsg(const serial_pack::serial_low::ConstPtr &msg);

    serial_pack_low(const std::string &port, uint32_t baudrate, ros::NodeHandle _nh) : serial(port, baudrate)
    {
        nh = _nh;
        sub_low = nh.subscribe<serial_pack::serial_low>("/low", 10, &serial_pack_low::doMsg, this);

        send_data.low_board.head = 0xFF;
        send_data.low_board.tail = 0xFE;
        send_data.low_board.tack_ID = 0xAD;
        send_data.low_board.low_board_text.db[0] = 0;
        send_data.low_board.low_board_text.db[1] = 0;
        send_data.low_board.low_board_text.db[2] = 0;
        send_data.low_board.low_board_text.db[3] = 0;
        send_data.low_board.low_board_text.tp = 0;
        send_data.low_board.low_board_text.tg = 0;
        send_data.low_board.crc8 = Get_CRC8_Check_Sum_UI((unsigned char *)&(send_data.low_board), 8, 0xFF);
    }
    ~serial_pack_low();
};

serial_pack_low::~serial_pack_low()
{
}

void serial_pack_low::doMsg(const serial_pack::serial_low::ConstPtr &msg)
{
    int flag = 0;
    send_data.low_board.low_board_text.db[0] = msg->db_1_flag;
    send_data.low_board.low_board_text.db[1] = msg->db_2_flag;
    send_data.low_board.low_board_text.db[2] = msg->db_3_flag;
    send_data.low_board.low_board_text.db[3] = msg->db_4_flag;
    send_data.low_board.low_board_text.tp = msg->tp_flag;
    send_data.low_board.low_board_text.tg = msg->tg_flag;
    send_data.low_board.crc8 = Get_CRC8_Check_Sum_UI((unsigned char *)&(send_data.low_board), 8, 0xFF);
    serial.ser.write((uint8_t *)&send_data, 10);
    for (size_t i = 0; i < 10; i++)
    {
        printf("%02X ",send_data.c[i]);
    }
    cout<<endl;
    ROS_INFO_STREAM("low receive");
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "low_node");
    ros::NodeHandle _nh("~");
    std::string _serial_port;
    int _serial_baud;
    _nh.param("port", _serial_port, std::string("/dev/ttyCH9344USB0"));
    _nh.param("baud", _serial_baud, 460800);
    serial_pack_low a(_serial_port, (uint32_t)_serial_baud, _nh);
    ros::AsyncSpinner spinner(5);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
