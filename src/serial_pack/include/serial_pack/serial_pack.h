#include <iostream>
#include <string>
#include <sstream>

#pragma pack(1)
struct _up_board_text // 8 bytes
{
    uint16_t x_move; // 1=0.0381mm
    uint16_t y_move; // 1=0.0381mm
    uint16_t yaw;    // 360度/65535*yaw
    uint8_t grasp;   // 90度 /255  *grasp
    uint8_t lift;    // 0/1
};
#pragma pack()

#pragma pack(1)
struct _low_board_text // 6 bytes
{
    uint8_t db[4]; // 挡板 1挡起 0放下
    uint8_t tp;    // 托盘 1/2/3/4号垃圾桶朝向
    uint8_t tg;    // 推杆 1推出 0缩回
};
#pragma pack()

#pragma pack(1)
struct _up_board_t  // 12 bytes
{
    uint8_t head;  // 0xFF
    uint8_t tack_ID; // 0xAE
    _up_board_text up_board_text;
    uint8_t crc8;
    uint8_t tail;  // 0xFE
};
#pragma pack()

#pragma pack(1)
struct _low_board_t // 10 bytes
{
    uint8_t head;  // 0xFF
    uint8_t tack_ID; // 0xAD
    _low_board_text low_board_text;
    uint8_t crc8;
    uint8_t tail;  // 0xFE
};
#pragma pack()

#pragma pack(1)
union _up_send_data
{
  _up_board_t up_board;
  uint8_t c[12];
};
#pragma pack()

#pragma pack(1)
union _low_send_data
{
  _low_board_t low_board;
  uint8_t c[10];
};
#pragma pack()