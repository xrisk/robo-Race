#ifndef DXL_H
#define DXL_H
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <vector>
#include <string>
#include <iostream>
#include <fcntl.h>
class Dxl{
  dynamixel::PortHandler *portHandler;
  dynamixel::PacketHandler *packetHandler;
  int dxl_comm_result;
  uint8_t dxl_error;
  uint16_t dxl_present_pos;

  public:
    Dxl(std::string);
    std::vector<int> scan(int);
    int write(int, float, int);
    float read(int);
    int get_present_speed(int);
    int set_moving_speed(int, int);
};


std::vector<std::string> get_available_ports(std::string);

#endif
