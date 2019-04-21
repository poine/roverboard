#include <boost/format.hpp>

#include "roverboard_bbb/roverboard_odrive.h"

Odrive::Odrive():
  serial_("/dev/ttyS5", 115200)
{}

Odrive::~Odrive() {
  serial_.close();
}

bool Odrive::init() {
  if (!serial_.init()) {
    std::printf("Failed to initialize serial port\n");
    return false;
  }
  std::printf("initialized serial port\n");
  return true;
}


void Odrive::send_velocity_setpoint(int m1, int m2) {
  std::string s = boost::str(boost::format("v 0 %d\r\n") % m1);
  //std::string s = "v 0 50\r\n" ;
  serial_.send_bytes(reinterpret_cast<const uint8_t* >(s.c_str()), s.length());
}

