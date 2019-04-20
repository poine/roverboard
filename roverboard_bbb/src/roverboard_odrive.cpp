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
  std::string s = "v 0 100\r\n" ;
  const char* cs = s.c_str();
  const uint8_t* cs1 = reinterpret_cast<const uint8_t* >(cs);
  serial_.send_bytes(cs1, s.length());
}

