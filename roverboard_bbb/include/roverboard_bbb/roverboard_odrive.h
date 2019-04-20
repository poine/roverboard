#ifndef ROVERBOARD_BBB__ODRIVE_H
#define ROVERBOARD_BBB__ODRIVE_H

#include <async_comm/serial.h>

class Odrive {
 public:
  Odrive();
  ~Odrive();
  bool init();
  void send_velocity_setpoint(int m1, int m2);
 private:
  async_comm::Serial serial_;
  
};
#endif // ROVERBOARD_BBB__ODRIVE_H
