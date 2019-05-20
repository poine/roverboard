
#include "roverboard_bbb/roverboard_odrive.h"

#include <iostream>



int main(int argc, const char * argv[]) {

  Odrive od;
  od.init();
  double enc[2], enc_vel[2];
  for (uint8_t i = 0; i < 100; i++) {
    od.send_velocity_setpoint(90, 0);
    od.read_feedback(enc, enc_vel);
    std::cout << "encoders: " << enc[0] << " " << enc[1] << " ticks vel: " <<
      enc_vel[0] << " " << enc_vel[1] << " ticks/s" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  od.send_velocity_setpoint(0, 0);
  return 0;
  
}
