
#include "roverboard_bbb/roverboard_odrive.h"





int main(int argc, const char * argv[]) {

  Odrive od;
  od.init();
  for (uint8_t i = 0; i < 100; i++) {
    od.send_velocity_setpoint(-10, -10);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return 0;
  
}
