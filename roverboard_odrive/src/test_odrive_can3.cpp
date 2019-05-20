#include <iostream>
#include <thread>

#include "roverboard_odrive/roverboard_odrive_can.h"


int main(int argc, const char * argv[]) {
  Odrive odrv = Odrive();
  odrv.init();

  //odrv.run();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  //for (auto i=0; i < 100; i++) {
    odrv.sendVelSetpoint(40);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    //}
  odrv.sendVelSetpoint(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  std::cerr << "shuting down" << std::endl;
  return 0;
}
