#include <iostream>
#include <thread>

#include "roverboard_odrive/roverboard_odrive_can.h"


int main(int argc, const char * argv[]) {
  Odrive odrv = Odrive();
  odrv.init();
  odrv.reboot(); // for reseting watchdog
  std::cerr << "odrive initialized" << std::endl;
  //odrv.run();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  //odrv.sendVelSetpoint(1, 50., 0.);
  double vsps[2] = {-50., 0.};
  double iffs[2] = {0., 0.};
  double pos[2], vel[2];
  double iqsp[2], iqm[2];
  double duration = 5.; // seconds
  double dt = 1./50;
  int nb_loop = int(duration/dt);
  double time[nb_loop];
  auto start = std::chrono::steady_clock::now();
  std::chrono::microseconds dt_chrono(int(dt*1e6));
  for (auto i=0; i < nb_loop; i++) {
    auto loop_start = std::chrono::steady_clock::now();
    time[i] = std::chrono::duration_cast<std::chrono::milliseconds>(loop_start-start).count()*1e-3;
    odrv.sendVelSetpoints(vsps, iffs);
    odrv.readFeedback(pos, vel, iqsp, iqm);
    std::cout << time[i] << " encoders: " << pos[0] << " " << pos[1] << " " << vel[0] << " " << vel[1] << " currents: " << iqsp[0] << " " << iqsp[1] << std::endl;

    auto loop_end = std::chrono::steady_clock::now();
    auto next_loop_time = start + (i+1)*dt_chrono;
    auto sleep_time = next_loop_time - loop_end;
    std::this_thread::sleep_for(sleep_time);
  }

  odrv.sendVelSetpoint(1, 0, 0);
  double vsps2[2] = {0., 0.};
  double iffs2[2] = {0., 0.};
  odrv.sendVelSetpoints(vsps2, iffs2);
 
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  std::cerr << "shuting down" << std::endl;
  return 0;
}

