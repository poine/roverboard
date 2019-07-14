
#include "roverboard_odrive/roverboard_odrive_ascii.h"

#include <iostream>
#include <chrono>


int main(int argc, const char * argv[]) {

  OdriveAscii od;
  od.init();
  double enc[2], enc_vel[2];
  double iqsp[2], iqm[2];
  double duration = 5., dt=1./50; // seconds
  //double dt = 1./50;
  int nb_loop = int(duration/dt);
  double time[nb_loop];
  auto start = std::chrono::steady_clock::now();
  std::chrono::microseconds dt_chrono(int(dt*1e6));
  for (uint8_t i = 0; i < nb_loop; i++) {
    auto loop_start = std::chrono::steady_clock::now();
    time[i] = std::chrono::duration_cast<std::chrono::milliseconds>(loop_start-start).count()*1e-3;
    //od.send_velocity_setpoint(0, 90);
    double vsps[2] = {90., 90.}, iq_ff[2] = {0., 0.};
    od.sendVelSetpoints(vsps, iq_ff);

    od.readFeedback(enc, enc_vel, iqsp, iqm);
    //od.read_feedback(enc, enc_vel);
    std::cout << i*dt <<  " " << time[i] <<
      " encoders: " << enc[0] << " " << enc[1] << " ticks vel: " <<
      enc_vel[0] << " " << enc_vel[1] << " ticks/s" << std::endl;
    auto loop_end = std::chrono::steady_clock::now();
    //auto elapsed = loop_end - loop_start;
    //auto sleep_time = dt_chrono - elapsed;
    auto next_loop_time = start + (i+1)*dt_chrono;
    auto sleep_time = next_loop_time - loop_end;
    std::this_thread::sleep_for(sleep_time);
  }
  //od.send_velocity_setpoint(0, 0);
  double vsps[2] = {0., 0.}, iq_ff[2] = {0., 0.};
  od.sendVelSetpoints(vsps, iq_ff);
  return 0;
  
}
