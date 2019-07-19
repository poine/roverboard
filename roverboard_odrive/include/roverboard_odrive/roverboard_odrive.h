#ifndef ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_H
#define ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_H

#define ODRIVE_AXIS_NB 2

class Odrive {
 public:
  virtual bool init() = 0;
  virtual void reboot() = 0;
  virtual void sendVelSetpoints(double* vsps, double* iq_ff) = 0;
  virtual void readFeedback(double* enc, double* enc_vel, double* iq_sp, double* iq_meas) = 0;
};


#endif // ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_H
