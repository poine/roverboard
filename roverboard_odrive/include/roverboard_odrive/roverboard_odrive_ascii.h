#ifndef ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_ASCII_H
#define ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_ASCII_H

#include <async_comm/serial.h>

#define ODRIVE_BUF_LEN 256

#include "roverboard_odrive/roverboard_odrive.h"

enum Status { iddle, waiting_fb_0, waiting_fb_1};

class OdriveAscii: public Odrive {
 public:
  OdriveAscii();
  ~OdriveAscii();
  bool init();
  //void send_velocity_setpoint(int m1, int m2);
  //void read_feedback(double* enc, double* enc_vel);
  void readFeedback(double* enc, double* enc_vel, double* iq_sp, double* iq_meas);
  void sendVelSetpoints(double* vsps, double* iq_ff);
  void reboot();
  
 private:
  async_comm::Serial serial_;
  uint8_t buf_[ODRIVE_BUF_LEN];
  uint8_t buf_idx_;
  enum Status status_;
  void serial_callback(const uint8_t* buf, size_t len);
  void parse_byte(const uint8_t b);
  void parse_line();
  std::mutex enc_mutex_;
  double enc_[ODRIVE_AXIS_NB];
  double enc_vel_[ODRIVE_AXIS_NB];
  double iq_sp_[ODRIVE_AXIS_NB];
  double iq_meas_[ODRIVE_AXIS_NB];
};
#endif // ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_ASCII_H
