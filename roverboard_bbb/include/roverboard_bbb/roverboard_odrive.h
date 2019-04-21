#ifndef ROVERBOARD_BBB__ODRIVE_H
#define ROVERBOARD_BBB__ODRIVE_H

#include <async_comm/serial.h>

#define ODRIVE_BUF_LEN 128
#define ODRIVE_AXIS_NB 2


enum Status { iddle, waiting_fb_0, waiting_fb_1};

class Odrive {
 public:
  Odrive();
  ~Odrive();
  bool init();
  void send_velocity_setpoint(int m1, int m2);
  void read_feedback(double* enc, double* enc_vel);
 private:
  async_comm::Serial serial_;
  uint8_t buf_[ODRIVE_BUF_LEN];
  uint8_t buf_idx_;
  enum Status status_;
  void serial_callback(const uint8_t* buf, size_t len);
  void parse_byte(const uint8_t b);
  std::mutex enc_mutex_;
  double enc_[ODRIVE_AXIS_NB];
  double enc_vel_[ODRIVE_AXIS_NB];
};
#endif // ROVERBOARD_BBB__ODRIVE_H
