#ifndef ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_CAN_H
#define ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_CAN_H

#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>
#include <mutex>

#define ODRIVE_AXIS_NB 2

class Odrive {
 public:
  Odrive();
  ~Odrive();
  bool init();
  //void run();
  void sendCANFrame(bool rtr, int node_id, int msg_id, int payload_len, void* payload);
  void printCanFrame(const can::Frame &f);
  void readFeedback(double* enc, double* enc_vel, double* iq_sp, double* iq_meas);
  void sendVelSetpoints(double* vsps, double* iq_ff);

  void sendVelSetpoint(int node_id, float vsp, float iff);
  void reqEncoder(int node_id);
  void reqIq(int node_id);
  
 private:
  std::mutex* enc_mutex_;
  double enc_[ODRIVE_AXIS_NB];
  double enc_vel_[ODRIVE_AXIS_NB];
  double iq_sp_[ODRIVE_AXIS_NB];
  double iq_meas_[ODRIVE_AXIS_NB];
  //can::DriverInterfaceSharedPtr g_driver_;
  can::ThreadedSocketCANInterfaceSharedPtr g_driver_; 
  can::FrameListenerConstSharedPtr msg_listener_;
  can::StateListenerConstSharedPtr state_listener_;

  void msgCallback(const can::Frame &f);
  void stateCallback(const can::State & s);
};


#endif // ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_CAN_H
