#ifndef ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_CAN_H
#define ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_CAN_H

#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>


class Odrive {
 public:
  Odrive();
  ~Odrive();
  bool init();
  //void run();
  void sendVelSetpoint(float vsp);

 private:
  can::DriverInterfaceSharedPtr g_driver_;
  //can::ThreadedSocketCANInterfaceSharedPtr g_driver_; 
  can::FrameListenerConstSharedPtr msg_listener_;
  can::StateListenerConstSharedPtr state_listener_;

  void msgCallback(const can::Frame &f);
  void stateCallback(const can::State & s);
};


#endif // ROVERBOARD_ODRIVE__ROVERBOARD_ODRIVE_CAN_H
