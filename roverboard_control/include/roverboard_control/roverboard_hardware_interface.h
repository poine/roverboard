#ifndef ROVERBOARD_BBB__ROVERBOARD_HARDWARE_INTERFACE_H
#define ROVERBOARD_BBB__ROVERBOARD_HARDWARE_INTERFACE_H

//
// This is ROS compatible hardware interface that uses beaglebone robotic hardware
// ( IMU, encoders) and a Odrive motor controller
//

#include <ros/ros.h>
// ros_control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>


//#ifdef ODRIVE_SERIAL_ASCII
//#warning "hardware interface using odrive serial ascii protocol"
#include "roverboard_odrive/roverboard_odrive_ascii.h"
//#else
//#warning "hardware interface using odrive can protocol"
#include "roverboard_odrive/roverboard_odrive_can.h"
#include "roverboard_odrive/roverboard_odrive.h"
//#endif

#define NB_JOINTS 2
//#define RVHI_DT 1e-2  // 100hz
//#define RVHI_DT 2e-2  // 50hz

class RoverBoardHardwareInterface : public hardware_interface::RobotHW
{
 public:
  RoverBoardHardwareInterface(int odrive_interface_type);
  virtual ~RoverBoardHardwareInterface();
  bool start(const ros::Time& time);
  void read(const ros::Time& time);
  void write();
  bool shutdown();
 private:
  // Joints
  double joint_position_[NB_JOINTS];
  double joint_velocity_[NB_JOINTS];
  double joint_effort_[NB_JOINTS];
  double joint_velocity_command_[NB_JOINTS];
  //double joint_effort_command_[NB_JOINTS];
 
  // for publishing joints state
  hardware_interface::JointStateInterface    js_interface_;
  // for accepting joints setpoints
  //hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;

  // Odrive motor controller
  //OdriveAscii od_;
  //OdriveAscii* oda_;
  //OdriveCAN*   odc_;
  Odrive* od_;
  
};

//
// odrive channel 0 right wheel, positive
// odrive channel 1 left wheel, negative
//

// caroline
#define ODRV_LW_CHAN 0
#define ODRV_RW_CHAN 1
#define ODRV_LW_POL  1.
#define ODRV_RW_POL -1.

#endif // ROVERBOARD_BBB__ROVERBOARD_HARDWARE_INTERFACE_H
