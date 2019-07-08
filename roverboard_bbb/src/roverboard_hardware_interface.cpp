#include <roverboard_bbb/roverboard_hardware_interface.h>
#include <controller_manager/controller_manager.h>

#define __NAME "roverboard_hardware_interface"
const std::string joint_name_[NB_JOINTS] = {"left_wheel_joint","right_wheel_joint"};

RoverBoardHardwareInterface::RoverBoardHardwareInterface() {
  ROS_INFO_STREAM_NAMED(__NAME, "in RoverBoardHardwareInterface::RoverBoardHardwareInterface...");

  ROS_INFO_STREAM_NAMED(__NAME, "Registering interfaces");
  // register joints
  for (int i=0; i<NB_JOINTS; i++) {
    joint_position_[i] = joint_velocity_[i] = joint_effort_[i] = 0.;
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
      joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
    joint_velocity_command_[i] = 0.;
    vj_interface_.registerHandle(hardware_interface::JointHandle(
	js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));
  }
  registerInterface(&js_interface_);
  registerInterface(&vj_interface_);
  // start odrive
  od_.init();
}

RoverBoardHardwareInterface::~RoverBoardHardwareInterface() {
  ROS_INFO_STREAM_NAMED(__NAME, "in RoverBoardHardwareInterface::~RoverBoardHardwareInterface");
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
bool RoverBoardHardwareInterface::start(const ros::Time& time) {
  od_.reboot();
  return true;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
bool RoverBoardHardwareInterface::shutdown() {
  ROS_INFO("in RoverBoardHardwareInterface::shutdown");

  return true;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
#define RAD_PER_TICK (2*M_PI/90)
void RoverBoardHardwareInterface::read(const ros::Time& now) {
  double enc[2], enc_vel[2];
  double iqsp[2], iqm[2];
  od_.readFeedback(enc, enc_vel, iqsp, iqm);
  joint_position_[0] = enc[ODRV_LW_CHAN]*RAD_PER_TICK*ODRV_LW_POL;
  joint_position_[1] = enc[ODRV_RW_CHAN]*RAD_PER_TICK*ODRV_RW_POL;
  joint_velocity_[0] = enc_vel[ODRV_LW_CHAN]*RAD_PER_TICK*ODRV_LW_POL;
  joint_velocity_[1] = enc_vel[ODRV_RW_CHAN]*RAD_PER_TICK*ODRV_RW_POL;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
#define TICK_PER_RAD (1./RAD_PER_TICK)
void RoverBoardHardwareInterface::write() {
  double sp_tps_left = joint_velocity_command_[0]*TICK_PER_RAD*ODRV_LW_POL;
  double sp_tps_right = joint_velocity_command_[1]*TICK_PER_RAD*ODRV_RW_POL;
  double current_ff_left = 0., current_ff_right = 0.;
#ifdef ODRIVE_SERIAL_ASCII
  od_.send_velocity_setpoint(sp_tps_right, sp_tps_left, current_ff_right, current_ff_left);
#else
  double vsps[2] = {sp_tps_right, sp_tps_left};
  double iffs[2] = {current_ff_right, current_ff_left};
  od_.sendVelSetpoints(vsps, iffs);
#endif
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, __NAME);
  ROS_INFO_STREAM_NAMED(__NAME, "roverboard hardware node starting...");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  RoverBoardHardwareInterface hw;
  if (!hw.start(ros::Time::now())) {
    ROS_ERROR_STREAM_NAMED(__NAME, "Failed to initialize hardware. bailling out...");
    return -1;
  }

  ros::NodeHandle nh;
  controller_manager::ControllerManager cm(&hw, nh);
  ros::Duration period(RVHI_DT);
  while (ros::ok()) {
    ros::Time now = ros::Time::now();
    hw.read(now);
    cm.update(now, period);
    hw.write();
    period.sleep();
  }

  
  ROS_INFO_STREAM_NAMED(__NAME, "roverboard hardware node exiting...");
  hw.shutdown();
  return 0;
}
