
#include "roverboard_odrive/roverboard_odrive_can.h"

#include <socketcan_interface/string.h>

#define ODRIVE_HEARTBEAT      0x001	
#define GET_ENCODER_ESTIMATES 0x009
#define SET_VEL_SETPOINT      0x00D
#define GET_IQ                0x014
#define REBOOT_ODRIVE         0x016
#define GET_VBUS_VOLTAGE      0x017

//#define DEBUG__
#ifdef DEBUG__
#define DEBUG_(_x) _x
#else
#define DEBUG_(_x)
#endif

OdriveCAN::OdriveCAN() {
  DEBUG_(std::cerr << "in OdriveCAN::OdriveCAN()" << std::endl;)
  g_driver_ = std::make_shared<can::ThreadedSocketCANInterface> ();
  enc_mutex_ = new std::mutex(); // FIXME: make C++42 do that automagically
}

OdriveCAN::~OdriveCAN() {
  DEBUG_(std::cerr << "in OdriveCAN::~Odrive()" << std::endl;)
  g_driver_->shutdown();
  g_driver_.reset();
  delete(enc_mutex_);  // FIXME: See above
}

bool OdriveCAN::init() {
  DEBUG_(std::cerr << "in OdriveCAN::init()" << std::endl;)
  msg_listener_ = g_driver_->createMsgListener(can::CommInterface::FrameDelegate(this, &OdriveCAN::msgCallback));
  state_listener_ = g_driver_->createStateListener(can::StateInterface::StateDelegate(this, &OdriveCAN::stateCallback));
  
  if(!g_driver_->init("can1", false)){
    stateCallback(g_driver_->getState());
    return false;
  }
  return true;
}


void OdriveCAN::readFeedback(double* enc, double* enc_vel, double* iq_sp, double* iq_meas) {
  // Send request for state variables
  reqEncoder(0);
  reqEncoder(1);
  reqIq(0);
  reqIq(1);
  // Retrieve previously stored Odrive state variables
  {
     std::lock_guard<std::mutex> guard(*enc_mutex_);
     memcpy(enc, enc_, 2*sizeof(double));
     memcpy(enc_vel, enc_vel_, 2*sizeof(double));
     memcpy(iq_sp, iq_sp_, 2*sizeof(double));
     memcpy(iq_meas, iq_meas_, 2*sizeof(double));
   }

}

void OdriveCAN::sendVelSetpoints(double* vsps, double* iq_ff) {
  for (auto i=0; i<ODRIVE_AXIS_NB; i++)
    sendVelSetpoint(i, vsps[i], iq_ff[i]);
}

  




void OdriveCAN::sendVelSetpoint(int node_id, float vsp, float iff) {
  can::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.is_error = false;
  f.id = SET_VEL_SETPOINT + (node_id<<5);
  f.dlc = 8;
  // for (uint8_t i=0; i < f.dlc; i++)
  //   {
  //     f.data[i] = 0;
  //   }

  int32_t sp = vsp/0.01;
  // std::cerr << " " << vsp << "->" << sp << "->";
  // char buf[4];
  // std::memcpy(buf, &sp, 4);
  // for(int i=0; i < 4; ++i){
  //     std::cout << std::hex << " " << (int) buf[i];
  //   }
  // std::cerr << std::endl;
  
  std::memcpy(&f.data[0], &sp, sizeof(sp));
  int32_t ff = iff/0.01;
  std::memcpy(&f.data[4], &ff, sizeof(ff));
  //can::Frame f = can::toframe("00D#000F0000");

  g_driver_->send(f);
}


void  OdriveCAN::reqEncoder(int node_id) { sendCANFrame(true, node_id, GET_ENCODER_ESTIMATES, 0, NULL);}
void  OdriveCAN::reqIq(int node_id) { sendCANFrame(true, node_id, GET_IQ, 0, NULL);}
void OdriveCAN::reboot() {sendCANFrame(false, 0, REBOOT_ODRIVE, 0, NULL);}


void OdriveCAN::sendCANFrame(bool rtr, int node_id, int msg_id, int payload_len, void* payload) {
  can::Frame f;
  f.is_extended = false;
  f.is_rtr = rtr;
  f.is_error = false;
  f.id = msg_id + (node_id<<5);
  f.dlc = payload_len;
  //std::memcpy(f.data, &payload, payload_len);
  g_driver_->send(f);
}
// void Odrive::run() {
//   g_driver_->run();
// }



void OdriveCAN::printCanFrame(const can::Frame &f) {
  if(f.is_error){
    std::cout << "E " << std::hex << f.id << std::dec;
  }else if(f.is_extended){
    std::cout << "e " << std::hex << f.id << std::dec;
  }else{
    std::cout << "s " << std::hex << f.id << std::dec;
  }

  std::cout << "\t";
  
  if(f.is_rtr){
    std::cout << "r";
  }else{
    std::cout << (int) f.dlc << std::hex;
    
    for(int i=0; i < f.dlc; ++i){
      std::cout << std::hex << " " << (int) f.data[i];
    }
  }
  std::cout << std::dec << std::endl;
}

void OdriveCAN::msgCallback(const can::Frame &f) {
  DEBUG_(std::cerr << "in Odrive:frame_cbk_()" << std::endl;)
  unsigned int node_id = f.id >> 5;
  unsigned int msg_id  = f.id & 0x1F;
  switch (msg_id) {
  case ODRIVE_HEARTBEAT:
    break;
  case GET_ENCODER_ESTIMATES: {
    DEBUG_(for(auto i=0; i<f.dlc; ++i) std::cout << std::hex << " " << (int) f.data[i];)
    float pos, vel;
    memcpy(&pos, &(f.data[0]), sizeof(float));
    memcpy(&vel, &(f.data[4]), sizeof(float));
    //DEBUG(std::cout << "encoders: " << node_id << " " << pos << " " << vel << std::endl;)
    {
      std::lock_guard<std::mutex> guard(*enc_mutex_);
      enc_[node_id] = pos;     // FIXME, only works with ids 0 and 1 :(
      enc_vel_[node_id] = vel;
    }
  }
    break;
  case GET_IQ: {
    float sp, meas;
    memcpy(&sp, &(f.data[0]), sizeof(float));
    memcpy(&meas, &(f.data[4]), sizeof(float));
    DEBUG_(std::cout << "Iq: " << node_id << " " << sp << " " << meas << std::endl;)
    {
      std::lock_guard<std::mutex> guard(*enc_mutex_);
      iq_sp_[node_id] = sp;       // FIXME, only works with ids 0 and 1 :(
      iq_meas_[node_id] = meas;
    }
  }
    break;
  default:
    std::cout << "unhandled msg: " << node_id << " " << msg_id << std::endl;    
  }

  
}

void OdriveCAN::stateCallback(const can::State & s) {
  //std::cerr << "stateCallback" << std::endl;
  std::string err;
  g_driver_->translateError(s.internal_error,err);
  std::cout << "stateCallback: state=" << s.driver_state << " internal_error=" << s.internal_error << "('" << err << "') asio: " << s.error_code << std::endl;
  if (s.internal_error == CAN_ERR_PROT) { // 
     std::cout << "stateCallback: rever prot err" << std::endl;
     g_driver_->recover();
  }

}

