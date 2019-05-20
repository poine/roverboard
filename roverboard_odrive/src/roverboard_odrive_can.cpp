#include "roverboard_odrive/roverboard_odrive_can.h"

#include <socketcan_interface/string.h>

Odrive::Odrive() {
  std::cerr << "in Odrive::Odrive()" << std::endl;
  //g_driver_ = std::make_shared<can::SocketCANInterface>();
  g_driver_ = std::make_shared<can::ThreadedSocketCANInterface> ();
}

Odrive::~Odrive() {
  std::cerr << "in Odrive::~Odrive()" << std::endl;
  g_driver_->shutdown();
  g_driver_.reset();
}

bool Odrive::init() {
  std::cerr << "in Odrive::init()" << std::endl;

  msg_listener_ = g_driver_->createMsgListener(can::CommInterface::FrameDelegate(this, &Odrive::msgCallback));
  state_listener_ = g_driver_->createStateListener(can::StateInterface::StateDelegate(this, &Odrive::stateCallback));
  
  if(!g_driver_->init("can1", false)){
    stateCallback(g_driver_->getState());
    return false;
  }
  return true;
}


void Odrive::sendVelSetpoint(float vsp) {
  can::Frame f;
  f.is_extended = false;
  f.is_rtr = false;
  f.is_error = false;
  f.id = 0x00D;
  f.dlc = 8;
  // for (uint8_t i=0; i < f.dlc; i++)
  //   {
  //     f.data[i] = 0;
  //   }

  int32_t sp = vsp/0.01;//5000;
#if 0
  f.data[0] = sp;
  f.data[1] = sp >>8;
  f.data[2] = sp >>16;
  f.data[3] = sp >>24;
  f.data[4] = 0;
  f.data[5] = 0;
  f.data[6] = 0;
  f.data[7] = 0;
#else
  std::memcpy(&f.data[0], &sp, sizeof(sp));
#endif
  //can::Frame f = can::toframe("00D#000F0000");

  g_driver_->send(f);
}


// void Odrive::run() {
//   g_driver_->run();
// }



void Odrive::msgCallback(const can::Frame &f) {
  //std::cerr << "in Odrive:frame_cbk_()" << std::endl;
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

void Odrive::stateCallback(const can::State & s) {
  //std::cerr << "stateCallback" << std::endl;
  std::string err;
  g_driver_->translateError(s.internal_error,err);
  std::cout << "stateCallback: state=" << s.driver_state << " internal_error=" << s.internal_error << "('" << err << "') asio: " << s.error_code << std::endl;
}

