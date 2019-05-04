#include <iostream>
#include <boost/format.hpp>

#include "roverboard_bbb/roverboard_odrive.h"




Odrive::Odrive():
  //serial_("/dev/ttyACM0", 115200),
  serial_("/dev/ttyACM0", 460800),
  buf_idx_(0),
  status_(iddle)
{
  serial_.register_receive_callback(std::bind(&Odrive::serial_callback, this, std::placeholders::_1, std::placeholders::_2));
}

Odrive::~Odrive() {
  serial_.close();
}

bool Odrive::init() {
  if (!serial_.init()) {
    std::printf("Failed to initialize serial port\n");
    return false;
  }
  std::printf("initialized serial port\n");
  return true;
}

void Odrive::serial_callback(const uint8_t* buf, size_t len) {
  //std::cout << "Received " << len << ": " << buf;
  //std::cout << "thread " << std::this_thread::get_id() << " in serial_callback...\n";
  for (size_t i = 0; i < len; i++)
    parse_byte(buf[i]);
}

void Odrive::parse_byte(const uint8_t b) {
  if (buf_idx_ < ODRIVE_BUF_LEN-1) {
    buf_[buf_idx_] = b;
    buf_idx_ += 1;
    if (buf_[buf_idx_-1] == '\n') {
      buf_[buf_idx_] = 0;
      //std::cout << "parsing buf (sta "<< status_ << "): " << buf_ << std::endl;
      switch (status_) {
      case waiting_fb_0: {
	std::lock_guard<std::mutex> guard(enc_mutex_);
	sscanf(reinterpret_cast<const char*>(buf_),"%lf %lf",&enc_[0], &enc_vel_[0]);
	std::string s2 = "f 1\r\n" ;
	serial_.send_bytes(reinterpret_cast<const uint8_t*>(s2.c_str()), s2.length());
	status_ = waiting_fb_1;
	break;
      }
      case waiting_fb_1: {
	std::lock_guard<std::mutex> guard(enc_mutex_);
	sscanf(reinterpret_cast<const char*>(buf_),"%lf %lf",&enc_[1], &enc_vel_[1]);
	status_ = iddle;
      }
      }
      buf_idx_ = 0;
    }
  }
  else {
    // buff overrun
    buf_idx_ = 0;
    
  }
}

			     
void Odrive::send_velocity_setpoint(int m1, int m2) {
  std::string s = boost::str(boost::format("v 0 %d\r\n") % m1);
  //std::cout << "sending: " << s << std::endl;
  serial_.send_bytes(reinterpret_cast<const uint8_t*>(s.c_str()), s.length());
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  std::string s2 = boost::str(boost::format("v 1 %d\r\n") % m2);
  //std::cout << "sending: " << s2 << std::endl;
  serial_.send_bytes(reinterpret_cast<const uint8_t*>(s2.c_str()), s2.length());
}

void Odrive::read_feedback(double* enc, double* enc_vel) {
  {
    std::lock_guard<std::mutex> guard(enc_mutex_);
    memcpy(enc, enc_, 2*sizeof(double));
    memcpy(enc_vel, enc_vel_, 2*sizeof(double));
  }
  
  std::string s = "f 0\r\n" ;
  //std::cout << "sending: " << s << std::endl;
  //std::cout << "thread " << std::this_thread::get_id() << " in read_feedback...\n";
  serial_.send_bytes(reinterpret_cast<const uint8_t*>(s.c_str()), s.length());
  //std::string s2 = "f 1\r\n" ;
  //serial_.send_bytes(reinterpret_cast<const uint8_t*>(s2.c_str()), s2.length());
  status_ = waiting_fb_0;
}
