#include <iostream>

#include <socketcan_interface/socketcan.h>

can::DriverInterfaceSharedPtr g_driver;

void print_frame(const can::Frame &f){

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

void print_error(const can::State & s){
    std::string err;
    g_driver->translateError(s.internal_error,err);
    std::cout << "ERROR: state=" << s.driver_state << " internal_error=" << s.internal_error << "('" << err << "') asio: " << s.error_code << std::endl;
}

int main(int argc, const char * argv[]) {

  g_driver = std::make_shared<can::SocketCANInterface>();
  can::FrameListenerConstSharedPtr frame_printer = g_driver->createMsgListener(print_frame);
  can::StateListenerConstSharedPtr error_printer = g_driver->createStateListener(print_error);
  if(!g_driver->init("can1", false)){
    print_error(g_driver->getState());
    return 1;
  }
  g_driver->run();
  
  g_driver->shutdown();
  g_driver.reset();
  
  return 0;
}
