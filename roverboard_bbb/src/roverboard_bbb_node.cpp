#include <iostream>
#include "odrive_cpp_sdk.h"

int main(int argc, const char * argv[]) {

  std::string odrive_serial_numbers[1] = {"0x303339373235510a330045"};
  std::string odrive_serial_numbers_map[2] = {"0x303339373235510a330045","0x303339373235510a330045"};
  int16_t zeroeth_radian_in_encoder_ticks_[2] = { -2, 0 };

  bool odrive_position_per_motor[2] = {false, true};
  float odrive_encoder_ticks_per_radian_per_motor[2] = { 57.2958 * (2048 * 4) / 360.0, 57.2958 * (2048 * 4) / 360.0 };
  odrive::CppSdk odrive_cpp_sdk(
				odrive_serial_numbers,
				1,
				odrive_serial_numbers_map,
				odrive_position_per_motor,
				odrive_encoder_ticks_per_radian_per_motor,
				2
				);
  std::cout << "odrive_cpp_sdk constructed" << std::endl;
  odrive_cpp_sdk.setZeroethRadianInEncoderTicks(zeroeth_radian_in_encoder_ticks_);

  int result = odrive_cpp_sdk.init();
  std::cout << "odrive_cpp_sdk.init got: " << result << std::endl;

  if (result == ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND) {
    std::cout << "odrive_cpp_sdk.init :: ODRIVE_SDK_ODRIVE_WITH_SERIAL_NUMBER_NOT_FOUND" << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
  
}
