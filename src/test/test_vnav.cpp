#include <unitree_legged_sdk/vectornav/vectornav.hpp>

class Vectornav;

int main () {

  Vectornav imu;
  bool b;
  while (true) {
    b = imu.getData();

    /* if (b) {
      std::cout << "hello world" << std::endl;
    }*/ 

    if(imu.getData()) {
      std::cout << "IMU RPY = " << imu.raRPY[0] << ", " << imu.raRPY[1] << ", " << imu.raRPY[2] << std::endl;
      std::cout << "IMU Ang Vel = " << imu.raGyro[0] << ", " << imu.raGyro[1] << ", " << imu.raGyro[2] << std::endl;
    }
  }
}
