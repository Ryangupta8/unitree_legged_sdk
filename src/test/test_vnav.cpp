#include <unitree_legged_sdk/vectornav/vectornav.hpp>

class Vectornav;

int main () {

  while (true) {
    Vectornav imu;
    std::cout << "Object created" << std::endl;
    imu.getData();

    /* if (b) {
      std::cout << "hello world" << std::endl;
    }*/ 

    // if(imu.getData()) std::cout << "Mingyo did it" << std::endl;
  }
}
