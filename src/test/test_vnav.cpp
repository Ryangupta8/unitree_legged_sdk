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

    if(imu.getData()) std::cout << "Mingyo did it" << std::endl;
  }
}
