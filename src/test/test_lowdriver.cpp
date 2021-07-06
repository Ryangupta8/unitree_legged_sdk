#include "unitree_legged_sdk/A1LowDriver.hpp"

using namespace UNITREE_LEGGED_SDK;


int main(void)
{
  std::cout << "Control level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  A1LowDriver custom;
  custom.InitializePnC();

  LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&A1LowDriver::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&A1LowDriver::UDPSend,      &custom));
  LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&A1LowDriver::UDPRecv,      &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while(1){
      sleep(10);
  };

  return 0; 
}
