/* ------------------------------

Read asynchronous data stream from Vectornav VN-100 IMU

Code Based on Python version written originally by Mingyo Seo
From the Robot Perception and Learning Lab, The University of Texas at Austin

Author: Ryan Gupta
Human Centered Robotics Lab
University of Texas at Austin

------------------------------ */
// C++ Standard
#include <iostream>
#include <ctime>
#include <string>
#include <cstring>
// Serial Port Access
#include <termios.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
// Eigen
#include <Eigen/Dense>


class Vectornav {
 public:
  Vectornav();

  ~Vectornav();

  bool getData();

  // Structures for Holding IMU Data
  Eigen::Vector3d raRPY;
  Eigen::Vector3d raMag;
  Eigen::Vector3d raAcc;
  Eigen::Vector3d raGyro;

 protected:

  int checksum(char chrData [116]);

  // Const Values
  char const DEVICE [13] = "/dev/ttyUSB0";
  int const BAUDRATE = 115200;
  double const TIMEOUT = 0.1;
  
  // Vars for receiving serial data
  int serial_port;
  // Create new termios struct, we call it 'tty' for convention
  // No need for "= {0}" at the end as we'll immediately write the existing
  // config to this struct
  termios tty;

};



