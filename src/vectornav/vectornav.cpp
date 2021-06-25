#include <unitree_legged_sdk/vectornav/vectornav.hpp>


Vectornav::Vectornav() {

  ///////////////////////////////////////////////////////////////////////////
  ///                                                                     ///
  /// Initialize Data Members to 0                                        ///
  //////////////////////////////////////////////////////////////////////////

  raRPY = Eigen::VectorXd::Zero(3);
  raMag = Eigen::VectorXd::Zero(3);
  raAcc = Eigen::VectorXd::Zero(3);
  raGyro = Eigen::VectorXd::Zero(3);

  ///////////////////////////////////////////////////////////////////////////
  ///                                                                     ///
  /// Set our Serial Port                                                 ///
  ///////////////////////////////////////////////////////////////////////////

  serial_port = open(DEVICE, O_RDWR);
  // Check for error
  if (serial_port < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
  }
  // Read in existing settings, and handle any error
  // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
  // must have been initialized with a call to tcgetattr() overwise behaviour
  // is undefined
  if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  ///////////////////////////////////////////////////////////////////////////
  ///                                                                     ///
  /// Set termios params                                                  ///
  ///////////////////////////////////////////////////////////////////////////

  // Control Modes
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
  tty.c_cflag &= ~CSIZE // Clear all the size bits, then use one of the statements below
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  // Local Modes
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  // Input Modes
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); 
                    // Disable any special handling of received bytes
  // Output Modes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // VMIN & VTIME
  // An important point to note is that VTIME means slightly different
  // things depending on what VMIN is. When VMIN is 0, VTIME specifies
  // a time-out from the start of the read() call. But when VMIN is > 0,
  // VTIME specifies the time-out from the start of the first received character.
  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), 
                           // returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;
  if (BAUDRATE == 115200) {
    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
  } else {
    std::cout << "BAUDRATE issue" << std::endl;
    exit(0);
  }
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }


}
