#include <unitree_legged_sdk/vectornav/vectornav.hpp>

using namespace std;
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
  tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
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

  // Write UART to VN100 to Tare the yaw data
  // $VNTAR*XX
  bool b_write = false;
  unsigned char tare[] = {'$', 'V', 'N', 'T', 'A', 'R', '*', 'X', 'X'};// , '3', '5'};
  // const char* cmdToSend = "$VNTAR*XX\0";
  b_write = write(serial_port, tare, sizeof(tare));
  // b_write = write(serial_port, cmdToSend, sizeof(cmdToSend+1));


  /*for(int i=0; i<5;++i) {
      for(int idx=0;idx<9;idx++)
      {
      //char sendChar = cmdToSend[idx];
      b_write= write(serial_port, cmdToSend+idx, sizeof(char));
      if (!b_write)exit(0);
      }
  }*/
  //if (!b_write) exit(0);
  // std::cout << "b_write = " << b_write << std::endl;

  // Allocate memory for read buffer, set size according to your needs
  char read_buf [256];
  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  memset(&read_buf, '\0', sizeof(read_buf));

  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
    printf("Error reading: %s", strerror(errno));
    exit(0);
  }

  printf("Read %i bytes. Received message: %s", num_bytes, read_buf);
  std::cout << std::endl;

}

Vectornav::~Vectornav() {
  close(serial_port);
}

int Vectornav::checksum(char chrData [116]) {
  int sumData = 0;
  for (int i=0; i<116; ++i) {
    // std::cout << "chrData[i] = " << chrData[i] << std::endl;
    sumData ^= chrData[i];
  }
  // std::cout << "---------------------------------" << std::endl;
  return sumData;
}


bool Vectornav::getData() {
  int valChecksum;
  int sumData;
  double raVal[4][3];
  char *pt;

  void* ptrRead;

  char chrStart;
  char chrData_single;
  char chrData[116];
  char chrEnd;
  char chrChecksum_single;
  char chrChecksum[2];

  int o = read(serial_port, &chrStart, 1);

  if (chrStart == '$') {
    for (int i=0; i<116; ++i) {
      int n = read(serial_port, &chrData_single, 1);
      // std::cout << "chrData_single = " << chrData_single << std::endl;
      chrData[i] = chrData_single;
    }
    // int n = read(serial_port, ptrRead, 116);
    // std::cout << "ptrRead = " << ptrRead << std::endl; 
    // std::cout << "n ptrRead = " << n << std::endl;
    // memcpy(chrData, ptrRead, 116);

    int q = read(serial_port, &chrEnd, 1);
  } else {return 0;}

  if (chrEnd == '*') {
    for (int i=0; i<2; ++i) {
      int r = read(serial_port, &chrChecksum_single, 1);
      chrChecksum[i] = chrChecksum_single;
    }
    // int r = read(serial_port, ptrRead, 2);
    // memcpy(chrChecksum, ptrRead, 2);

    valChecksum = 0;

    if (chrChecksum[0] <= '9')
        valChecksum += 16 * (chrChecksum[0] - '0');
    else
        valChecksum += 16 * (chrChecksum[0] - 'A' + 10);

    if (chrChecksum[1] <= '9')
        valChecksum +=(chrChecksum[1] - '0');
    else
        valChecksum +=(chrChecksum[1] - 'A' + 10);

    sumData = checksum(chrData);
    valChecksum %= 256;
  } else {return 0;}

  // std::cout << "chrStart = " << chrStart << std::endl;
  // std::cout << "chrEnd = " << chrEnd << std::endl;
  // std::cout << "chrChecksum = " << chrChecksum << std::endl;
  // std::cout << "chrData = " << chrData << std::endl;
  // std::cout << "sumData =  " << sumData << std::endl;
  // std::cout << "valChecksum =  " << valChecksum << std::endl;
  // std::cout << "---------------------------------" << std::endl;

  if (valChecksum == sumData) {

    pt = strtok (chrData, ",");
    // pt = strtok(NULL,",");
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
        // pt = strtok (chrData,",");
        // std::cout << "pt = " << pt << std::endl;
        raVal[i][j] = atof(pt); // 
        pt = strtok (NULL, ",");
      }
    }
  } else {return 0;}

  /*std::cout << "raVal[0][.] = " << raVal[0][1] << ", " << raVal[0][2] 
            << ", " <<raVal[0][3] << std::endl;
  std::cout << "raVal[1][.] = " << raVal[1][1] << ", " << raVal[1][2] 
            << ", " <<raVal[1][3] << std::endl;
  std::cout << "raVal[2][.] = " << raVal[2][1] << ", " << raVal[2][2] 
            << ", " <<raVal[2][3] << std::endl;
  std::cout << "raVal[3][.] = " << raVal[3][1] << ", " << raVal[3][2] 
            << ", " <<raVal[3][3] << std::endl;*/

  // Fill our Eigen Vectors
  raRPY[0] = raVal[0][3]; raRPY[1] = raVal[0][2]; raRPY[2] = raVal[0][1];
  raMag[0] = raVal[1][1]; raMag[1] = raVal[1][2]; raMag[2] = raVal[1][3];
  raAcc[0] = raVal[2][1]; raAcc[1] = raVal[2][2]; raAcc[2] = raVal[2][3];
  raGyro[0] = raVal[3][1]; raGyro[1] = raVal[3][2]; raGyro[2] = raVal[3][3];


  return 1;

  /*// Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int n = read(serial_port, &read_buf, sizeof(read_buf));
  // n is the number of bytes read. n may be 0 if no bytes 
  // were received, and can also be negative to signal an error.*/


}
