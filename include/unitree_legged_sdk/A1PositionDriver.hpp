#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>

#include <unitree__legged_sdk/vectornav/vectornav.hpp>

#include <Eigen/Dense>

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class A1PositionDriver {
 public:
  A1PositionDriver(): control(LeggedType::A1, LOWLEVEL), udp() {
    control.InitCmdData(cmd);
  }

  ~A1PositionDriver();

  void Initialize();

  void UDPRecv();
  void UDPSend();
  void RobotControl();

  Control control;
  UDP udp;

  LowCmd cmd = {0};
  LowState state = {0};

  int motiontime = 0;
  float dt = 0.002;

  Vectornav* imu_;
  bool imu_b;

  Eigen::VectorXd starting_q;
  // Eigen::VectorXd finish_q;
  // Eigen::VectorXd des_q;

  // Gain values for the low level robot
  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;

  // Sensor Data from the Robot
  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  Eigen::VectorXd tau_;
  Eigen::VectorXi foot_force_;

  //Sensor Data from teh IMU
  Eigen::Vector3d rpy_, prev_rpy_;
  Eigen::Vector3d acc_, prev_acc_;
  Eigen::Vector3d ang_vel_, prev_ang_vel_;

 protected:
  double clamp_value(double in, double min, double max); 
  void compute_des_q();
  void extract_sensor_data();
  void set_command();
  void set_initial_command();

  double curr_time;
  double start_time;
};
